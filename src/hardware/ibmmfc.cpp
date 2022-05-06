#include "dosbox.h"
#include "mixer.h"
#include "inout.h"
#include "pic.h"
#include "setup.h"

#include "fmgen/opp.h"
#include "z80ex/include/z80ex.h"

#include <cstring>

#define FMGEN_FREQ 11800000uL
#define FMCLK (FMGEN_FREQ/3)
#define CPU_FREQ (FMGEN_FREQ/2)
#define TMRCLK_A 500000uL
#define TMRCLK_B 2000000uL
#define TMRA_PRESC (TMRCLK_B/TMRCLK_A)

static FM::OPP fmchip;
static Z80EX_CONTEXT *fmcpu;

static Z80EX_BYTE rom[0x8000];
static Z80EX_BYTE ram[0x4000];
static int fmcpu_int;

static Bit32u imfcBase;
static Bitu imfcIrq;
static Bitu sampleRate;
static MixerChannel *chan;
static unsigned buf_pos, buf_lastpos;

static Bit8u pc_tcr;
static Bit8u pc_timers_irq;

static void update_irq();

static void opp_process()
{
	if ( buf_pos <= buf_lastpos )
		return;

	fmchip.Mix( ((FM::Sample *)MixTemp) + buf_lastpos * 2, buf_pos - buf_lastpos,
		sampleRate, (BYTE *)MixTemp, (BYTE *)(MixTemp + sizeof(MixTemp)) );

	buf_lastpos = buf_pos;
}

static unsigned char uart_buf[64];
static unsigned uart_buf_pos = 0;
static Bit8u uart_cmd = 0;
static pthread_mutex_t uart_buf_mutex = PTHREAD_MUTEX_INITIALIZER;

void uart_queue( unsigned char *buf, int len )
{//fprintf(stderr,"midi q %d\n",len);
	pthread_mutex_lock( &uart_buf_mutex );
	if ( len + uart_buf_pos > 64 )
		len = 64 - uart_buf_pos;
	memcpy( uart_buf + uart_buf_pos, buf, len );
	uart_buf_pos += len;
	pthread_mutex_unlock( &uart_buf_mutex );
}

static int uart_int()
{
	int act = 0;
	
	if ( uart_cmd & 0x01 )
		if ( (pc_tcr & 0x10) == 0 )
			act = 1;
//if(act)fprintf(stderr,"uart %u %x %x\n",act,uart_cmd,pc_tcr);
	return act;
}

static void uart_write( unsigned addr, Bit8u val )
{
	static unsigned state = 0;
	
	if ( addr )
	{
		if ( state < 3 )
			state ++;
		else
		{
			uart_cmd = val;
			if ( val & 0x40 )
				state = 0;
		}
	}
//fprintf(stderr,"uart %d: %x, %x %x\n",addr,val,uart_cmd,pc_tcr);
}

static Bit8u uart_read( unsigned addr )
{
	Bit8u val = 0;

	pthread_mutex_lock( &uart_buf_mutex );
	if ( addr & 1 )
	{
		if ( uart_buf_pos < 64 )
			val |= 0x01;
		if ( uart_buf_pos )
			val |= 0x02;// | 0x40;
		else
			val |= 0x04;
		
		if ( (pc_tcr & 0x10) == 0 )
			val |= 0x80;
		//fprintf(stderr,"uart stat: %x\n",val);
	}
	else
	{
		if ( uart_buf_pos )
		{
			val = uart_buf[0];
			uart_buf_pos --;
			memmove( uart_buf, uart_buf + 1, uart_buf_pos );
		}
		//fprintf(stderr,"uart data: %x\n",val);
	}
	pthread_mutex_unlock( &uart_buf_mutex );

	return val;
}

static struct
{
	Bit8u mode;
	unsigned latch;
	int byte;
} pit[3];

static int tclks_per_sample;
static int timera_count;
static long long timerb_count;

static void pit_write( unsigned addr, Bit8u val )
{
	if ( addr == 3 )
	{
		if ( (val & 0xD) != 0x4 )
			fprintf(stderr,"%s %u: unsupported timer mode %x\n",__FUNCTION__,__LINE__,val);
		pit[val >> 6].mode = val & 0x3F;
		pit[val >> 6].byte = val & 0x10 ? 0 : 1;
	}
	else
	{
		if ( pit[addr].byte )
		{
			pit[addr].latch &= 0x00FF;
			pit[addr].latch |= val << 8;
		}
		else
		{
			pit[addr].latch &= 0xFF00;
			pit[addr].latch |= val;
		}
fprintf(stderr,"%s %u: %u %x\n",__FUNCTION__,__LINE__,addr,pit[addr].latch);
		if ( pit[addr].mode & 0x20 )
			pit[addr].byte = !pit[addr].byte;
		
		if ( addr == 0 )
			timera_count = 0;
		else
			timerb_count = 0;
	}
}

static Bit8u pit_read( unsigned addr )
{
	return 0;
}

static void process_timers( Bitu len )
{
	static int ta_presc;
	unsigned tb_clks = len * tclks_per_sample;
	unsigned ta_clks = 0;
	
	ta_presc += tb_clks;
	if ( ta_presc >= TMRA_PRESC )
	{
		ta_clks = ta_presc / TMRA_PRESC;
		ta_presc %= TMRA_PRESC;
	}
	ta_clks = tb_clks;
	
	if ( ta_clks > 0 )
	{//fprintf(stderr,"%s %u %x %x\n",__FUNCTION__,__LINE__,timera_count,ta_clks);
		timera_count -= ta_clks;
		while ( timera_count <= 0 )
		{//fprintf(stderr,"%s %u %d\n",__FUNCTION__,__LINE__,timera_count);
			timera_count += pit[0].latch ? pit[0].latch : 0x10000;
			if ( pc_tcr & 0x04 ) 
				pc_timers_irq |= 1;
		}//fprintf(stderr,"%s %u %d\n",__FUNCTION__,__LINE__,timera_count);
	}
	
	if ( tb_clks > 0 )
	{
		timerb_count -= tb_clks;
		while ( timerb_count <= 0 )
		{
			long long tb_val;
			tb_val = pit[1].latch ? pit[1].latch : 0x10000;
			tb_val *= pit[2].latch ? pit[2].latch : 0x10000;
			
			timerb_count += tb_val;
			if ( pc_tcr & 0x08 ) 
				pc_timers_irq |= 2;
		}
	}
	
	update_irq();
}

class PIUbridge
{
public:
	PIUbridge()
	{
		data = 0;
		data_ready = false;
		rxinten = txinten = false;
	}
	
	inline void put( unsigned char val )
	{
		data = val;
		data_ready = true;
		update_irq();
	}

	inline unsigned char get()
	{
		unsigned char val;
		/* mutex? */
		val = data;
		data_ready = false;
		update_irq();
		
		return val;
	}
	
	inline unsigned char peek() { return data; }

	inline void setRxInterrupt( bool val ) { rxinten = val; update_irq(); }
	inline void setTxInterrupt( bool val ) { txinten = val; update_irq(); }
	inline bool isRxIntEnabled() { return rxinten; }
	inline bool isTxIntEnabled() { return txinten; }

	inline bool isReady() { return data_ready; }
	inline bool isRxInterrupt() { return data_ready && rxinten; }
	inline bool isTxInterrupt() { return !data_ready && txinten; }

private:
	unsigned char data;
	bool data_ready;
	bool rxinten;
	bool txinten;
};

static PIUbridge pctofm, fmtopc;
static unsigned char extra_bits;

static Z80EX_BYTE fmcpu_mread( Z80EX_CONTEXT *cpu, Z80EX_WORD addr, int m1_state, void *user_data )
{
	if ( addr < 0x8000 )
		return rom[addr];
	return ram[ addr & 0x3FFF ];
}

static void fmcpu_mwrite( Z80EX_CONTEXT *cpu, Z80EX_WORD addr, Z80EX_BYTE value, void *user_data )
{
	if ( addr < 0x8000 )
		return;
	ram[ addr & 0x3FFF ] = value;
}

static Z80EX_BYTE fmcpu_pread( Z80EX_CONTEXT *cpu, Z80EX_WORD port, void *user_data )
{
	Z80EX_BYTE retval = 0xFF;
	switch ( (port >> 4) & 3 )
	{
		case 0:
			if ( port & 1 )
				retval = fmchip.ReadStatus();
			break;
		case 1:
			retval = uart_read( port & 1 );
			break;
		case 2:
			/* port A output, port B input */
			switch ( port & 0xF )
			{
				case 0:
					retval = fmtopc.peek();
					break;
				case 1:
					retval = pctofm.get();//fprintf(stderr,"%s %u: 0x%lx -> %x%.02lx\n",__FUNCTION__,__LINE__,port & 0xFF,pc_tcr&0x10?1:0,retval);
					break;
				case 2:
					retval = extra_bits ? 0x20 : 0;
					if ( pctofm.isReady() )
						retval |= 0x02;
					if ( pctofm.isRxInterrupt() )
						retval |= 0x01;
					if ( !fmtopc.isReady() )
						retval |= 0x80;
					if ( fmtopc.isTxInterrupt() )
						retval |= 0x08;
					if ( pctofm.isRxIntEnabled() )
						retval |= 0x04;
					if ( fmtopc.isTxIntEnabled() )
						retval |= 0x40;//fprintf(stderr,"%s %u: 0x%lx -> %.2lx\n",__FUNCTION__,__LINE__,port & 0xFF,retval);
					break;
				default:
					fprintf(stderr,"%s %u: invalid PIU read: %.2lx\n",__FUNCTION__,__LINE__,port & 0xF);
					break;
			}
			break;
	}
//fprintf(stderr,"%s %u: 0x%lx -> %.2lx\n",__FUNCTION__,__LINE__,port & 0xFF,retval);
	return retval;
}

static unsigned char fmchip_reg;
static void fmcpu_pwrite( Z80EX_CONTEXT *cpu, Z80EX_WORD port, Z80EX_BYTE value, void *user_data )
{//fprintf(stderr,"%s %u: 0x%lx <- %.2lx\n",__FUNCTION__,__LINE__,port & 0xFF,value);
	switch ( (port >> 4) & 3 )
	{
		case 0:
			if ( port & 1 )
			{
				opp_process();
				fmchip.SetReg( fmchip_reg, value );
			}
			else
				fmchip_reg = value;
			break;
		case 1:
			uart_write( port & 1, value );
			break;
		case 2:
			/* port A output, port B input */
			switch ( port & 0xF )
			{
				case 0://fprintf(stderr,"%s %u: 0x%lx <- %x%.02lx\n",__FUNCTION__,__LINE__,port & 0xFF,extra_bits?1:0,value);
					fmtopc.put( value );
					break;
				/*case 1:
					
					break;*/
				/*case 2:
					
					break;*/
				case 3:
					if ( value & 0x80 )
					{
						if ( (value & 0xFE) != 0xA6 )
							fprintf(stderr,"%s %u: invalid PIU mode: %.2lx\n",__FUNCTION__,__LINE__,value);
					}
					else
					{
						switch ( (value >> 1) & 7 )
						{
							case 6: /* INTEA - interrupt on buf free */
								fmtopc.setTxInterrupt( value & 1 );
								break;
							case 2:	/* INTEB - interrupt on receive */
								pctofm.setRxInterrupt( value & 1 );
								break;
							case 5:
								extra_bits = value & 1;
								break;
							default:
								fprintf(stderr,"%s %u: invalid PIU set bit: %.2lx\n",__FUNCTION__,__LINE__,value);
								break;
						}
					}
					break;
				default:
					fprintf(stderr,"%s %u: invalid PIU write: %.2lx < %.2lx\n",__FUNCTION__,__LINE__,port,value);
					break;
			}
			break;
	}
}

static Z80EX_BYTE fmcpu_intread( Z80EX_CONTEXT *cpu, void *user_data )
{
	return 0xFF;
}

static unsigned tst_per_sample;
static void fm_process( Bitu len )
{
	int tstates = tst_per_sample * len;
	unsigned sampletst = 0;
	static unsigned err;

	while ( tstates > 0 )
	{
		int t = 0;
		unsigned us;
		
		if ( fmcpu_int )
			t = z80ex_int( fmcpu );
		if ( t == 0 )
			t = z80ex_step( fmcpu );
		fmcpu_int = 0;

		tstates -= t;
		sampletst += t;
		if ( sampletst >= tst_per_sample )
		{
			buf_pos += sampletst / tst_per_sample;
			sampletst %= tst_per_sample;
		}
		
		t += err;
		us = 1000000uL * t / CPU_FREQ;
		err = t - us * CPU_FREQ / 1000000uL;
		fmcpu_int |= fmchip.Count( us );
		fmcpu_int |= pctofm.isRxInterrupt();
		fmcpu_int |= fmtopc.isTxInterrupt();
		fmcpu_int |= uart_int();
		//fprintf(stderr,"%s %x %u %u\n",__FUNCTION__,fmcpu_int,pctofm.isRxInterrupt(),fmtopc.isTxInterrupt());
	}
}

static void IBMMFC_CallBack( Bitu len )
{//fprintf(stderr,"%s %u: %lu\n",__FUNCTION__,__LINE__,len);
	//chan->AddSilence();
	buf_lastpos = buf_pos = 0;
	memset( MixTemp, 0, sizeof(MixTemp) );
	fm_process( len );

	buf_pos = len;
	opp_process();
	if ( chan )
		chan->AddSamples_s16( len, (Bit16s*)MixTemp );

	process_timers( len );
}

static void write_imfc( Bitu port, Bitu val, Bitu /* iolen */ )
{//fprintf(stderr,"%s %u: 0x%lx <- %.2lx\n",__FUNCTION__,__LINE__,port,val);
	switch ( port & 0xF )
	{
		/*case 0:
			break;*/
		case 1://fprintf(stderr,"%s %u: %u %u, %u %u\n",__FUNCTION__,__LINE__,pctofm.isReady(),pctofm.isRxIntEnabled(),fmtopc.isReady(),fmtopc.isTxIntEnabled());
			pctofm.put( val );//fprintf(stderr,"%s %u: 0x%lx <- %x%.02lx\n",__FUNCTION__,__LINE__,port,pc_tcr&0x10?1:0,val);
			break;
		/*case 2:
			
			break;*/
		case 3:
			if ( val & 0x80 )
			{
				if ( (val & 0xFE) != 0xBC )
					fprintf(stderr,"%s %u: invalid PIU mode: %.2lx\n",__FUNCTION__,__LINE__,val);
			}
			else
			{
				switch ( (val >> 1) & 7 )
				{
					case 4: /* INTEA - interrupt on receive */
						fmtopc.setRxInterrupt( val & 1 );
						break;
					case 2: /* INTEB - interrupt on transmit ready */
						pctofm.setTxInterrupt( val & 1 );
						break;
				}
			}
			break;
		case 4 ... 7:
			pit_write( port & 3, val );
			break;
		case 8 ... 0xB://fprintf(stderr,"%s %u: TCR <- %.2lx\n",__FUNCTION__,__LINE__,val);
			pc_tcr = val;
			pc_timers_irq &= val & 3;
			update_irq();
			break;
		default:
			fprintf(stderr,"%s %u: invalid IBM MFC write: %.2lx < %.2lx\n",__FUNCTION__,__LINE__,port,val);
			break;
	}
}

/* port A input, port B output */
static Bitu read_imfc( Bitu port, Bitu /* iolen */ )
{
	Bit8u retval = 0xff;

	switch ( port & 0xF )
	{
		case 0:
			retval = fmtopc.get();//fprintf(stderr,"%s %u: 0x%lx -> %x%.02x\n",__FUNCTION__,__LINE__,port,extra_bits?1:0,retval);
			break;
		case 1:
			retval = pctofm.peek();
			break;
		case 2:
			retval = extra_bits ? 0x80 : 0;
			if ( !pctofm.isReady() )
				retval |= 0x02;
			if ( pctofm.isTxInterrupt() )
				retval |= 0x01;
			if ( fmtopc.isReady() )
				retval |= 0x20;
			if ( fmtopc.isRxInterrupt() )
				retval |= 0x08;
			if ( pctofm.isTxIntEnabled() )
				retval |= 0x04;
			if ( fmtopc.isRxIntEnabled() )
				retval |= 0x10;//fprintf(stderr,"%s %u: 0x%lx -> %x\n",__FUNCTION__,__LINE__,port,retval);
			break;
		case 0xC ... 0xF:
			retval = pc_timers_irq;
			if ( pc_timers_irq || pctofm.isTxInterrupt() || fmtopc.isRxInterrupt() )
				retval |= 0x80;//fprintf(stderr,"%s %u: TSR -> %x\n",__FUNCTION__,__LINE__,retval);
			retval |= 0x7C;
			break;
		default:
			fprintf(stderr,"%s %u: invalid IBM MFC read: %.2lx\n",__FUNCTION__,__LINE__,port & 0xF);
			break;
	}
//fprintf(stderr,"%s %u: 0x%lx -> %x\n",__FUNCTION__,__LINE__,port,retval);
	return retval;
}

#define MOUSE_DELAY 5.0

static void MOUSE_Limit_Events(Bitu /*val*/)
{
	fprintf(stderr,"%s %u\n",__FUNCTION__,__LINE__);
	/*mouse.timer_in_progress = false;
	if ( mouse.events )
	{
		mouse.timer_in_progress = true;
		PIC_AddEvent(MOUSE_Limit_Events,MOUSE_DELAY);
		PIC_ActivateIRQ(MOUSE_IRQ);
	}*/
	PIC_ActivateIRQ( imfcIrq );
}

static void update_irq()
{
	static Bit8u cur_lvl = 0;
	Bit8u active = pc_timers_irq || pctofm.isTxInterrupt() || fmtopc.isRxInterrupt();
	
	if ( (pc_tcr & 0xC0) != 0xC0 )
		active = 0;
	
	if ( cur_lvl != active )
	{//fprintf(stderr,"%s %u: %u -> %u\n",__FUNCTION__,__LINE__,cur_lvl,active);
		if ( active )
			PIC_ActivateIRQ( imfcIrq );
		else
			PIC_DeActivateIRQ( imfcIrq );
	}
	
	cur_lvl = active;
}

class IBMMFC:public Module_base
{
private:
	IO_WriteHandleObject WriteHandler;
	IO_ReadHandleObject ReadHandler;
	MixerObject MixerChan;

public:
	IBMMFC( Section* configuration ) : Module_base(configuration)
	{
		Section_prop *section = static_cast<Section_prop *>(configuration);
		imfcBase = section->Get_hex( "imfcbase" );
		imfcIrq = section->Get_int( "imfcirq" );
		sampleRate = section->Get_int( "imfcrate" );
		Prop_path *rompath = section->Get_path( "imfcrom" );

		if ( sampleRate < 8000 )
			sampleRate = 8000;
		tst_per_sample = CPU_FREQ / sampleRate;
		tclks_per_sample = TMRCLK_B / sampleRate;

		FILE *fp = fopen( rompath->realpath.c_str(), "rb" );
		if ( fp )
		{
			fread( rom, 1, sizeof(rom), fp );
			fclose( fp );
		}

		chan = MixerChan.Install( &IBMMFC_CallBack, sampleRate, "IBMMFC" );
		chan->SetScale( 2.0f );
		
		WriteHandler.Install( imfcBase, write_imfc, IO_MB, 16 );
		ReadHandler.Install( imfcBase, read_imfc, IO_MB, 16 );

		fmchip.Init( FMCLK, sampleRate );
		fmcpu = z80ex_create( fmcpu_mread, this, fmcpu_mwrite, this,
			fmcpu_pread, this, fmcpu_pwrite, this, fmcpu_intread, this );

		z80ex_reset( fmcpu );
		fmchip.Reset();
		fmcpu_int = 0;
		pc_timers_irq = 0;

		chan->Enable( true );
	}
	
	~IBMMFC()
	{
		if ( fmcpu )
			z80ex_destroy( fmcpu );
	}
};

static IBMMFC* test;
   
void IBMMFC_Init( Section* sec )
{ 
	test = new IBMMFC( sec );
}
void IBMMFC_ShutDown( Section* sec )
{
	delete test;
}
