#include "opp.h"

namespace FM
{

OPP::OPP() : OPM()
{

}

void OPP::SetReg(uint addr, uint data)
{
	if (addr < 0x08)
	{
		//LOG1("%s: Writing %02X to undocumented register %d\n", machine().describe_context(), v, r);
	}
	else if (addr == 0x09)
		OPM::SetReg(0x01, data);
	else
		OPM::SetReg(addr, data);
}

}
