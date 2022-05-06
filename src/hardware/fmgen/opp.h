#ifndef FM_OPP_H
#define FM_OPP_H

#include "opm.h"

namespace FM
{

class OPP : public OPM
{
public:
	OPP();
	void 	SetReg(uint addr, uint data);
};

}

#endif // FM_OPP_H
