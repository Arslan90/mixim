/*
 * multiFunctions.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: arslan
 */

#include "omnetpp.h"
#include "math.h"
#include "multiFunctions.h"

unsigned long multiFunctions::cantorPairingFunc(const unsigned long x, const unsigned long y){
		unsigned long result = 0;
		if (!((x>=0) && (y>=0))){
			opp_error("entries for cantorParing function must be naturals numbers (namespace myFunctions)");
		}else{
			result = (((x+y)*(x+y+1))/2 + y);
		}
		return result;
}

std::pair<unsigned long ,unsigned long > multiFunctions::inverseCantorPairingFunc(const unsigned long  z)
{
	unsigned long x = 0 , y = 0, t = 0, w = 0;
	if (!(z>=0)){
		opp_error("entry for Inverse of CantorParing function must be natural numbers (namespace myFunctions)");
	}else{
		w = floor((sqrt(8*z+1)-1)/2);
		t = (pow(w,2)+w)/2;
		y = z-t;
		x = w-y;
	}
	return std::pair<unsigned long ,unsigned long > (x,y);
}



