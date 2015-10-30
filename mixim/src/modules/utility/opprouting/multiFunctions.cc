/*
 * multiFunctions.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: arslan
 */

#include "multiFunctions.h"
#include "simutil.h"

int multiFunctions::cantorPairingFunc(const int x, const int y){
		int result = 0;
		if (!((x>=0) && (y>=0))){
			opp_error("entries for cantorParing function must be naturals numbers (namespace myFunctions)");
		}else{
			result = (((x+y)*(x+y+1))/2 + y);
		}
		return result;
}

