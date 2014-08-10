//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.mixim.modules.netw.opprouting;

import org.mixim.base.modules.BaseNetwLayer;
//
// TODO auto-generated type
//
simple ProphetV2 extends BaseNetwLayer
{	
    parameters:
    	@class(ProphetV2);
    	
    	double PEncMax=default(0.5);
//    	double I_TYP @unit("s")=default(1800s);
		double I_TYP =default(1800);
    	double Beta=default(0.9);
    	double GAMMA=default(0.999885791);
//    	int secondsInTimeUnit @unit("s")=default(30s);
    	int secondsInTimeUnit =default(30);
    	int fwdStrategy =default(7);
    	int qStrategy = default(1);
}
