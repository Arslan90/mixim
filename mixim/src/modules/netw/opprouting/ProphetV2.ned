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
    	
    	double PEncMax=default(0.7);
    	double PFirstContact=default(0.5);
    	double PMinThreshold=default(0.0);
//    	double I_TYP @unit("s")=default(1800s);
		double I_TYP =default(1800);
    	double Beta=default(0.9);
//    	double GAMMA=default(0.999885791);
    	double GAMMA=default(0.999);
//    	int secondsInTimeUnit @unit("s")=default(30s);
    	int secondsInTimeUnit =default(30);
    	int fwdStrategy =default(7);
    	int qStrategy = default(1);
    	int storageSize = default(1000);
    	
    	bool withAck = default(false);
    	int ackSize = default(1000);
    	
    	bool withTTL = default(false);
    	int ttl = default(900);
    	
    	bool recordContactStats = default(true);
    	
    	bool withGlobalClassifier = default(true);
        bool withSuccClassifier = default(true);
        bool withFailClassifier = default(true);
        bool withFailRIBClassifier = default(false);
        bool withFailBndlOfferClassifier = default(false);
        bool withFailBndlRespClassifier = default(false);
        bool withFailBndlClassifier = default(false);
    	bool withFailBndlAckClassifier = default(false);
    	
    	bool CDFForGlobalClassifier = default(false);
        bool CDFForSuccClassifier = default(false);
        bool CDFForFailClassifier = default(false);        bool CDFForFailRIBClassifier = default(false);        bool CDFForFailBndlOfferClassifier = default(false);        bool CDFForFailBndlRespClassifier = default(false);        bool CDFForFailBndlClassifier = default(false);        bool CDFForFailBndlAckClassifier = default(false);
}       