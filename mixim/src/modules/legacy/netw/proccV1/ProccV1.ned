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

package org.mixim.modules.legacy.netw.proccV1;

import org.mixim.modules.legacy.netw.opprouting.LEG_DtnNetwLayer;
//
// TODO auto-generated type
//
simple ProccV1 extends LEG_DtnNetwLayer
{	
    parameters:
    	@class(ProccV1);
    	
    	double Alpha=default(0.9);
    	double GAMMA=default(0.999);
    	double Delta=default(0.1);
    	int secondsInTimeUnit =default(30);
    	double PMinThreshold=default(0.0);
    	int fwdStrategy =default(7);
    	int qStrategy = default(1);
    	int historySize = default(10);
    	
    	headerLength @unit(bit) = default(272bit);
    	
        bool withFailRIBClassifier = default(false);
        bool withFailBndlOfferClassifier = default(false);
        bool withFailBndlRespClassifier = default(false);
        bool withFailBndlClassifier = default(false);
    	bool withFailBndlAckClassifier = default(false);
    	
        bool CDFForFailRIBClassifier = default(false);
        bool CDFForFailBndlOfferClassifier = default(false);
        bool CDFForFailBndlRespClassifier = default(false);
        bool CDFForFailBndlClassifier = default(false);
        bool CDFForFailBndlAckClassifier = default(false);
    	

}
