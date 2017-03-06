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

package org.mixim.modules.netw.prophetNotConform;

import org.mixim.modules.netw.opprouting.DtnNetwLayer;
//
// TODO auto-generated type
//
simple ProphetNCV2 extends DtnNetwLayer
{	
    parameters:
    	@class(ProphetNCV2);
    	
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
    	
    	headerLength @unit(bit) = default(272bit);
    	
    	bool withPartialUpdate = default(true);
    	
    	bool withEMethod = default(false);
    	bool withPredLength = default(false);
    	
    	
        bool withFailRIBClassifier = default(false);
        bool withFailBndlOfferClassifier = default(false);
        bool withFailBndlRespClassifier = default(false);
        bool withFailBndlClassifier = default(false);
    	bool withFailBndlAckClassifier = default(false);
    	
        bool CDFForFailRIBClassifier = default(false);        bool CDFForFailBndlOfferClassifier = default(false);        bool CDFForFailBndlRespClassifier = default(false);        bool CDFForFailBndlClassifier = default(false);        bool CDFForFailBndlAckClassifier = default(false);
        
}       