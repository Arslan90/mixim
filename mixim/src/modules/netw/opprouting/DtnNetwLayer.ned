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
// TODO auto-generated module
//
simple DtnNetwLayer extends BaseNetwLayer
{	
    parameters:
    	@class(DtnNetwLayer);
    	
    	int storageSize = default(1000);
    	
    	bool withAck = default(false);
    	int ackSize = default(1000);
    	
    	bool withTTL = default(false);
    	int ttl = default(900);
    	
    	bool withTTLForCtrl = default(false);
    	//int ttlForCtrl = default(900);
    	
    	double factorForTTLCtrl = default(1.0);
    	
    	bool DF = default(false);
    	int MTU = default(2304); // MTU for WIFI 802.11P in bytes (octets)
    	headerLength @unit(bit) = default(272bit);
    	
	    double heartBeatMsgPeriod = default(0.5);
    	double netwRouteExpirency = default(2.0);
    	double netwRoutePending = default(1.0);
    	
    	int scheduleStrategy = default(0);
    	
    	double I_Preds = default(5);
    	
    	bool recordContactStats = default(true);
    	
    	bool withGlobalClassifier = default(true);
        bool withSuccClassifier = default(true);
        bool withFailClassifier = default(true);
    	
    	bool CDFForGlobalClassifier = default(false);
        bool CDFForSuccClassifier = default(false);
        bool CDFForFailClassifier = default(false);
        
        // equiped vehicle pourcentage, by default 1 (100%)
        double equipedVehPc = default(1);
        
        double updateInterval = default(10.0);
        
//        string cutPoints = default("1;5;20;50;100;500;");
//        string cutPoints = default("20;50;100");

//		double delayed = default(0.0);
//		
//		double delayedFrag = default(0.0);
		
//		bool withRestart = default(false);
		
//		bool withConnectionRestart = default(false);
//		
//		bool withContactTraceFile = default(false);
//		string contactTraceFileName = default("netContact.trace");
//		double contactTrFlUpdatePeriod = default(5.0);
}       
