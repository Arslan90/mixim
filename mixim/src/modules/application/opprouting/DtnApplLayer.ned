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

package org.mixim.modules.application.opprouting;

import org.mixim.base.modules.BaseNetwLayer;
//
// TODO auto-generated module
//
simple DtnApplLayer
{
    	bool withDtnMsg = default(true); // wheter we are testing the sent of DTN Msg
		
		string scenarioType = default("Free");
		// Parameters related to Sector scenarioType only
		int rowSectorGrid = default(-1);
		int colSectorGrid = default(-1);   
		double sectorSizeX = default(-1);  
		double sectorSizeY = default(-1);
		bool useNegativeValues = default(false);  
		double sectorOffsetX = default(-1);
		double sectorOffsetY = default(-1); 
		
		
		string dataGeneration = default("VEH2VPA");
		string dataForwarding = default("ALL");
		
		string dtnMsgSending = default("Periodic");
		string dtnMsgReception = default("Unique");

		double dtnMsgPeriod = default(-1); // period for sending dtnMessage
		double dtnMsgMaxTime = default(-1);     
		double dtnMsgMinTime = default(-1);     
		bool dtnMsgSynchronized = default(true);  
		
		
		bool sectorMode = default(false);
		bool updateMode = default(false);
}
