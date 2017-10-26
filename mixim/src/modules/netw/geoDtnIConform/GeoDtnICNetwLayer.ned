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

package org.mixim.modules.netw.geoDtnIConform;

import org.mixim.modules.netw.opprouting.DtnNetwLayer;

//
// TODO auto-generated module
//
simple GeoDtnICNetwLayer extends DtnNetwLayer
{
    parameters:
    	@class(GeoDtnICNetwLayer);
    	bool withMETDFwd = default(true);
    	bool withDistFwd = default(true); 
    	
    	// 0: No Custody, 
    	// 1: Custody for Dist with No Active Clearance of theses bundles, 
    	// 2: Custody for Dist with Active Clearence of these bundles
    	int custodyMode = default(0);
    	// CBH: CheckBeforeHello
    	bool withCBH = default(false);
}
