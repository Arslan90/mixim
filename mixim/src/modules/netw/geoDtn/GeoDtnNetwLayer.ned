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

package org.mixim.modules.netw.geoDtn;

import org.mixim.modules.netw.opprouting.DtnNetwLayer;

//
// TODO auto-generated module
//
simple GeoDtnNetwLayer extends DtnNetwLayer
{
    parameters:
    	@class(GeoDtnNetwLayer);
    	double heartBeatMsgPeriod = default(0.5);
    	double netwRouteExpirency = default(2.0);
    	double netwRoutePending = default(1.0); 
}
