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

package org.mixim.modules.legacy.netw.simpleApplNetw;

import org.mixim.base.modules.BaseNetwLayer;
//
// TODO auto-generated module
//
simple SimpleApplNetw extends BaseNetwLayer
{
    parameters:
        @class(SimpleApplNetw);
        
        bool sentMsg = default(false);
        int periodForMsg @unit("s")= default(30s);
        int nbrMsgByStep = default(100);
        int msgLength @unit(bit) = default(1000bit);
        
        double stopPos = default(-1);
        double stopDuration = default(-1);
        double stopDistStep = default(-1);
        double releasePos = default(-1);
        
        bool with127M = default(false);
        double targetPos = default(-1);
        
        
}
