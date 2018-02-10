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

cplusplus {{
#include "NetwPkt_m.h"
#include "Coord.h"
typedef std::set<unsigned long> E2EAcks;
typedef std::set<unsigned long> H2PAcks;
typedef std::map<unsigned long, double> AckSerialsWithExpireTime;
}}
packet NetwPkt;

class LAddress::L3Type extends void;
class noncobject E2EAcks;
class noncobject H2PAcks;
class noncobject Coord;
class noncobject AckSerialsWithExpireTime;
//
// TODO generated message class
//
packet DtnNetwPkt extends NetwPkt {
    int vpaSectorId;
    int srcType;
    bool df_Flag = false;
    short fragNbr = 0;
    short fragTotal = 0;
    unsigned long contactID;
    E2EAcks e2eAcks;
    H2PAcks h2hAcks;
    AckSerialsWithExpireTime ackSerialsWithTimestamp;
    Coord currentPos;
    int hopCount = 0;
}
