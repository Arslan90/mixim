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
#include "SimpleAddress.h"
#include "BundleMeta.h"
typedef std::list<BundleMeta> bndlMeta;
typedef std::set<LAddress::L3Type> Neighbors;
typedef std::set<unsigned long> E2EAcks;
typedef std::set<unsigned long> H2PAcks;
}}
packet NetwPkt;

enum t_result_field;
class noncobject BundleMeta;
class noncobject bndlMeta;
class LAddress::L3Type extends void;
class noncobject Neighbors;
class noncobject E2EAcks;
class noncobject H2PAcks;
//
// TODO generated message class
//
packet GeoDtnNetwPkt extends NetwPkt {
    LAddress::L3Type fwdMETD;
    LAddress::L3Type fwdDist;
    LAddress::L3Type vpaAddr;
    int vpaSectorId;
    int srcType;
    double vpaPosX;
    double vpaPosY;
    double vpaPosZ;
    double srcMETD;
    double srcDist_NP_VPA;
    bool custodyTransfert = false;
    bool df_Flag = false;
    short fragNbr = 0;
    short fragTotal = 0;
    unsigned long contactID;
    bndlMeta bndlmeta = std::list<BundleMeta>();
    Neighbors knownNeighbors;
    E2EAcks e2eAcks;
    H2PAcks h2hAcks;
    
    int result_Field @enum(t_result_field);
    int identifier;
    int tlv_Flags;
    int hopCount = 0;
    bool restartIEP = false;
}
