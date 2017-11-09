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
#include "DtnNetwPkt_m.h"
#include "SimpleAddress.h"
#include "BundleMeta.h"
typedef std::list<BundleMeta> bndlMeta;
typedef std::set<LAddress::L3Type> Neighbors;
typedef std::set<unsigned long> CustodySerials;
}}
packet DtnNetwPkt;

enum t_result_field;
class noncobject BundleMeta;
class noncobject bndlMeta;
class LAddress::L3Type extends void;
class noncobject Neighbors;
class noncobject CustodySerials;
//
// TODO generated message class
//
packet GeoDtnNetwPkt extends DtnNetwPkt {
    LAddress::L3Type fwdMETD = 0; // LAddress::L3NULL = 0;
    LAddress::L3Type fwdDist = 0;
    LAddress::L3Type vpaAddr = 0;
    double vpaPosX;
    double vpaPosY;
    double vpaPosZ;
    double srcMETD = 1.79769e+308;
    double srcDist_NP_VPA = 1.79769e+308;
    bool custodyTransfert = false;
    bndlMeta bndlmeta = std::list<BundleMeta>();
    Neighbors knownNeighbors;
    CustodySerials custodySerials;
    int nbrReplica;
    
    int result_Field @enum(t_result_field);
    int identifier;
    int tlv_Flags;
    bool restartIEP = false;
}
