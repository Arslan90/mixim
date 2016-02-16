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
#include <bitset>
#include <map>
#include <vector>
#include "SimpleAddress.h"
#include "Prophet_Enum.h"
// #include "Prophet_Struct.h"
#include "BundleMeta.h"
//typedef Prophet_Struct::preds preds;
//typedef std::vector<double> history;
typedef std::map<LAddress::L3Type, std::vector<double> > histOfPredictions;
//typedef std::list<Prophet_Struct::bndl_meta> bndlMeta;
typedef std::list<BundleMeta> bndlMeta;
}}
packet NetwPkt;

enum t_result_field;
//class noncobject history;
class noncobject histOfPredictions;
//class noncobject preds;
//struct bndl_meta;
class noncobject BundleMeta;
class noncobject bndlMeta;
//
// TODO generated message class
//
packet ccProphet extends NetwPkt {
    int nodeClass;
    int result_Field @enum(t_result_field);
    int identifier;
    int tlv_Flags;
    histOfPredictions historyOfPredictions;
//    preds predictions = std::map<LAddress::L3Type, double>();
//    bndlMeta bndlmeta = std::list<Prophet_Struct::bndl_meta>();
    bndlMeta bndlmeta = std::list<BundleMeta>();
    int hopCount = 0;
}
