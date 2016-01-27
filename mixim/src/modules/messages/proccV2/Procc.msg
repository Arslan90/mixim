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
#include "BundleMeta.h"
typedef std::map<LAddress::L3Type, std::vector<double> > histOfPredictions;
typedef std::map<LAddress::L3Type, double> predictions;
typedef std::list<BundleMeta> bndlMeta;
}}
packet NetwPkt;

enum t_result_field;
class noncobject histOfPredictions;
class noncobject predictions;
class noncobject BundleMeta;
class noncobject bndlMeta;
//
// TODO generated message class
//
packet Procc extends NetwPkt {
    int nodeClass;
    int result_Field @enum(t_result_field);
    int identifier;
    unsigned long contactID;
    bool fragmentFlag = false;
    short fragmentNum = 0;
    short totalFragment = 0;
    int tlv_Flags;
//    histOfPredictions historyOfPredictions = std::map<LAddress::L3Type, std::vector<double> >();
	histOfPredictions historyOfPredictions;
	predictions preds;
    bndlMeta bndlmeta = std::list<BundleMeta>();
    int hopCount = 0;
}
