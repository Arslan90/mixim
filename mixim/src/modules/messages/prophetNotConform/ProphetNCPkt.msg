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
#include <map>
#include "SimpleAddress.h"
#include "Prophet_Enum.h"
#include "BundleMeta.h"
typedef std::map<LAddress::L3Type, double> predictions;
typedef std::list<BundleMeta> bndlMeta;
}}
packet DtnNetwPkt;

enum t_result_field;
class noncobject predictions;
class noncobject BundleMeta;
class noncobject bndlMeta;
//
// TODO generated message class
//
packet ProphetNCPkt extends DtnNetwPkt {
    int result_Field @enum(t_result_field);
    int identifier;
    int tlv_Flags;
    predictions preds;
    bndlMeta bndlmeta = std::list<BundleMeta>();
    bool restartIEP = false;
}
