//
// Copyright (C) 2008 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

package org.mixim.examples.veins;

import org.mixim.base.modules.*;
import org.mixim.modules.application.ieee80211p.VPAp;
import org.mixim.modules.legacy.netw.proccV2.ProccV2;
import org.mixim.modules.nic.Nic80211pOpp;

module CarProCC
{
    parameters:
        string applType; //type of the application layer
        string mobilityType; //type of the mobility module
    gates:
        input radioIn; // gate for sendDirect
    submodules:
        appl: <applType> like IBaseApplLayer {
            parameters:
                @display("p=60,50");
        }

        netw: ProccV2 {
            parameters:
                @display("p=60,133;i=block/network");
        }

        arp: BaseArp {
            parameters:
                @display("p=130,108");
        }

        nic: Nic80211pOpp {
            parameters:
                @display("p=60,205");
        }

        mobility: <mobilityType> like IMobility {
            parameters:
                @display("p=130,172;i=block/cogwheel");
        }
    connections:
        netw.upperLayerOut --> appl.lowerLayerIn;
        netw.upperLayerIn <-- appl.lowerLayerOut;
        netw.upperControlOut --> appl.lowerControlIn;
        netw.upperControlIn <-- appl.lowerControlOut;
        
        nic.upperLayerOut --> netw.lowerLayerIn;
        nic.upperLayerIn <-- netw.lowerLayerOut;
        nic.upperControlOut --> netw.lowerControlIn;
        nic.upperControlIn <-- netw.lowerControlOut;

        radioIn --> nic.radioIn;

}
