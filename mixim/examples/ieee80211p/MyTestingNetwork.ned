//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
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

package org.mixim.examples.ieee80211p;

import org.mixim.base.modules.BaseWorldUtility;
import org.mixim.base.connectionManager.ConnectionManager;

import org.mixim.modules.mobility.traci.TraCIScenarioManagerLaunchd;

//
// TODO auto-generated module
//
network MyTestingNetwork
{
    parameters:
        int numNodes; // total number of hosts in the network ROAD SIDE UNITS

        @display("bgg=100,,#EA1E1E");
    submodules:
        baseWorldUtility: BaseWorldUtility {
            @display("p=70,20");
        }
        connectionManager: ConnectionManager {
            @display("p=264,20");
        }
        manager: TraCIScenarioManagerLaunchd {
            @display("p=161,20");
        }

        //Voici j'ajoute le nom VPA[nombre des unités]: le file.ned where to get the VPA module
        VPA[numNodes]: RoadSideUnit {
            parameters:
                @display("i=device/antennatower;p=102,100");
        }

    connections allowunconnected:
}
