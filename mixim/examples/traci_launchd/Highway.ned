//
// Copyright (C) 2008 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

//ARTURO ADDED THIS TO USE THE:  src/modules/node/WirelessNode.ned
//import org.mixim.modules.node.WirelessNode;
//import org.mixim.base.modules.IBaseApplLayer;

package org.mixim.examples.traci_launchd;

import org.mixim.base.connectionManager.ConnectionManager;
import org.mixim.base.modules.BaseWorldUtility;
import org.mixim.modules.mobility.traci.TraCIScenarioManagerLaunchd;

module Highway
{
    parameters:
        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
        @display("bgb=550,300;bgg=100,1,#FA1616");

        int numNodes; // total number of hosts in the network ROAD SIDE UNITS

    submodules:
        connectionManager: ConnectionManager {
            parameters:
                @display("p=355,25;i=abstract/multicast");
        }
        world: BaseWorldUtility {
            parameters:
                playgroundSizeX = playgroundSizeX;
                playgroundSizeY = playgroundSizeY;
                playgroundSizeZ = playgroundSizeZ;
                @display("p=527,25;i=misc/globe");
        }
        manager: TraCIScenarioManagerLaunchd {
            parameters:
                @display("p=460,25");
        }

        //Voici j'ajoute le nom VPA[nombre des unités]: le file.ned where to get the VPA module
        //VPA[numNodes]: RoadSideUnit {
        VPA[numNodes]: RoadSideUnitMac80211 {
            parameters:
                @display("i=device/antennatower;p=301,101");
        }


    connections allowunconnected:
}

