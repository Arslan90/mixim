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

import org.mixim.base.connectionManager.ConnectionManager;
import org.mixim.base.modules.BaseWorldUtility;
import org.mixim.modules.mobility.traci.TraCIScenarioManagerLaunchd;
import org.mixim.modules.obstacle.ObstacleControl;
import org.mixim.modules.world.annotations.AnnotationManager;

module Highway
{
    parameters:
        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
        @display("bgb=500,300;bgg=100,,#F00303");
        
        int numeroNodes; // total number of hosts in the network ROAD SIDE UNITS
        
    submodules:
        obstacles: ObstacleControl {
            @display("p=468,27");
        }
        annotations: AnnotationManager {
            @display("p=373,27");
        }
        connectionManager: ConnectionManager {
            parameters:
                @display("p=135,24;i=abstract/multicast");
        }
        world: BaseWorldUtility {
            parameters:
                playgroundSizeX = playgroundSizeX;
                playgroundSizeY = playgroundSizeY;
                playgroundSizeZ = playgroundSizeZ;
                @display("p=313,24;i=misc/globe");
        }
        manager: TraCIScenarioManagerLaunchd {
            parameters:
                @display("p=244,27");
        }

        //ARTURO Voici j'ajoute le nom VPA[nombre des unités]: le file.ned where to get the VPA module
        //VPA[numeroNodes]: RoadSideUnit {
        VPA[numeroNodes]:RoadSideUnit{
            parameters:
                @display("i=device/antennatower;p=300,111");
        }

    connections allowunconnected:
}

