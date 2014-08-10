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
import org.mixim.modules.application.ieee80211p.ArturoTestWaveApplLayer;
import org.mixim.modules.mobility.ConstSpeedMobility;
import org.mixim.modules.mobility.LinearMobility;
import org.mixim.modules.mobility.traci.TraCIMobility;

import org.mixim.modules.application.ieee80211p.TestWaveApplLayer;
import org.mixim.base.modules.BaseArp;

import org.mixim.modules.nic.Nic80211p;


module RoadSideUnit
{
    parameters:
        @display("p=10,10;b=4,4,rect;o=white;i=device/antennatower");

    gates:
        input radioIn; // gate for sendDirect

    submodules:
        arp: BaseArp {
            @display("p=257,28;b=30,25");
        }

        nic: Nic80211p {
            @display("p=90,150;b=100,20,rect");
        }

        arturoTestWaveApplLayer: ArturoTestWaveApplLayer {
            @display("p=90,43");
        }
        Mobility: ConstSpeedMobility {
            @display("p=223,121");
        }
    connections:
        radioIn --> nic.radioIn;
        nic.upperControlOut --> arturoTestWaveApplLayer.lowerControlIn;
        nic.upperLayerOut --> arturoTestWaveApplLayer.lowerLayerIn;
        arturoTestWaveApplLayer.lowerControlOut --> nic.upperControlIn;
        arturoTestWaveApplLayer.lowerLayerOut --> nic.upperLayerIn;
}
