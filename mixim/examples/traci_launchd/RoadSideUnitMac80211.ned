

package org.mixim.examples.traci_launchd;

import org.mixim.base.modules.*;
import org.mixim.modules.mobility.ConstSpeedMobility;
import org.mixim.modules.mobility.LinearMobility;
import org.mixim.modules.nic.Nic80211;


module RoadSideUnitMac80211
{
    parameters:
        string applType; //type of the application layer
        string netwType; //type of the network layer
        string mobilityType; //type of the mobility module
        @display("bgb=225,281");
    gates:
        input radioIn; // gate for sendDirect
    submodules:
        arp: BaseArp {
            parameters:
                @display("p=130,108;b=24,24,rect,blue;i=block/process");
        }
        appl: <applType> like IBaseApplLayer {
            parameters:
                @display("p=60,50;i=app");
        }
        net: <netwType> like IBaseNetwLayer {
            parameters:
                @display("p=60,108;i=block/layer");
        }
        nic80211: Nic80211 {
            @display("p=101,222");
        }
        Mobility: ConstSpeedMobility {
            @display("p=161,39");
        }
    connections:
        net.upperLayerOut --> appl.lowerLayerIn;
        net.upperLayerIn <-- appl.lowerLayerOut;
        net.upperControlOut --> { @display("ls=red;m=m,70,0,70,0"); } --> appl.lowerControlIn;
        net.upperControlIn <-- { @display("ls=red;m=m,70,0,70,0"); } <-- appl.lowerControlOut;

        radioIn --> nic80211.radioIn;

        nic80211.upperLayerOut --> net.lowerLayerIn;
        nic80211.upperControlOut --> net.lowerControlIn;
        net.lowerControlOut --> nic80211.upperControlIn;
        net.lowerLayerOut --> nic80211.upperLayerIn;
}

