//***************************************************************************
// * My file:        Network.ned
// * I'm using this in order to test separately the Mac80211 broadcast scenario
// **************************************************************************/
package org.mixim.examples.Mac80211;

import org.mixim.base.modules.BaseNetwork;
import org.mixim.modules.node.Host80211;
//
//This example shows how to use MiXiMs 802.11 protocol implementation. 
//
//The network consists of ten hosts broadcasting bursts of three packets
//to the other hosts. When a broadcast is received a reply is sent back 
//to the sender.
//
//The hosts use "Mac80211" as MAC layer and their phy layer uses 
//"SimplePathlossModel" as AnalogueModel and "Decider80211" as Decider.
//Note that Mac80211 and Decider80211 are always meant to be used
//together.
//
//The default configuration in omnetpp.ini uses no propagation delay.
//
network RoadSideUnit extends BaseNetwork
{
    parameters:
        int numHosts; // total number of hosts in the network  

        @display("bgg=100,,#E71010");
    submodules:
        host[numHosts]: Host80211 { //Arturo, here I'm calling the ned file: Host80211
            @display("p=137,95;i=device/antennatower");
        }

    connections allowunconnected:
    // all connections and gates are to be generated dynamically
}
