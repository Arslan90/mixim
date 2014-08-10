//****************************************************************************
// * My file:        TestingBurstApplLayer.ned
// * OK this is to add the module as the main apps for my testing platform. 
// ***************************************************************************/
package org.mixim.modules.application;

import org.mixim.base.modules.IBaseApplLayer;

// Application layer to test lower layer implementations
//
// This application layer does exactly the same as the
// TestApplLayer. The only difference is that is sends a burst of
// broadcast messages instead of just one. The burst size is specified
// in burstSize and can be set in omnetpp.ini
//
// @see TestApplLayer
// @author Daniel Willkomm
simple TestingBurstApplLayer like IBaseApplLayer
{
    parameters:
        double timeT @unit("s"); //ARTURO, trying to pass values from NED files.
        bool coreDebug = default(false); //debug switch for base class (TestApplLayer)
        bool debug = default(false); // debug switch
        int headerLength @unit("bit");   // length of the application message header (in bits)
        bool appCW = default(false); //use or not APPS_CW
		double appMinCW = default(0.01); //Range of APPS_CW 
        //int burstSize;                   // size of the burst
        //bool burstReply = default(true);  // if true, send unicast replies to received broadcasts
    gates:
        input lowerLayerIn; // from network layer
        output lowerLayerOut; // to network layer
        input lowerControlIn; // control from network layer
        output lowerControlOut; // control to network layer
}
