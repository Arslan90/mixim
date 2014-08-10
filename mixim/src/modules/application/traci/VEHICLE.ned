// My file VEHICLE.ned based on the file TraCIDemo.ned


package org.mixim.modules.application.traci;

import org.mixim.base.modules.IBaseApplLayer;

//
// TraCI IVC Demo
//
simple VEHICLE like IBaseApplLayer
{
    parameters:
        @display("i=block/app2");
        //double timeT @unit("s") = default(5s); //ARTURO, trying to pass values from NED files.
        double timeT = default(5); //ARTURO, trying to pass values from NED files.
        double epicenterValue = default(20); //ARTURO, trying to pass values from NED files.
        bool debug = default(false);  // output debugging information
        int headerLength @unit("bit")= default(0bit);  // length of the application message header (in bits)
        bool appCW = default(false);// Wheter using or not my artificial appCW
        int appMinCW = default(100);// adding my own value of ST_MAX   
        
        //Valores de la Map.
        int offsetX=  default(4000); // valor extra para el axis X
		int offsetY= default(5000); // valor extra para el axis Y
		int maxY= default(41947); // valor maximo de la map.
           

    gates:
        input lowerLayerIn; // from network layer
        output lowerLayerOut; // to network layer
        input lowerControlIn; // control from network layer
        output lowerControlOut; // control to network layer
}
