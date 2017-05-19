//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
//

package org.mixim.modules.application.opprouting;
import org.mixim.modules.application.opprouting.DtnApplLayer;
//import org.mixim.modules.application.ieee80211p.BaseWaveApplLayer;

simple VEHICLEpOpp extends DtnApplLayer
{
    parameters: //Arturo, Agrege esto
        @class(VEHICLEpOpp);
        @display("i=block/app2");
        double timeT = default(5); //agregue esto
        double epicenterValue @unit("m") = default(20m); //agregue esto
        bool appCW = default(false);// Wheter using or not my artificial appCW
        int appMinCW = default(100);// adding my own value of ST_MAX     
        
        //Valores de la Map.
//      int offsetX=  default(4000); // valor extra para el axis X
//		int offsetY= default(5000); // valor extra para el axis Y
//		int maxY= default(41947); // valor maximo de la map.
		
//		bool dtnTestMode = default(false); // wheter we are testing DTN
//		int dtnTestCycle = default(5); // cycle of periodic sending dtnMessage
//		int dtnTestMaxTime; // max time for sending dtnMessage
//		bool dtnSynchronized = default(false);
//		
//		bool sectorMode = default(false);
//		bool updateMode = default(false);
		
		// my functions
		bool vehicleIdAsInt = default(false);
		
		bool reRouteAtEnd = default(false);
		int maxNbrOfReRouting = default(0);
		int nbrVehForReRouting = default(5);
		
		withDtnMsg = default(true);
        withOtherMsg = default(false);
}
