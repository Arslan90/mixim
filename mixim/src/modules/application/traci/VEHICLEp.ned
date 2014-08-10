//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
//

package org.mixim.modules.application.traci;
import org.mixim.modules.application.ieee80211p.BaseWaveApplLayer;

simple VEHICLEp extends BaseWaveApplLayer
{
    parameters: //Arturo, Agrege esto
        @class(VEHICLEp);
        @display("i=block/app2");
        double timeT = default(5); //agregue esto
        double epicenterValue @unit("m") = default(20m); //agregue esto
        bool appCW = default(false);// Wheter using or not my artificial appCW
        int appMinCW = default(100);// adding my own value of ST_MAX     
        
        //Valores de la Map.
        int offsetX=  default(4000); // valor extra para el axis X
		int offsetY= default(5000); // valor extra para el axis Y
		int maxY= default(41947); // valor maximo de la map.
           
}
