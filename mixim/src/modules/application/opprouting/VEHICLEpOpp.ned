//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
//

package org.mixim.modules.application.opprouting;
import org.mixim.modules.application.ieee80211p.BaseWaveApplLayer;

simple VEHICLEpOpp extends BaseWaveApplLayer
{
    parameters: //Arturo, Agrege esto
        @class(VEHICLEpOpp);
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
