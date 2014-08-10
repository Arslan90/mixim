//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
//

package org.mixim.modules.application.opprouting;
import org.mixim.base.modules.IBaseApplLayer;
import org.mixim.modules.application.ieee80211p.BaseWaveApplLayer;

simple VPApOpp extends BaseWaveApplLayer
{
    parameters:
        @class(VPApOpp); //MERDEEEE!!!!  J'ai trouve! Fait attention de lier bien le fichie .ned avec l'application correct.!! 
        double timeT @unit("s") = default(5s);
        double epicenterValue @unit("m") = default(20m);
}
