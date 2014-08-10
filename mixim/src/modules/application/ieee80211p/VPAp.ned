//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
//

package org.mixim.modules.application.ieee80211p;
import org.mixim.base.modules.IBaseApplLayer;

simple VPAp extends BaseWaveApplLayer
{
    parameters:
        @class(VPAp); //MERDEEEE!!!!  J'ai trouve! Fait attention de lier bien le fichie .ned avec l'application correct.!! 
        double timeT @unit("s") = default(5s);
        double epicenterValue @unit("m") = default(20m);
}
