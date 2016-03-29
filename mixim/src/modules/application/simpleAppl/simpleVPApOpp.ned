//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
//

package org.mixim.modules.application.simpleAppl;
import org.mixim.modules.application.opprouting.DtnApplLayer;
//import org.mixim.base.modules.IBaseApplLayer;
//import org.mixim.modules.application.ieee80211p.BaseWaveApplLayer;

simple simpleVPApOpp extends DtnApplLayer
{
    parameters:
        @class(simpleVPApOpp); //MERDEEEE!!!!  J'ai trouve! Fait attention de lier bien le fichie .ned avec l'application correct.!! 
        double timeT @unit("s") = default(5s);
        double epicenterValue @unit("m") = default(20m);
//        bool dtnTestMode = default(false); // wheter we are testing DTN
//        bool silentMode = default(false); // turn VPA in silent mode
        int updateSectorCycle = default(300);
//        bool updateMode = default(false);
        
        withDtnMsg = default(false);
        withOtherMsg = default(false);
        
        double stopPos = default(-1);
        double stopDuration = default(-1);
        double stopDistStep = default(-1);
        double releasePos = default(-1);
        
        bool with127M = default(false);
        double targetPos = default(-1);
        
        int periodForMsg @unit("s")= default(30s);
        int nbrMsgByStep = default(100);
}
