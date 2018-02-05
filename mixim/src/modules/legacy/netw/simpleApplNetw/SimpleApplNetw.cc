//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "SimpleApplNetw.h"
#include "Mac80211Pkt_m.h"
#include "NetwPkt_m.h"
#include "FindModule.h"
#include "sstream"
#include <math.h>



Define_Module(SimpleApplNetw);

void SimpleApplNetw::initialize(int stage)
{
    BaseNetwLayer::initialize(stage);

    if (stage == 0){

    	sentMsg = par("sentMsg").boolValue();

    	periodForMsg = par("periodForMsg");
    	nbrMsgByStep = par("nbrMsgByStep");
    	msgLength = par("msgLength");

    	remainingMsgByStep = 0;
    	nbrReceived = 0;
        nbrSent = 0;

        msg = new cMessage("Testing");
//        if (sentMsg){
//        	scheduleAt(simTime(),msg);
//        }

        vehStopped = registerSignal("vehStopped");
        simulation.getSystemModule()->subscribe(vehStopped, this);

        isSuscribed = false;

        if (isSubscribed(vehStopped,this)){
        	isSuscribed = true;
        }

		stopPos = par("stopPos").doubleValue();
		stopDuration = par("stopDuration").doubleValue();
		stopDistStep = par("stopDistStep").doubleValue();
		releasePos = par("releasePos").doubleValue();

		targetPos = par("targetPos").doubleValue();

		with127M = par("with127M").boolValue();


    }else if (stage == 1){
    	mobility = FindModule<simpleTraCIMobility*>::findSubModule(this->getParentModule());
    	if (mobility != NULL){
    		mobility->setStopPos(stopPos);
    		mobility->setStopDuration(nbrMsgByStep*periodForMsg);
    		mobility->setStopDistStep(stopDistStep);
    		mobility->setReleasePos(releasePos);
    		mobility->setTargetPos(targetPos);
    		mobility->setWith127M(with127M);
    	}


    	for (double i = targetPos-stopPos;i > targetPos-releasePos; i-=stopDistStep){
			std::ostringstream iss;
			iss << i;
			nbrReceivedByDist.insert(std::pair<std::string,double>(iss.str(),0));
		}

    	if (with127M){
    		std::ostringstream iss;
			iss << 127;
			nbrReceivedByDist.insert(std::pair<std::string,double>(iss.str(),0));
    	}
    }
}

void SimpleApplNetw::handleSelfMsg(cMessage *msg)
{
	if (strcmp(msg->getName(),"Testing")==0){
		double vehLanePosition = 0;
		if (mobility != NULL){
			vehLanePosition = mobility->cmdGetVehiclelanePosition();
		}
		NetwPkt* pkt = new NetwPkt();
		std::ostringstream iss;
		if (vehLanePosition != 0){
			iss << targetPos-ceil(vehLanePosition);
			pkt->setName(iss.str().c_str());
		}
		pkt->setSrcAddr(myNetwAddr);
		pkt->setDestAddr(LAddress::L3BROADCAST);
		pkt->setBitLength(msgLength);
		sendDown(pkt);
		remainingMsgByStep--;
		if (remainingMsgByStep > 0){
			scheduleAt(simTime()+periodForMsg, msg);
		}

		nbrSent++;
	}
}

void SimpleApplNetw::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;

    NetwPkt *netwPkt = check_and_cast<NetwPkt *>(m->decapsulate());

    nbrReceived++;

    std::map<std::string,int>::iterator it = nbrReceivedByDist.find(netwPkt->getName());
    if (it == nbrReceivedByDist.end()){
    	nbrReceivedByDist.insert(std::pair<std::string,double>(netwPkt->getName(),1));
    }else{
    	nbrReceivedByDist[netwPkt->getName()] = it->second + 1;
    }

    delete netwPkt;
    delete m;
}

void SimpleApplNetw::finish()
{
	simulation.getSystemModule()->unsubscribe(vehStopped, this);
	cancelAndDelete(msg);
	recordScalar("# nbrSent", nbrSent);
	recordScalar("# nbrReceived", nbrReceived);

	for (std::map<std::string,int>::iterator it = nbrReceivedByDist.begin(); it != nbrReceivedByDist.end(); it++) {
		std::string tmp = "# nbrReceived_"+it->first;
		recordScalar(tmp.c_str(), it->second);
	}
}

void SimpleApplNetw::receiveSignal(cComponent *source, simsignal_t signalID, long  l)
{
	Enter_Method_Silent("receiveSignal()");
	if (strcmp(getSignalName(signalID),"vehStopped") == 0){
//	if (signalID == "vehStopped"){
		if (sentMsg){
			scheduleAt(simTime(),msg);
		}
	}
}

void SimpleApplNetw::receiveSignal(cComponent *source, simsignal_t signalID, double d)
{
	Enter_Method_Silent("receiveSignal()");
	if (strcmp(getSignalName(signalID),"vehStopped") == 0){
//	if (signalID == "vehStopped"){
		if (sentMsg){
			remainingMsgByStep = nbrMsgByStep;
			scheduleAt(simTime(),msg);
		}
	}
}





