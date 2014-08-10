//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "BaseWaveApplLayer.h"

const simsignalwrap_t BaseWaveApplLayer::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void BaseWaveApplLayer::initialize(int stage) {
	BaseApplLayer::initialize(stage);

	if (stage==0) {
		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
		            getParentModule()->getParentModule());
		assert(myMac);
		myId = getId();
		headerLength = par("headerLength").longValue();

		sendBeacons = par("sendBeacons").boolValue();
		beaconLengthBits = par("beaconLengthBits").longValue();
		beaconPriority = par("beaconPriority").longValue();

		sendData = par("sendData").boolValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataOnSch = par("dataOnSch").boolValue();
		dataPriority = par("dataPriority").longValue();


		/* Arturo, Ok I'll create my own scheduler in my own program, not here.*/
		sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);

		//simulate asynchronous channel access
		double offSet = dblrand() * (par("beaconInterval").doubleValue()/2);
		offSet = offSet + floor(offSet/0.050)*0.050;

		if (sendBeacons) {
			//ARTURO, I'm using my onw cMessage
			//scheduleAt(simTime() + offSet, sendBeaconEvt);
		}

	}
}

WaveShortMessage*  BaseWaveApplLayer::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial) {
	WaveShortMessage* wsm =		new WaveShortMessage(name.c_str());
	wsm->addBitLength(headerLength);
	wsm->addBitLength(lengthBits);

	switch (channel) {
		case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
	}
	wsm->setPsid(0);
	wsm->setPriority(priority);
	wsm->setWsmVersion(1);
	wsm->setTimestamp(simTime());
	wsm->setSenderAddress(myId);
	wsm->setRecipientAddress(rcvId);
	wsm->setSenderPos(curPosition);
	wsm->setSerial(serial);

	if (name == "beacon") {
		DBG << "Apps: Creating Beacon with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}
	if (name == "data") {
		DBG << "Apps: Creating Data with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}

	return wsm;
}



//Receiving packets. I'm overriding this function in my own program.
void BaseWaveApplLayer::handleLowerMsg(cMessage* msg) {

	Mac80211Pkt* mac = dynamic_cast<Mac80211Pkt*>(msg);
	ASSERT(mac);

	WaveShortMessage*  wsm =  dynamic_cast<WaveShortMessage*>(mac->decapsulate());
	if (wsm != NULL) {

		if (std::string(wsm->getName()) == "beacon") {
			onBeacon(wsm);
		}
		else if (std::string(wsm->getName()) == "data") {
			onData(wsm);
		}
		else {
			ASSERT(false); //ASSERT Stops all programs.
		}
		delete(wsm);
	}
	else {
		DBG << "Apps: unknown message received\n";
	}

	delete(msg);
}

//SelfMessage
void BaseWaveApplLayer::handleSelfMsg(cMessage* msg) {
	switch (msg->getKind()) {
		case SEND_BEACON_EVT: {
			DBG << "logs, BaseWaveApplLayer: Self message: sending Message. " << endl;
			sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));

			//Reschedule the selfmessage
			//double offSet = dblrand() * (par("beaconInterval").doubleValue()/2);
			//offSet = offSet + floor(offSet/0.050)*0.050;
	        double CW= intuniform(0, 10)* 0.1;//computing CW
			double offSet= par("beaconInterval").doubleValue() + CW;//Arturo, I'll add just the interval (NO asynch channel access)
	        scheduleAt(simTime() + offSet, sendBeaconEvt);
			break;
		}
		default: {
			if (msg)
				DBG << "Apps: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
			break;
		}
	}
}

void BaseWaveApplLayer::sendWSM(WaveShortMessage* wsm) {
	sendDown(wsm);
}

void BaseWaveApplLayer::finish() {
	cancelAndDelete(sendBeaconEvt);
}

BaseWaveApplLayer::~BaseWaveApplLayer() {

}


/* OK; Creo que estas funciones de activar para activar el ROI de TRACI no las necesitare..*/
void BaseWaveApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	/*Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	*/
}

void BaseWaveApplLayer::handlePositionUpdate(cObject* obj) {
	/* ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
	curPosition = mobility->getCurrentPosition();
	*/
}

