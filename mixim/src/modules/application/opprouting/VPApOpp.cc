/*
 * My file VPAp.cc based on file: TestWaveApplLayer.cc
 * This is the same as scenario but using the 802.11p MAC layer.
 * It not seems to be easy to handle the same version for two different standards.
 * But I've to be synchro in order to have same version in 802.11 and 802.11p
 *
 *
 */

#include "VPApOpp.h"

Define_Module(VPApOpp);

void VPApOpp::initialize(int stage) {
	/*OK, here I'm trying to initialize everything in this part instead of using upper classes initialization */
	//BaseWaveApplLayer::initialize(stage);
	BaseApplLayer::initialize(stage); //IMPORTANT, It seems that I've to include upper class to avoid a crash.

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

    	//Receive the T value
        T = hasPar("timeT") ? par("timeT").doubleValue(): 5;
        EV <<"logs, T time: " << T <<endl;
        messageSequence = 0; //First message sequence.
    	CW= 0; //Contention Window value.

		//Borrar automaticamente el directorio nic cuando se cree el primer VPA.
		if (myId== 8)
			system("rm  nic/*.txt");

    	//Schedule the next first VPA TX
		sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
		scheduleAt(simTime() + 1, sendBeaconEvt);

		/*NOTE: The VPAs can have overlap zones and vehicles may lost their updates due to
		 * the hidden node problem. But if I add a CW the  VPAS will contend with the
		 * vehicles for the TX/RX resource allocation. Maybe if the problem is so big
		 * I'll give a lower CW for the VPAs.
		 * BUT!, Base on the testing I've made with 802.11/802.11p, 80211p start to loss packets with ten
		 * nodes TX at same time. for the 802.11 big problems arise with only 2 nodes.  */
	}
}


//handle self-Messages
void VPApOpp::handleSelfMsg(cMessage* msg) {
	switch (msg->getKind()) {
		case SEND_BEACON_EVT: {
			DBG << "logs, VPA self MESSAGE" << endl;

	        sendVPApBroadcast(messageSequence++); //Sending periodic VPA Beacon (BROADCAST_MESSAGE) with broadcast counter.

			//Reschedule the self-message
			scheduleAt(simTime() + T, sendBeaconEvt);
	        EV <<"logs, T time: " << T <<endl;
			break;
		}
		default: {
			if (msg)
				DBG << "logs, Unkown selfmessage kind: " << msg->getName() << endl;
			break;
		}
	}
}


//Receiving packets. I'm overriding this class 'cause the inheret class causes problems.
void VPApOpp::handleLowerMsg(cMessage* msg) {

	NetwPkt* netw = dynamic_cast<NetwPkt*>(msg);
	ASSERT(netw);
	WaveShortMessage*  wsm =  dynamic_cast<WaveShortMessage*>(netw->decapsulate());
	if (wsm != NULL) {
		EV << "logs, Receiving packet " << wsm->getName() <<endl;
	delete(msg);
	}
}


void VPApOpp::sendVPApBroadcast(int messageSequence) {

	//This paragraph is to transmit a message (VAPiD+messageSequence).
	char numstr[5]; // Numbered Message
	sprintf(numstr, "%d+%d", myApplAddr(),messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	char* result = numstr; //concatenate in VPAiD,messageSequence

	//Interesting! when using the SCH instead the CCH I got worst TX/RX performance.
	//Anyway I do not have time to check it out. I'll stay tuned with CCH.
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	sendWSM(prepareWSM(result, beaconLengthBits, channel, beaconPriority, 0,2));
	//sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));

	//EV << "logs,tx,"<< simTime() <<",Sending VPA broadcast packet!" <<","<< endl;
	//EV << "logs, backoff,tx,"<< myApplAddr() <<","<< simTime() <<",VPA broadcast packet!" <<","<< endl;
	EV << "logs, VPA,"<< simTime() <<",From,"<< myApplAddr() <<",,tx,"<<",,,,,,,"<<endl;//equivalente a Vehicle

}


//OVERRIDING THE SENT MESSAGE 'CAUSE I NEED TO ADD THE SETKIND MESSAGE.
WaveShortMessage*  VPApOpp::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial) {
	WaveShortMessage* wsm =		new WaveShortMessage(name.c_str());
	wsm->addBitLength(headerLength);
	wsm->addBitLength(lengthBits);

	switch (channel) {
		case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
	}
	wsm->setKind(BROADCAST_VPA_WMS);//30=BROADCAST_VPA_WMS, 40=BROADCAST_VEH_WMS
	wsm->setPsid(0);
	wsm->setPriority(priority);
	wsm->setWsmVersion(1);
	wsm->setTimestamp(simTime());
	wsm->setSenderAddress(myId);
	wsm->setRecipientAddress(rcvId);
	wsm->setSenderPos(curPosition);
	wsm->setSerial(serial);

	return wsm;
}



/************** TO DELETE / TESTING AREA *************/
/************** TO DELETE / TESTING AREA *************/

void VPApOpp::onBeacon(WaveShortMessage* wsm) {
}

void VPApOpp::onData(WaveShortMessage* wsm) {
}

VPApOpp::~VPApOpp() {
}


