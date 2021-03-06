/*
 * My file VPAp.cc based on file: TestWaveApplLayer.cc
 * This is the same as scenario but using the 802.11p MAC layer.
 * It not seems to be easy to handle the same version for two different standards.
 * But I've to be synchro in order to have same version in 802.11 and 802.11p
 *
 *
 */

#include "VPApOpp.h"
#include "multiFunctions.h"
//#include "ApplOppControlInfo.h"
#include "DtnNetwLayer.h"
#include "clistener.h"

Define_Module(VPApOpp);

void VPApOpp::initialize(int stage) {
	/*OK, here I'm trying to initialize everything in this part instead of using upper classes initialization */
	//BaseWaveApplLayer::initialize(stage);
	DtnApplLayer::initialize(stage); //IMPORTANT, It seems that I've to include upper class to avoid a crash.

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

    	receivedBundles = std::map<unsigned long, WaveShortMessage* >();
    	receivedBundles.clear();

    	vehiclesAddr = std::set<int>();
    	vehiclesAddr.clear();
    	updateSectorCycle = par("updateSectorCycle");
    	update = new cMessage("update veh density", UPDATE);
    	if (updateSectorCycle >=0){
    		scheduleAt(simTime()+updateSectorCycle,update);
    	}else{
    		opp_error("updateSectorCycle value cannot be negative(VPApOpp::initialize)");
    	}
    	currentVehDensity = 0;

    	currentVehInContact = 0;

    	/*
    	 * Section created by me for initializing dtnTestMode & silentMode booleans
    	 */

//    	delayStats.setName("DelayStats for 1st Copy");
    	delays.setName("Delays");

//	    hopCountStats.setName("HopCountStats for 1st Copy");
	    hopCountVector.setName("HopCount");

	    vehicleDensity.setName("Vehicle Density");

	    vehicleInContact.setName("Vehicle In Contact");

    	/*
    	 * End of section
    	 */

		sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);

		if (isEquiped && withDtnMsg && ((getDataSrcFromStrategy(strategy)=="ALL") || (getDataSrcFromStrategy(strategy)=="VPA"))){
			// we have to send bundles
			if (sendingStrategy == SectorEntry){
				opp_error("SectorEntry sending strategy not supported for VPA");
			}else if(sendingStrategy == Periodic){
				double tmp = (dtnMsgSynchronized)? 0: uniform(0,dtnMsgPeriod);
				if ((dtnMsgMinTime <= simTime().dbl() + tmp) && (simTime().dbl() + tmp <= dtnMsgMaxTime)) {
					scheduleAt(simTime() + tmp, dtnMsg);
					nbrMsgSent++;
				}
			}
		}

		if (withOtherMsg){
			//Schedule the next first VPA TX

			scheduleAt(simTime() + 1, sendBeaconEvt);

			/*NOTE: The VPAs can have overlap zones and vehicles may lost their updates due to
			 * the hidden node problem. But if I add a CW the  VPAS will contend with the
			 * vehicles for the TX/RX resource allocation. Maybe if the problem is so big
			 * I'll give a lower CW for the VPAs.
			 * BUT!, Base on the testing I've made with 802.11/802.11p, 80211p start to loss packets with ten
			 * nodes TX at same time. for the 802.11 big problems arise with only 2 nodes.  */
		}

		simulation.getSystemModule()->subscribe("sectorChanged", this);

		simulation.getSystemModule()->subscribe("InContact", this);

	}
}


//handle self-Messages
void VPApOpp::handleSelfMsg(cMessage* msg) {

	if (isEquiped){

	switch (msg->getKind()) {
		case DTN_TEST_MODE:
			if (withDtnMsg){
				sendDtnMessage();
	    		// Finally reschedule message
				if(withDtnMsg && (sendingStrategy == Periodic) && ((dtnMsgMinTime <= simTime().dbl() + dtnMsgPeriod) && (simTime().dbl() + dtnMsgPeriod <= dtnMsgMaxTime))) {
					scheduleAt(simTime() + dtnMsgPeriod, dtnMsg);
					nbrMsgSent++;
				}
			}
			break;
		case SEND_BEACON_EVT: {
			DBG << "logs, VPA self MESSAGE" << endl;

	        sendVPApBroadcast(messageSequence++); //Sending periodic VPA Beacon (BROADCAST_MESSAGE) with broadcast counter.

			//Reschedule the self-message
			scheduleAt(simTime() + T, sendBeaconEvt);
	        EV <<"logs, T time: " << T <<endl;
	        nbrMsgSent++;
			break;
		}
		case UPDATE: {
			vehiclesAddr.clear();
	    	if (updateSectorCycle >=0){
	    		scheduleAt(simTime()+updateSectorCycle,update);
	    		vehicleDensity.record(currentVehDensity);
	    		vehicleInContact.record(currentVehInContact);
	    	}else{
	    		opp_error("updateSectorCycle value cannot be negative(VPApOpp::handleSelfMsg)");
	    	}
			break;
		}
		default: {
			if (msg)
				DBG << "logs, Unkown selfmessage kind: " << msg->getName() << endl;
			break;
		}
	}

	}
}


//Receiving packets. I'm overriding this class 'cause the inheret class causes problems.
void VPApOpp::handleLowerMsg(cMessage* msg) {

	NetwPkt* netw = dynamic_cast<NetwPkt*>(msg);
	ASSERT(netw);
	WaveShortMessage*  wsm =  dynamic_cast<WaveShortMessage*>(netw->decapsulate());

	nbrMsgSent++;

	if (wsm != NULL) {
		EV << "logs, Receiving packet " << wsm->getName() <<endl;
	}
	if (wsm->getKind()==DTN_TEST_MODE){
		nbrBundleReceived++;
		simtime_t time = (simTime()-wsm->getTimestamp());

		bool existUnderOtherVPA = false;

		if (receivingStrategy == Any){
			existUnderOtherVPA = bundleExistUnderOtherVPA(wsm->getSerial());
		}


		if (((receivedBundles.empty())	||	(receivedBundles.find(wsm->getSerial())== receivedBundles.end()))
			&& (!existUnderOtherVPA)){

			receivedBundles.insert(std::pair<unsigned long long ,WaveShortMessage*>(wsm->getSerial(), wsm));
			nbrUniqueBundleReceived++;

			vehiclesAddr.insert(wsm->getSenderAddress());

			totalDelay = totalDelay + time.dbl();
			if (nbrUniqueBundleReceived>0){
				avgDelay = totalDelay / nbrUniqueBundleReceived;
			}else{
				opp_error("Nbr Unique Bundle Received equal or less then zero(VPApOpp::handleLowerMsg)");
			}

			delays.record(avgDelay);
			delayStats.collect(time.dbl());

			totalHops = totalHops + wsm->getHopCount();
			if (nbrUniqueBundleReceived>0){
				avgHops = totalHops / nbrUniqueBundleReceived;
			}else{
				opp_error("Nbr Unique Bundle Received equal or less then zero(VPApOpp::handleLowerMsg)");
			}

			hopCountVector.record(avgHops);
			hopCountStats.collect(wsm->getHopCount());

			emit(receiveSignalId, nbrUniqueBundleReceived);

		}
	}

	delete(msg);
	delete wsm;
}


void VPApOpp::sendVPApBroadcast(int messageSequence) {

	//This paragraph is to transmit a message (VAPiD+messageSequence).
	char numstr[5]; // Numbered Message
	sprintf(numstr, "%d+%d", myApplAddr(),messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	char* result = numstr; //concatenate in VPAiD,messageSequence

	//Interesting! when using the SCH instead the CCH I got worst TX/RX performance.
	//Anyway I do not have time to check it out. I'll stay tuned with CCH.
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	sendWSM(prepareWSM(result, beaconLengthBits, channel, beaconPriority, 0,multiFunctions::cantorPairingFunc(netwAddr,nbrMsgSent)));

	//EV << "logs,tx,"<< simTime() <<",Sending VPA broadcast packet!" <<","<< endl;
	//EV << "logs, backoff,tx,"<< myApplAddr() <<","<< simTime() <<",VPA broadcast packet!" <<","<< endl;
	EV << "logs, VPA,"<< simTime() <<",From,"<< myApplAddr() <<",,tx,"<<",,,,,,,"<<endl;//equivalente a Vehicle

}


//OVERRIDING THE SENT MESSAGE 'CAUSE I NEED TO ADD THE SETKIND MESSAGE.
WaveShortMessage*  VPApOpp::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, unsigned long serial) {
	WaveShortMessage* wsm =		new WaveShortMessage(name.c_str());
	wsm->addBitLength(headerLength);
	wsm->addBitLength(lengthBits);

	switch (channel) {
		case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
	}
	if (withDtnMsg){
		wsm->setKind(DTN_TEST_MODE);
	}else {
		wsm->setKind(BROADCAST_VPA_WMS);//30=BROADCAST_VPA_WMS, 40=BROADCAST_VEH_WMS
	}

	wsm->setPsid(0);
	wsm->setPriority(priority);
	wsm->setWsmVersion(1);
	wsm->setTimestamp(simTime());
	wsm->setSenderAddress(netwAddr);
	wsm->setRecipientAddress(rcvId);
	wsm->setSenderPos(curPosition);
	wsm->setSerial(serial);

	return wsm;
}

bool VPApOpp::bundleExistUnderOtherVPA(unsigned long  serial)
{
	bool exist = false;

	cModule *systemModule = this->getParentModule();
	while (systemModule->getParentModule() !=NULL){
		systemModule = systemModule->getParentModule();
	}
	int numberVPA = systemModule->par("numeroNodes");

	cModule *vpa;
	for (int i = 0; i < numberVPA; ++i) {
		vpa = systemModule->getSubmodule("VPA", i);
		if (vpa!=NULL){
			cModule *appl = vpa->getSubmodule("appl");
			if (appl!=NULL){
				VPApOpp *vpa_appl = check_and_cast<VPApOpp*>(appl);
				std::map<unsigned long ,WaveShortMessage*> receivedBundleOtherVPA = vpa_appl->getReceivedBundles();

				if ((! receivedBundleOtherVPA.empty()) && (receivedBundleOtherVPA.find(serial) != receivedBundleOtherVPA.end())){
					exist = true;
				}
			}
		}
		if (exist){
			break;
		}
	}
	return exist;
}

void VPApOpp::sendDtnMessage()
{
	if (scenarioModel == Sector){
		opp_error("Sending strategy behaviour undefined from VPA to VPA when in Sector Model");
	}

	if (getDataDestFromStrategy(strategy)=="VPA"){
		int addr = 0;
		switch (receivingStrategy) {
			case Unique:
				addr = destAddr;
				break;
			case Random:
				addr = randomVPADestAddr();
				break;
			case Any:
				opp_error("Must Define an anycast addr for Free model");
				break;
			default:
				opp_error("Undefined behavior when selecting destinated VPA");
				break;
		}

		//Sending message
		t_channel channel = dataOnSch ? type_SCH : type_CCH;
		//	sendWSM(prepareWSM(result, dataLengthBits, channel, dataPriority, 0,2));

		DBG <<"periodic DtnMessage sent at " <<simTime() <<",From," << netwAddr << " to address "<< addr <<endl;
		std::string s = "Periodic DTN message sent from :"+netwAddr;
		std::string tmp = "to the current netw addr : "+addr;
		s = s+tmp;
		if (isNetwAddrInit){
		sendWSM(prepareWSM(s, dataLengthBits, channel, dataPriority, addr,multiFunctions::cantorPairingFunc(netwAddr,nbrMsgSent)));
		nbrBundleSent++;
		}else {
			opp_error("netw address not yet initialized");
		}
	}else{
		opp_error("Sending DtnMsg to VEH & ALL mode is not supported");
	}
}

void VPApOpp::receiveSignal(cComponent *source, simsignal_t signalID, const char *s)
{
	Enter_Method_Silent();
	if (strcmp(getSignalName(signalID),"sectorChanged") == 0){
		// TO DO
	}

	if (strcmp(getSignalName(signalID),"InContact") == 0){
		// TO DO
	}
}

int VPApOpp::randomVPADestAddr()
{
	int vpaDestAddr = -2;
	cModule *systemModule = this->getParentModule();
	while (systemModule->getParentModule() !=NULL){
		systemModule = systemModule->getParentModule();
	}
	int numberVPA = 0;
	if (systemModule->hasPar("numeroNodes")){
		numberVPA = systemModule->par("numeroNodes");
	}
	if (!systemModule->hasPar("numeroNodes") || (0 <= numberVPA)){
		opp_error("Impossible to determine number of VPA");
	}

	do {
		vpaDestAddr = intuniform(0,numberVPA-1);
		cModule *vpa = systemModule->getSubmodule("VPA", vpaDestAddr);
		if (vpa!=NULL){
			DtnNetwLayer *netw = FindModule<DtnNetwLayer*>::findSubModule(vpa);
			if (netw!=NULL){
				vpaDestAddr = netw->getMyNetwAddr();
			}
		}
	}while (vpaDestAddr == netwAddr);

	if (vpaDestAddr == -2){
		opp_error("Impossible to determine VPA address");
	}

	return vpaDestAddr;
}

/************** TO DELETE / TESTING AREA *************/
/************** TO DELETE / TESTING AREA *************/

VPApOpp::~VPApOpp() {
}

void VPApOpp::finish()
{
	DtnApplLayer::finish();
	simulation.getSystemModule()->unsubscribe("sectorChanged", this);
	simulation.getSystemModule()->unsubscribe("InContact", this);
}
