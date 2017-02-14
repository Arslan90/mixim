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

    	vehHasBndls = 0;

//    	anyVPA = par("updateMode").boolValue();


    	/*
    	 * Section created by me for initializing dtnTestMode & silentMode booleans
    	 */

//    	delayStats.setName("DelayStats for 1st Copy");
    	delays.setName("Delays");

//	    hopCountStats.setName("HopCountStats for 1st Copy");
	    hopCountVector.setName("HopCount");

	    vehicleDensity.setName("Vehicle Density");

	    vehicleInContact.setName("Vehicle In Contact");

		vehPassedBy = 0;
		vehInRadio = 0;
		vehInRadioHasBndls = 0;
		vehHasBndls = 0;
		vehNotSentBndl = 0;
		vehSent0Bndl = 0;
		vehSentFewBndl = 0;
		vehSentAllBndl = 0;
		vehMeanBndlSent = 0;
		nbrMeanBndlSent = 0;
		vehMeanBndlSuccSent = 0;
		nbrMeanBndlSuccSent = 0;

		vVehPassedBy.setName("total vehs");
		vVehInRadio.setName("total vehs in contact");
		vVehHasBndls.setName("total vehs have bndls");
		vVehInRadioHasBndls.setName("total vehs in radio and have bndls");
		vVehNotSentBndl.setName("total vehNotSentBndls");
		vVehSent0Bndl.setName("total vehSent0Bndls");
		vVehSentFewBndl.setName("total vehSentFewBndls");
		vVehSentAllBndl.setName("total vehSentAllBndls");
		vVehMeanBndlSent.setName("Mean Bndls Sents");
		vVehMeanBndlSuccSent.setName("Mean Bndls Receiveds");

		totalMeanNeighborNotSent = 0;
		nbrMeanNeighborNotSent = 0;
		vVehMeanNeighborsNotSent.setName("Mean Neighbors NotSent");

		totalMeanNeighbor0Sent = 0;
		nbrMeanNeighbor0Sent = 0;
		vVehMeanNeighbors0Sent.setName("Mean Neighbors 0 Sent");

		totalMeanNeighborFewSent = 0;
		nbrMeanNeighborFewSent = 0;
		vVehMeanNeighborsFewSent.setName("Mean Neighbors Few Sent");

		totalMeanNeighborAllSent = 0;
		nbrMeanNeighborAllSent = 0;
		vVehMeanNeighborsAllSent.setName("Mean Neighbors All Sent");

		totalMeanVPAContactNotSent = 0;
		nbrMeanVPAContactNotSent = 0;
		vVehMeanVPAContactsNotSent.setName("Mean VPA Contacts NotSent");

		totalMeanVPAContact0Sent = 0;
		nbrMeanVPAContact0Sent = 0;
		vVehMeanVPAContacts0Sent.setName("Mean VPA Contacts 0 Sent");

		totalMeanVPAContactFewSent = 0;
		nbrMeanVPAContactFewSent = 0;
		vVehMeanVPAContactsFewSent.setName("Mean VPA Contacts Few Sent");

		totalMeanVPAContactAllSent = 0;
		nbrMeanVPAContactAllSent = 0;
		vVehMeanVPAContactsAllSent.setName("Mean VPA Contacts All Sent");

		totalMeanVPADistanceNotSent = 0;
		nbrMeanVPADistanceNotSent = 0;
		vVehMeanVPADistancesNotSent.setName("Mean VPA Distances NotSent");

		totalMeanVPADistance0Sent = 0;
		nbrMeanVPADistance0Sent = 0;
		vVehMeanVPADistances0Sent.setName("Mean VPA Distances 0 Sent");

		totalMeanVPADistanceFewSent = 0;
		nbrMeanVPADistanceFewSent = 0;
		vVehMeanVPADistancesFewSent.setName("Mean VPA Distances Few Sent");

		totalMeanVPADistanceAllSent = 0;
		nbrMeanVPADistanceAllSent = 0;
		vVehMeanVPADistancesAllSent.setName("Mean VPA Distances All Sent");

		vehCustody = 0;
		vehCusInRadio = 0;
		vehCusInRadioHasBndls = 0;
		vehCusNotSentBndl = 0;
		vehCusSent0Bndl = 0;
		vehCusSentFewBndl = 0;
		vehCusSentAllBndl = 0;

		vVehCustody.setName("total vehs with dist 0");
		vVehCusInRadio.setName("total VehCus in contact");
		vVehCusInRadioHasBndls.setName("total VehCus in radio and have bndls");
		vVehCusNotSentBndl.setName("total VehCus NotSentBndls");
		vVehCusSent0Bndl.setName("total VehCus Sent0Bndls");
		vVehCusSentFewBndl.setName("total VehCus SentFewBndls");
		vVehCusSentAllBndl.setName("total VehCus SentAllBndls");

		totalMeanCusNeighborNotSent = 0;
		nbrMeanCusNeighborNotSent = 0;
		vVehMeanCusNeighborsNotSent.setName("MeanCus Neighbors NotSent");

		totalMeanCusNeighbor0Sent = 0;
		nbrMeanCusNeighbor0Sent = 0;
		vVehMeanCusNeighbors0Sent.setName("MeanCus Neighbors 0 Sent");

		totalMeanCusNeighborFewSent = 0;
		nbrMeanCusNeighborFewSent = 0;
		vVehMeanCusNeighborsFewSent.setName("MeanCus Neighbors Few Sent");

		totalMeanCusNeighborAllSent = 0;
		nbrMeanCusNeighborAllSent = 0;
		vVehMeanCusNeighborsAllSent.setName("MeanCus Neighbors All Sent");

		totalMeanCusVPAContactNotSent = 0;
		nbrMeanCusVPAContactNotSent = 0;
		vVehMeanCusVPAContactsNotSent.setName("MeanCus VPA Contacts NotSent");

		totalMeanCusVPAContact0Sent = 0;
		nbrMeanCusVPAContact0Sent = 0;
		vVehMeanCusVPAContacts0Sent.setName("MeanCus VPA Contacts 0 Sent");

		totalMeanCusVPAContactFewSent = 0;
		nbrMeanCusVPAContactFewSent = 0;
		vVehMeanCusVPAContactsFewSent.setName("MeanCus VPA Contacts Few Sent");

		totalMeanCusVPAContactAllSent = 0;
		nbrMeanCusVPAContactAllSent = 0;
		vVehMeanCusVPAContactsAllSent.setName("MeanCus VPA Contacts All Sent");

		totalMeanCusVPADistanceNotSent = 0;
		nbrMeanCusVPADistanceNotSent = 0;
		vVehMeanCusVPADistancesNotSent.setName("MeanCus VPA Distances NotSent");

		totalMeanCusVPADistance0Sent = 0;
		nbrMeanCusVPADistance0Sent = 0;
		vVehMeanCusVPADistances0Sent.setName("MeanCus VPA Distances 0 Sent");

		totalMeanCusVPADistanceFewSent = 0;
		nbrMeanCusVPADistanceFewSent = 0;
		vVehMeanCusVPADistancesFewSent.setName("MeanCus VPA Distances Few Sent");

		totalMeanCusVPADistanceAllSent = 0;
		nbrMeanCusVPADistanceAllSent = 0;
		vVehMeanCusVPADistancesAllSent.setName("MeanCus VPA Distances All Sent");

		countRcvH = 0;
		countRcvB = 0;
		countRcvA = 0;

		totalRcvH = 0;
		totalRcvB = 0;
		totalRcvA = 0;

		vMeanRcvH.setName("Mean RcvH InRadio VPA");
		vMeanRcvB.setName("Mean RcvB InRadio VPA");
		vMeanRcvA.setName("Mean RcvA InRadio VPA");

		total0RcvH = 0;
		total0RcvB = 0;
		total0RcvA = 0;
		vMean0RcvH.setName("Mean RcvH0 InRadio VPA");
		vMean0RcvB.setName("Mean RcvB0 InRadio VPA");
		vMean0RcvA.setName("Mean RcvA0 InRadio VPA");
		total1RcvH = 0;
		total1RcvB = 0;
		total1RcvA = 0;
		vMean1RcvH.setName("Mean RcvH1 InRadio VPA");
		vMean1RcvB.setName("Mean RcvB1 InRadio VPA");
		vMean1RcvA.setName("Mean RcvA1 InRadio VPA");
		total2RcvH = 0;
		total2RcvB = 0;
		total2RcvA = 0;
		vMean2RcvH.setName("Mean RcvH2 InRadio VPA");
		vMean2RcvB.setName("Mean RcvB2 InRadio VPA");
		vMean2RcvA.setName("Mean RcvA2 InRadio VPA");
		total3RcvH = 0;
		total3RcvB = 0;
		total3RcvA = 0;
		vMean3RcvH.setName("Mean RcvH3 InRadio VPA");
		vMean3RcvB.setName("Mean RcvB3 InRadio VPA");
		vMean3RcvA.setName("Mean RcvA3 InRadio VPA");
		L20Sent = 0;
		L20Dropped = 0;
		L20Received = 0;
		L20Lost = 0;
		L20BackOff = 0;
		L20Duration = 0;
		vL20Sent.setName("total L20Sent");
		vL20Dropped.setName("total L20Dropped");
		vL20Received.setName("total L20Received");
		vL20Lost.setName("total L20Lost");
		vL20BackOff.setName("total L20BackOff");
		vL20Duration.setName("total L20Duration");
		L21Sent = 0;
		L21Dropped = 0;
		L21Received = 0;
		L21Lost = 0;
		L21BackOff = 0;
		L21Duration = 0;
		vL21Sent.setName("total L21Sent");
		vL21Dropped.setName("total L21Dropped");
		vL21Received.setName("total L21Received");
		vL21Lost.setName("total L21Lost");
		vL21BackOff.setName("total L21BackOff");
		vL21Duration.setName("total L21Duration");
		L22Sent = 0;
		L22Dropped = 0;
		L22Received = 0;
		L22Lost = 0;
		L22BackOff = 0;
		L22Duration = 0;
		vL22Sent.setName("total L22Sent");
		vL22Dropped.setName("total L22Dropped");
		vL22Received.setName("total L22Received");
		vL22Lost.setName("total L22Lost");
		vL22BackOff.setName("total L22BackOff");
		vL22Duration.setName("total L22Duration");
		L23Sent = 0;
		L23Dropped = 0;
		L23Received = 0;
		L23Lost = 0;
		L23BackOff = 0;
		L23Duration = 0;
		vL23Sent.setName("total L23Sent");
		vL23Dropped.setName("total L23Dropped");
		vL23Received.setName("total L23Received");
		vL23Lost.setName("total L23Lost");
		vL23BackOff.setName("total L23BackOff");
		vL23Duration.setName("total L23Duration");


//    	dtnTestMode = par("dtnTestMode").boolValue();
//    	silentMode = par("silentMode").boolValue();

    	/*
    	 * End of section
    	 */

		sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
//		nbrBundleSent = 0;
//		nbrBundleReceived = 0;
//		nbrUniqueBundleReceived = 0;

//		dtnSynchronized = par("dtnSynchronized").boolValue();
//		if (dtnTestMode){
//			avgDelay = 0;
//			totalDelay = 0;
//			/*
//			 * we are in dtnTestMode
//			 */
//			if (silentMode){
//				/*
//				 * we are in silentMode, VPA will not send any messages, it only receive
//				 */
//
//			}else {
//				//Schedule the next first VPA TX
//
//				scheduleAt(simTime() + 1, sendBeaconEvt);
//
//				/*NOTE: The VPAs can have overlap zones and vehicles may lost their updates due to
//				 * the hidden node problem. But if I add a CW the  VPAS will contend with the
//				 * vehicles for the TX/RX resource allocation. Maybe if the problem is so big
//				 * I'll give a lower CW for the VPAs.
//				 * BUT!, Base on the testing I've made with 802.11/802.11p, 80211p start to loss packets with ten
//				 * nodes TX at same time. for the 802.11 big problems arise with only 2 nodes.  */
//			}
//		}

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
	    		vVehPassedBy.record(vehPassedBy);
	    		vVehHasBndls.record(vehHasBndls);
	    		vVehInRadio.record(vehInRadio);
	    		vVehInRadioHasBndls.record(vehInRadioHasBndls);
	    		vVehNotSentBndl.record(vehNotSentBndl);
	    		vVehSent0Bndl.record(vehSent0Bndl);
	    		vVehSentFewBndl.record(vehSentFewBndl);
	    		vVehSentAllBndl.record(vehSentAllBndl);
//	    		double meanBndlSent = (vehMeanBndlSent == 0) ? 0:double(vehMeanBndlSent)/double(nbrMeanBndlSent);
//	    		vVehMeanBndlSent.record(meanBndlSent);
//	    		double meanBndlSuccSent = (vehMeanBndlSuccSent == 0) ? 0:double(vehMeanBndlSuccSent)/double(nbrMeanBndlSuccSent);
//	    		vVehMeanBndlSuccSent.record(meanBndlSuccSent);
	    		double meanNeighborsNotSent = (nbrMeanNeighborNotSent == 0) ? 0: double(double(totalMeanNeighborNotSent)/double(nbrMeanNeighborNotSent));
	    		vVehMeanNeighborsNotSent.record(meanNeighborsNotSent);

	    		double meanNeighbors0Sent = (nbrMeanNeighbor0Sent == 0) ? 0: double(double(totalMeanNeighbor0Sent)/double(nbrMeanNeighbor0Sent));
	    		vVehMeanNeighbors0Sent.record(meanNeighbors0Sent);

	    		double meanNeighborsFewSent = (nbrMeanNeighborFewSent == 0) ? 0: double(double(totalMeanNeighborFewSent)/double(nbrMeanNeighborFewSent));
	    		vVehMeanNeighborsFewSent.record(meanNeighborsFewSent);

	    		double meanNeighborsAllSent = (nbrMeanNeighborAllSent == 0) ? 0: double(double(totalMeanNeighborAllSent)/double(nbrMeanNeighborAllSent));
	    		vVehMeanNeighborsAllSent.record(meanNeighborsAllSent);

	    		double MeanVPAContactsNotSent = (nbrMeanVPAContactNotSent == 0) ? 0: double(double(totalMeanVPAContactNotSent)/double(nbrMeanVPAContactNotSent));
	    		vVehMeanVPAContactsNotSent.record(MeanVPAContactsNotSent);

	    		double MeanVPAContacts0Sent = (nbrMeanVPAContact0Sent == 0) ? 0: double(double(totalMeanVPAContact0Sent)/double(nbrMeanVPAContact0Sent));
	    		vVehMeanVPAContacts0Sent.record(MeanVPAContacts0Sent);

	    		double MeanVPAContactsFewSent = (nbrMeanVPAContactFewSent == 0) ? 0: double(double(totalMeanVPAContactFewSent)/double(nbrMeanVPAContactFewSent));
	    		vVehMeanVPAContactsFewSent.record(MeanVPAContactsFewSent);

	    		double MeanVPAContactsAllSent = (nbrMeanVPAContactAllSent == 0) ? 0: double(double(totalMeanVPAContactAllSent)/double(nbrMeanVPAContactAllSent));
	    		vVehMeanVPAContactsAllSent.record(MeanVPAContactsAllSent);

	    		double MeanVPADistancesNotSent = (nbrMeanVPADistanceNotSent == 0) ? 0: double(double(totalMeanVPADistanceNotSent)/double(nbrMeanVPADistanceNotSent));
	    		vVehMeanVPADistancesNotSent.record(MeanVPADistancesNotSent);

	    		double MeanVPADistances0Sent = (nbrMeanVPADistance0Sent == 0) ? 0: double(double(totalMeanVPADistance0Sent)/double(nbrMeanVPADistance0Sent));
	    		vVehMeanVPADistances0Sent.record(MeanVPADistances0Sent);

	    		double MeanVPADistancesFewSent = (nbrMeanVPADistanceFewSent == 0) ? 0: double(double(totalMeanVPADistanceFewSent)/double(nbrMeanVPADistanceFewSent));
	    		vVehMeanVPADistancesFewSent.record(MeanVPADistancesFewSent);

	    		double MeanVPADistancesAllSent = (nbrMeanVPADistanceAllSent == 0) ? 0: double(double(totalMeanVPADistanceAllSent)/double(nbrMeanVPADistanceAllSent));
	    		vVehMeanVPADistancesAllSent.record(MeanVPADistancesAllSent);

	    		vVehCustody.record(vehCustody);
	    		vVehCusInRadio.record(vehCusInRadio);
	    		vVehCusInRadioHasBndls.record(vehCusInRadioHasBndls);
	    		vVehCusNotSentBndl.record(vehCusNotSentBndl);
	    		vVehCusSent0Bndl.record(vehCusSent0Bndl);
	    		vVehCusSentFewBndl.record(vehCusSentFewBndl);
	    		vVehCusSentAllBndl.record(vehCusSentAllBndl);

	    		double MeanCusNeighborsNotSent = (nbrMeanCusNeighborNotSent == 0) ? 0: double(double(totalMeanCusNeighborNotSent)/double(nbrMeanCusNeighborNotSent));
	    		vVehMeanCusNeighborsNotSent.record(MeanCusNeighborsNotSent);

	    		double MeanCusNeighbors0Sent = (nbrMeanCusNeighbor0Sent == 0) ? 0: double(double(totalMeanCusNeighbor0Sent)/double(nbrMeanCusNeighbor0Sent));
	    		vVehMeanCusNeighbors0Sent.record(MeanCusNeighbors0Sent);

	    		double MeanCusNeighborsFewSent = (nbrMeanCusNeighborFewSent == 0) ? 0: double(double(totalMeanCusNeighborFewSent)/double(nbrMeanCusNeighborFewSent));
	    		vVehMeanCusNeighborsFewSent.record(MeanCusNeighborsFewSent);

	    		double MeanCusNeighborsAllSent = (nbrMeanCusNeighborAllSent == 0) ? 0: double(double(totalMeanCusNeighborAllSent)/double(nbrMeanCusNeighborAllSent));
	    		vVehMeanCusNeighborsAllSent.record(MeanCusNeighborsAllSent);

	    		double MeanCusVPAContactsNotSent = (nbrMeanCusVPAContactNotSent == 0) ? 0: double(double(totalMeanCusVPAContactNotSent)/double(nbrMeanCusVPAContactNotSent));
	    		vVehMeanCusVPAContactsNotSent.record(MeanCusVPAContactsNotSent);

	    		double MeanCusVPAContacts0Sent = (nbrMeanCusVPAContact0Sent == 0) ? 0: double(double(totalMeanCusVPAContact0Sent)/double(nbrMeanCusVPAContact0Sent));
	    		vVehMeanCusVPAContacts0Sent.record(MeanCusVPAContacts0Sent);

	    		double MeanCusVPAContactsFewSent = (nbrMeanCusVPAContactFewSent == 0) ? 0: double(double(totalMeanCusVPAContactFewSent)/double(nbrMeanCusVPAContactFewSent));
	    		vVehMeanCusVPAContactsFewSent.record(MeanCusVPAContactsFewSent);

	    		double MeanCusVPAContactsAllSent = (nbrMeanCusVPAContactAllSent == 0) ? 0: double(double(totalMeanCusVPAContactAllSent)/double(nbrMeanCusVPAContactAllSent));
	    		vVehMeanCusVPAContactsAllSent.record(MeanCusVPAContactsAllSent);

	    		double MeanCusVPADistancesNotSent = (nbrMeanCusVPADistanceNotSent == 0) ? 0: double(double(totalMeanCusVPADistanceNotSent)/double(nbrMeanCusVPADistanceNotSent));
	    		vVehMeanCusVPADistancesNotSent.record(MeanCusVPADistancesNotSent);

	    		double MeanCusVPADistances0Sent = (nbrMeanCusVPADistance0Sent == 0) ? 0: double(double(totalMeanCusVPADistance0Sent)/double(nbrMeanCusVPADistance0Sent));
	    		vVehMeanCusVPADistances0Sent.record(MeanCusVPADistances0Sent);

	    		double MeanCusVPADistancesFewSent = (nbrMeanCusVPADistanceFewSent == 0) ? 0: double(double(totalMeanCusVPADistanceFewSent)/double(nbrMeanCusVPADistanceFewSent));
	    		vVehMeanCusVPADistancesFewSent.record(MeanCusVPADistancesFewSent);

	    		double MeanCusVPADistancesAllSent = (nbrMeanCusVPADistanceAllSent == 0) ? 0: double(double(totalMeanCusVPADistanceAllSent)/double(nbrMeanCusVPADistanceAllSent));
	    		vVehMeanCusVPADistancesAllSent.record(MeanCusVPADistancesAllSent);

	    		double MeanRcvH = (countRcvH == 0) ? 0: double(double(totalRcvH)/double(countRcvH));
	    		vMeanRcvH.record(MeanRcvH);

	    		double MeanRcvB = (countRcvB == 0) ? 0: double(double(totalRcvB)/double(countRcvB));
	    		vMeanRcvB.record(MeanRcvB);

	    		double MeanRcvA = (countRcvA == 0) ? 0: double(double(totalRcvA)/double(countRcvA));
	    		vMeanRcvA.record(MeanRcvA);

	    		recordL2Stats();
	    		resetL20Stats();
	    		resetL21Stats();
	    		resetL22Stats();
	    		resetL23Stats();

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
//			int currentVehDensity = vehiclesAddr.size();
//			vehicleDensity.record(currentVehDensity);

			totalDelay = totalDelay + time.dbl();
			if (nbrUniqueBundleReceived>0){
				avgDelay = totalDelay / nbrUniqueBundleReceived;
			}else{
				opp_error("Nbr Unique Bundle Received equal or less then zero(VPApOpp::handleLowerMsg)");
			}

			delays.record(avgDelay);
			delayStats.collect(time.dbl());

			hopCountVector.record(wsm->getHopCount());
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
//	sendWSM(prepareWSM(result, beaconLengthBits, channel, beaconPriority, 0,2));
	sendWSM(prepareWSM(result, beaconLengthBits, channel, beaconPriority, 0,multiFunctions::cantorPairingFunc(netwAddr,nbrMsgSent)));
	//sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));

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
	//	MYDEBUG <<"logs, VEH," <<simTime() <<",From," << myApplAddr() << "," << traci->getExternalId()  <<",tx," <<  junctionID << ", messageSequence, " <<  messageSequence << ", messageSequenceVPA, " << messageSequenceVPA << ","<< vehPos.x <<","<<  axeY<<"," <<endl;
		//MYDEBUG <<"logs, backoff,tx,"<< currentSector <<","<< traci->getExternalId()<< ","  << simTime() << "," <<endl;


	//	char numstr[6]; // Numbered Message
	//	sprintf(numstr, "%d+%d", messageSequenceVPA,messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	//	char* result = numstr; //concatenate in VPAiD,messageSequence

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
		int oldSectorId,newSectorId,nbrBndls,inContact,isDistZero,totalNeighbors,nbrCountNeighbors;
		double totalVPAContactDur, nbrVPAContactDur, totalVPADistanceDur, nbrVPADistanceDur, L2BackoffDur;
		long rcvB, rcvH, rcvA, L2Sent, L2Drop, L2Rcv, L2Lost, L2Backoff;
		char* oldSectorChar = strtok(strdup(s),":");
		std::stringstream ss1 (oldSectorChar);
		ss1 >> oldSectorId;
		char* newSectorChar = strtok(NULL,":");
		std::stringstream ss2 (newSectorChar);
		ss2 >> newSectorId;
		std::stringstream ss5 (strtok(NULL,":"));
		ss5 >> nbrBndls;
		std::stringstream ss3 (strtok(NULL,":"));
		ss3 >> inContact;
		std::stringstream ss4 (strtok(NULL,":"));
		ss4 >> isDistZero;
		char* bndleSentSerials = strtok(NULL,":");
		std::stringstream ss6 (strtok(NULL,":"));
		ss6 >> totalNeighbors;
		std::stringstream ss7 (strtok(NULL,":"));
		ss7 >> nbrCountNeighbors;
		std::stringstream ss8 (strtok(NULL,":"));
		ss8 >> totalVPAContactDur;
		std::stringstream ss9 (strtok(NULL,":"));
		ss9 >> nbrVPAContactDur;
		std::stringstream ss10 (strtok(NULL,":"));
		ss10 >> totalVPADistanceDur;
		std::stringstream ss11 (strtok(NULL,":"));
		ss11 >> nbrVPADistanceDur;
		std::stringstream ss12 (strtok(NULL,":"));
		ss12 >> rcvH;
		std::stringstream ss13 (strtok(NULL,":"));
		ss13 >> rcvB;
		std::stringstream ss14 (strtok(NULL,":"));
		ss14 >> rcvA;
		std::stringstream ss15 (strtok(NULL,":"));
		ss15 >> L2Sent;
		std::stringstream ss16 (strtok(NULL,":"));
		ss16 >> L2Drop;
		std::stringstream ss17 (strtok(NULL,":"));
		ss17 >> L2Rcv;
		std::stringstream ss18 (strtok(NULL,":"));
		ss18 >> L2Lost;
		std::stringstream ss19 (strtok(NULL,":"));
		ss19 >> L2Backoff;
		std::stringstream ss20 (strtok(NULL,":"));
		ss20 >> L2BackoffDur;

		int sentToVPA = 0;
		int receivedByVPA = 0;
		char* serial = strtok(strdup(bndleSentSerials),",");
		while (serial != NULL){
			if (strcmp(serial,"")!=0){
				sentToVPA++;
			}
			if ((strcmp(serial,"")!=0)&&(receivedBundles.find(atoi(serial)) != receivedBundles.end())){
				receivedByVPA++;
			}
			serial = strtok(NULL,",");
		}

		bool contactedVPA;
		if (inContact == 1){
			contactedVPA = true;
		}else if (inContact == 0){
			contactedVPA = false;
		}else {
			opp_error("unable to recognize if veh contacted vpa or not");
		}

		bool custodyVeh;
		if ( isDistZero == 1){
			custodyVeh = true;
		}else if (isDistZero == 0){
			custodyVeh = false;
		}else {
			opp_error("unable to recognize if veh is a possible custody veh or not");
		}

		if (this->getParentModule()->getIndex() == oldSectorId){
			currentVehDensity--;
			vehPassedBy++;
			if ((rcvH >0) || (rcvB >0) || (rcvA >0)){
				cout << "debug found" << endl;
			}
			if (nbrBndls > 0){ vehHasBndls++;}
			if (contactedVPA){
				vehInRadio++;
				if (nbrBndls > 0){
					vehInRadioHasBndls++;
					if (rcvH >0){countRcvH++; totalRcvH+=rcvH;}
					if (rcvB >0){countRcvB++; totalRcvB+=rcvB;}
					if (rcvA >0){countRcvA++; totalRcvA+=rcvA;}
					if (sentToVPA == 0){
						totalMeanNeighborNotSent+=totalNeighbors;
						nbrMeanNeighborNotSent+=nbrCountNeighbors;
						totalMeanVPAContactNotSent+=totalVPAContactDur;
						nbrMeanVPAContactNotSent+=nbrVPAContactDur;
						totalMeanVPADistanceNotSent+=totalVPADistanceDur;
						nbrMeanVPADistanceNotSent+=nbrVPADistanceDur;
						vehNotSentBndl++;
						updateL20Stats(rcvH,rcvB,rcvA,L2Sent,L2Drop,L2Rcv,L2Lost,L2Backoff,L2BackoffDur);
					}else if (sentToVPA > 0){
						if (receivedByVPA == 0){
							totalMeanNeighbor0Sent+=totalNeighbors;
							nbrMeanNeighbor0Sent+=nbrCountNeighbors;
							totalMeanVPAContact0Sent+=totalVPAContactDur;
							nbrMeanVPAContact0Sent+=nbrVPAContactDur;
							totalMeanVPADistance0Sent+=totalVPADistanceDur;
							nbrMeanVPADistance0Sent+=nbrVPADistanceDur;
							vehSent0Bndl++;
							updateL21Stats(rcvH,rcvB,rcvA,L2Sent,L2Drop,L2Rcv,L2Lost,L2Backoff,L2BackoffDur);
						}else if (receivedByVPA > 0){
							if (receivedByVPA < sentToVPA){
								totalMeanNeighborFewSent+=totalNeighbors;
								nbrMeanNeighborFewSent+=nbrCountNeighbors;
								totalMeanVPAContactFewSent+=totalVPAContactDur;
								nbrMeanVPAContactFewSent+=nbrVPAContactDur;
								totalMeanVPADistanceFewSent+=totalVPADistanceDur;
								nbrMeanVPADistanceFewSent+=nbrVPADistanceDur;
								vehSentFewBndl++;
								updateL22Stats(rcvH,rcvB,rcvA,L2Sent,L2Drop,L2Rcv,L2Lost,L2Backoff,L2BackoffDur);
							}else if (receivedByVPA == sentToVPA){
								totalMeanNeighborAllSent+=totalNeighbors;
								nbrMeanNeighborAllSent+=nbrCountNeighbors;
								totalMeanVPAContactAllSent+=totalVPAContactDur;
								nbrMeanVPAContactAllSent+=nbrVPAContactDur;
								totalMeanVPADistanceAllSent+=totalVPADistanceDur;
								nbrMeanVPADistanceAllSent+=nbrVPADistanceDur;
								vehSentAllBndl++;
								updateL23Stats(rcvH,rcvB,rcvA,L2Sent,L2Drop,L2Rcv,L2Lost,L2Backoff,L2BackoffDur);
							}else {
								opp_error("Nbr msg received cannot be higher than those sent");
							}
						}else{
							opp_error("Unable to recognize if veh sent successfully bndls to VPA");
						}
					}else {
						opp_error("Unable to recognize if veh sent bndls to VPA");
					}
				}else if (nbrBndls ==0){
					// veh in radio but had not bndls
				}else {
					opp_error("Unable to recognize if veh had bndls");
				}
			}

			if (custodyVeh){
				vehCustody++;
				if (contactedVPA){
					vehCusInRadio++;
					if (nbrBndls > 0){
						vehCusInRadioHasBndls++;
						if (sentToVPA == 0){
							totalMeanCusNeighborNotSent+=totalNeighbors;
							nbrMeanCusNeighborNotSent+=nbrCountNeighbors;
							totalMeanCusVPAContactNotSent+=totalVPAContactDur;
							nbrMeanCusVPAContactNotSent+=nbrVPAContactDur;
							totalMeanCusVPADistanceNotSent+=totalVPADistanceDur;
							nbrMeanCusVPADistanceNotSent+=nbrVPADistanceDur;
							vehCusNotSentBndl++;
						}else if (sentToVPA > 0){
							if (receivedByVPA == 0){
								totalMeanCusNeighbor0Sent+=totalNeighbors;
								nbrMeanCusNeighbor0Sent+=nbrCountNeighbors;
								totalMeanCusVPAContact0Sent+=totalVPAContactDur;
								nbrMeanCusVPAContact0Sent+=nbrVPAContactDur;
								totalMeanCusVPADistance0Sent+=totalVPADistanceDur;
								nbrMeanCusVPADistance0Sent+=nbrVPADistanceDur;
								vehCusSent0Bndl++;
							}else if (receivedByVPA > 0){
								if (receivedByVPA < sentToVPA){
									totalMeanCusNeighborFewSent+=totalNeighbors;
									nbrMeanCusNeighborFewSent+=nbrCountNeighbors;
									totalMeanCusVPAContactFewSent+=totalVPAContactDur;
									nbrMeanCusVPAContactFewSent+=nbrVPAContactDur;
									totalMeanCusVPADistanceFewSent+=totalVPADistanceDur;
									nbrMeanCusVPADistanceFewSent+=nbrVPADistanceDur;
									vehCusSentFewBndl++;
								}else if (receivedByVPA == sentToVPA){
									totalMeanCusNeighborAllSent+=totalNeighbors;
									nbrMeanCusNeighborAllSent+=nbrCountNeighbors;
									totalMeanCusVPAContactAllSent+=totalVPAContactDur;
									nbrMeanCusVPAContactAllSent+=nbrVPAContactDur;
									totalMeanCusVPADistanceAllSent+=totalVPADistanceDur;
									nbrMeanCusVPADistanceAllSent+=nbrVPADistanceDur;
									vehCusSentAllBndl++;
								}else {
									opp_error("Nbr msg received cannot be higher than those sent");
								}
							}else{
								opp_error("Unable to recognize if veh sent successfully bndls to VPA");
							}
						}else {
							opp_error("Unable to recognize if veh sent bndls to VPA");
						}
					}else if (nbrBndls ==0){
						// veh in radio but had not bndls
					}else {
						opp_error("Unable to recognize if veh had bndls");
					}
				}
			}



//			vehInRadio = vehInRadio+inContact;
//			if (inContact == 1){
////				if((sentToVPA > 0) || (nbrBndls > 0)){
//				if((nbrBndls > 0)){
//					vehInRadioHasBndls++;
//					vehHasBndls ++;
//				}
//				if (sentToVPA > 0){
//					if (receivedByVPA != 0){
//						totalMeanNeighborSent+=totalNeighbors;
//						nbrMeanNeighborSent+=nbrCountNeighbors;
//					}else {
//						totalMeanNeighborNotSent+=totalNeighbors;
//						nbrMeanNeighborNotSent+=nbrCountNeighbors;
//					}
//				}
//			}else{
//				if (nbrBndls > 0){
//					vehHasBndls ++;
//				}
//			}
//			if (sentToVPA > 0){
//				if ((receivedByVPA == 0)&&(receivedByVPA < sentToVPA)){
//					vehSent0Bndl++;
//				}
//				if ((receivedByVPA > 0)&&(receivedByVPA < sentToVPA)){
//					vehSentFewBndl++;
//				}
//				if (receivedByVPA == sentToVPA){
//					vehSentAllBndl++;
//				}
//
//				vehMeanBndlSent+=sentToVPA;
//				nbrMeanBndlSent++;
//				if (receivedByVPA > 0){
//					vehMeanBndlSuccSent+=receivedByVPA;
//					nbrMeanBndlSuccSent++;
//				}
//			}
//			if ((inContact ==1) && (nbrBndls > 0) && (sentToVPA ==0)){
//				vehNotSentBndl++;
//			}
		}

		if (this->getParentModule()->getIndex() == newSectorId){
			currentVehDensity++;
//			vehicleDensity.record(currentVehDensity);
		}
//		cout << "Received signal with string: " << s << " old sector char: " << oldSectorChar << " new sector char: "<< newSectorChar << endl;

	}

	if (strcmp(getSignalName(signalID),"InContact") == 0){
		int SectorAddr,value,  haveBndlsId;
		char* SectorAddrChar = strtok(strdup(s),":");
		std::stringstream ss1 (SectorAddrChar);
		ss1 >> SectorAddr;
		char* valueChar = strtok(NULL,":");
		std::stringstream ss2 (valueChar);
		ss2 >> value;
//		char* haveBndlsChar = strtok(NULL,":");
//		std::stringstream ss3 (haveBndlsChar);
//		ss3 >> haveBndlsId;
		if (netwAddr == SectorAddr){
			currentVehInContact = currentVehInContact + value;
//			currentVehHaveBndls = currentVehHaveBndls + haveBndlsId;
		}
//		cout << "Received signal with string: " << s << " old sector char: " << oldSectorChar << " new sector char: "<< newSectorChar << endl;

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

void VPApOpp::onBeacon(WaveShortMessage* wsm) {
}

void VPApOpp::onData(WaveShortMessage* wsm) {
}

VPApOpp::~VPApOpp() {
}

void VPApOpp::finish()
{
	DtnApplLayer::finish();
	recordScalar("TotalRcvH",totalRcvH);
	recordScalar("TotalRcvB",totalRcvB);
	recordScalar("TotalRcvA",totalRcvA);

	recordScalar("CountRcvH",countRcvH);
	recordScalar("CountRcvB",countRcvB);
	recordScalar("CountRcvA",countRcvA);
	simulation.getSystemModule()->unsubscribe("sectorChanged", this);
	simulation.getSystemModule()->unsubscribe("InContact", this);
}

void VPApOpp::resetL20Stats()
{
	total0RcvH = 0;
	total0RcvB = 0;
	total0RcvA = 0;
	L20Sent 	= 0;
	L20Dropped 	= 0;
	L20Received = 0;
	L20Lost 	= 0;
	L20BackOff 	= 0;
	L20Duration = 0;
}



void VPApOpp::updateL20Stats(long  rcvH, long  rcvB, long  rcvA, long  L2S, long  L2D, long  L2R, long  L2L, long  L2Back, long  L2BackDur)
{
	total0RcvH += rcvH;
	total0RcvB += rcvB;
	total0RcvA += rcvA;
	L20Sent 	+= L2S;
	L20Dropped 	+= L2D;
	L20Received += L2R;
	L20Lost 	+= L2L;
	L20BackOff 	+= L2Back;
	L20Duration += L2BackDur;
}



void VPApOpp::resetL21Stats()
{
	total1RcvH = 0;
	total1RcvB = 0;
	total1RcvA = 0;
	L21Sent 	= 0;
	L21Dropped 	= 0;
	L21Received = 0;
	L21Lost 	= 0;
	L21BackOff 	= 0;
	L21Duration = 0;
}



void VPApOpp::updateL21Stats(long  rcvH, long  rcvB, long  rcvA, long  L2S, long  L2D, long  L2R, long  L2L, long  L2Back, long  L2BackDur)
{
	total1RcvH += rcvH;
	total1RcvB += rcvB;
	total1RcvA += rcvA;
	L21Sent 	+= L2S;
	L21Dropped 	+= L2D;
	L21Received += L2R;
	L21Lost 	+= L2L;
	L21BackOff 	+= L2Back;
	L21Duration += L2BackDur;
}



void VPApOpp::resetL22Stats()
{
	total2RcvH = 0;
	total2RcvB = 0;
	total2RcvA = 0;
	L22Sent 	= 0;
	L22Dropped 	= 0;
	L22Received = 0;
	L22Lost 	= 0;
	L22BackOff 	= 0;
	L22Duration = 0;
}



void VPApOpp::updateL22Stats(long  rcvH, long  rcvB, long  rcvA, long  L2S, long  L2D, long  L2R, long  L2L, long  L2Back, long  L2BackDur)
{
	total2RcvH += rcvH;
	total2RcvB += rcvB;
	total2RcvA += rcvA;
	L22Sent 	+= L2S;
	L22Dropped 	+= L2D;
	L22Received += L2R;
	L22Lost 	+= L2L;
	L22BackOff 	+= L2Back;
	L22Duration += L2BackDur;
}



void VPApOpp::resetL23Stats()
{
	total3RcvH = 0;
	total3RcvB = 0;
	total3RcvA = 0;
	L23Sent 	= 0;
	L23Dropped 	= 0;
	L23Received = 0;
	L23Lost 	= 0;
	L23BackOff 	= 0;
	L23Duration = 0;
}



void VPApOpp::updateL23Stats(long  rcvH, long  rcvB, long  rcvA, long  L2S, long  L2D, long  L2R, long  L2L, long  L2Back, long  L2BackDur)
{
	total3RcvH += rcvH;
	total3RcvB += rcvB;
	total3RcvA += rcvA;
	L23Sent 	+= L2S;
	L23Dropped 	+= L2D;
	L23Received += L2R;
	L23Lost 	+= L2L;
	L23BackOff 	+= L2Back;
	L23Duration += L2BackDur;
}

void VPApOpp::recordL2Stats()
{
	vMean0RcvH.record(total0RcvH);
	vMean0RcvB.record(total0RcvB);
	vMean0RcvA.record(total0RcvA);
	vMean1RcvH.record(total1RcvH);
	vMean1RcvB.record(total1RcvB);
	vMean1RcvA.record(total1RcvA);
	vMean2RcvH.record(total2RcvH);
	vMean2RcvB.record(total2RcvB);
	vMean2RcvA.record(total2RcvA);
	vMean3RcvH.record(total3RcvH);
	vMean3RcvB.record(total3RcvB);
	vMean3RcvA.record(total3RcvA);

	vL20Sent	.record(	L20Sent 	);
	vL20Dropped	.record(	L20Dropped 	);
	vL20Received.record(	L20Received );
	vL20Lost	.record(	L20Lost 	);
	vL20BackOff	.record(	L20BackOff 	);
	vL20Duration.record(	L20Duration );
	vL21Sent	.record(	L21Sent 	);
	vL21Dropped	.record(	L21Dropped 	);
	vL21Received.record(	L21Received );
	vL21Lost	.record(	L21Lost 	);
	vL21BackOff	.record(	L21BackOff 	);
	vL21Duration.record(	L21Duration );
	vL22Sent	.record(	L22Sent 	);
	vL22Dropped	.record(	L22Dropped 	);
	vL22Received.record(	L22Received );
	vL22Lost	.record(	L22Lost 	);
	vL22BackOff	.record(	L22BackOff 	);
	vL22Duration.record(	L22Duration );
	vL23Sent	.record(	L23Sent 	);
	vL23Dropped	.record(	L23Dropped 	);
	vL23Received.record(	L23Received );
	vL23Lost	.record(	L23Lost 	);
	vL23BackOff	.record(	L23BackOff 	);
	vL23Duration.record(	L23Duration );
}








