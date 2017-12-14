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

#include "GeoDtnICNetwLayer.h"
#include "math.h"
#include "algorithm"
#include "multiFunctions.h"

Define_Module(GeoDtnICNetwLayer);

void GeoDtnICNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
	if (stage == 0){
		withMETDFwd = par("withMETDFwd").boolValue();
		withDistFwd = par("withDistFwd").boolValue();
		if (!(withMETDFwd || withDistFwd)) {
			opp_error("No valid forwarder, please choose either one of them or both");
		}

		custodyMode = par("custodyMode");
		if ((custodyMode != No) && (custodyMode != Yes_WithoutACK) && (custodyMode != Yes_WithACK)){
			opp_error("No valid CustodyMode");
		}

		withCBH = par("withCBH").boolValue();

		withExplicitE2EAck = par("withExplicitE2EAck").boolValue();

		multiMetricFwdStrat = par("multiMetricFwdStrat");
		if ((multiMetricFwdStrat != AtLeastOne) && (multiMetricFwdStrat != Both)){
			opp_error("No valid Forwarding Strategy");
		}

		nbr2Fwds = 0;
		nbr1Fwds = 0;
		nbr0ValidFwds = 0;
		nbr1ValidFwds = 0;

		currentNbrIsrt = 0;
		lastNbrIsrt = 0;

		bndlInterestVec.setName("Evolve of interesting bundle of neighborhood");

		missedOpprVec.setName("Evolve of missed opportunities");

		inRadioWithVPA = registerSignal("InContact");

		gDistFwd = 0;
		bDistFwd = 0;
		gMETDFwd = 0;
		bMETDFwd = 0;

		Fwd_No  		= 0;
		Fwd_Yes_METD    = 0;
		Fwd_Yes_Dist    = 0;
		Fwd_Yes_Both    = 0;

		withAddressedAck = par("withAddressedAck").boolValue();

		custodyList = par("custodyList");
		if ((custodyList != No_Diffuse) && (custodyList != Diffuse) && (custodyList != Diffuse_Delete)){
			opp_error("No valid CustodyList");
		}

		majorationOfCustodyTimestamp = par("majorationOfCustodyTimestamp").doubleValue();

	}
}

void GeoDtnICNetwLayer::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    GeoDtnNetwPkt *netwPkt = check_and_cast<GeoDtnNetwPkt *>(m->decapsulate());

    coreEV << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Addressed to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

   	if (isEquiped){
		switch (netwPkt->getKind()) {
			case HELLO:
				handleHelloMsg(netwPkt);
				updateInRadioWithVPA(HELLO,netwPkt->getSrcType());
				break;
			case Bundle:
				if (netwPkt->getDestAddr() == myNetwAddr){
					handleBundleMsg(netwPkt);
					updateInRadioWithVPA(Bundle,netwPkt->getSrcType());
				}
				break;
			case Bundle_Ack:
				// if sent acks are addressed to me or
				// they are broadcasted
				if ((withAddressedAck && (netwPkt->getDestAddr() == myNetwAddr)) || (!withAddressedAck)){
//				if (!withAddressedAck){
					handleBundleAckMsg(netwPkt);
					updateInRadioWithVPA(Bundle_Ack,netwPkt->getSrcType());
//				} else if (withAddressedAck){
//					if (netwPkt->getDestAddr() == myNetwAddr){
//						handleBundleAckMsg(netwPkt);
//						updateInRadioWithVPA(Bundle_Ack,netwPkt->getSrcType());
//					}
				}
				break;
			default:
				opp_error("Unknown or unsupported DtnNetwMsgKinds when calling HandleLowerMsg()");
				break;
		}
	}

    updatingL3Received();

    delete netwPkt;
    delete msg;
}

void GeoDtnICNetwLayer::handleSelfMsg(cMessage *msg)
{
	if (msg == heartBeatMsg){
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,getCurrentMETD(),getCurrentDist(), simTime(), true, nodeType, getCurrentPos()));
		sendingHelloMsg();
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void GeoDtnICNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
	std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(upper_msg->getSerial());
	if (it == bundlesReplicaIndex.end()){
		bundlesReplicaIndex.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), 0));
		currentNbrIsrt++;
	}
}

void GeoDtnICNetwLayer::finish()
{
	recordScalar("# Distinct Forwarders", nbr2Fwds);
	recordScalar("# Same Forwarders", nbr1Fwds);
	recordScalar("# Unique valid Forwarders", nbr1ValidFwds);
	recordScalar("# No valid Forwarders", nbr0ValidFwds);

	recordScalar("Bad choice of METD Fwd", bMETDFwd);
	recordScalar("Bad choice of Dist Fwd", bDistFwd);
	recordScalar("Good choice of METD Fwd", gMETDFwd);
	recordScalar("Good choice of Dist Fwd", gMETDFwd);

	recordScalar("# No Forwarding", Fwd_No);
	recordScalar("# Yes Forwarding based on METD", Fwd_Yes_METD);
	recordScalar("# Yes Forwarding based on Dist", Fwd_Yes_Dist);
	recordScalar("# Yes Forwarding based on Both", Fwd_Yes_Both);

	recordAllScalars();
}

void GeoDtnICNetwLayer::sendingHelloMsg()
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
	double myCurrentDist = getCurrentDist();
	double myCurrentMETD = getCurrentMETD();
	netwPkt->setSrcMETD(myCurrentMETD);
	netwPkt->setSrcDist_NP_VPA(myCurrentDist);
	int nbrEntries = 0;
	if (checkBeforeHelloMechanism()){
		/***************** Cleaning AckSerials from old entries *****/
		if (withTTLForCtrl){
			deletedAckSerials();
		}
		/***************** Cleaning AckSerials from old entries *****/
		std::set<unsigned long> storedAck = std::set<unsigned long>(ackSerial);
		netwPkt->setE2eAcks(storedAck);
		std::set<unsigned long > storedBundle;
		for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
			storedBundle.insert((*it)->getSerial());
		}
		netwPkt->setH2hAcks(storedBundle);
		std::set<unsigned long > custodyBundle;
		if (withDistFwd && (custodyMode != No) && (custodyList != No_Diffuse)){
			/***************** Cleaning CustodySerials from old entries *****/
			if (withTTLForCtrl){
				deletedCustodySerials();
			}
			/***************** Cleaning CustodySerials from old entries *****/
			// If we use Dist metric, Custody mode is Yes and Custody list is > 0
			//custodyBundle = std::set<unsigned long >(custodySerial);
			custodyBundle = buildCustodySerialWithTimeStamp();
			if (myCurrentDist == 0){
				for(std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
					unsigned long myCustodySerial = (*it);
					unsigned long myCustodyTimestamp;
					if ((myCurrentMETD <= 0)||(myCurrentMETD == maxDbl)) {
						myCustodyTimestamp = 0;
					}else{
						myCustodyTimestamp = (unsigned long) (ceil((myCurrentMETD * majorationOfCustodyTimestamp + simTime().dbl())));
					}
					unsigned long myCustodyWithTimestamp = multiFunctions::cantorPairingFunc(myCustodySerial, myCustodyTimestamp);

					custodyBundle.insert(myCustodyWithTimestamp);
				}
			}
			netwPkt->setCustodySerials(custodyBundle);
		}
		nbrEntries = ackSerial.size()+ storedBundle.size()+ custodyBundle.size();

//		nbrEntries = ackSerial.size()+ storedBundle.size();
//		std::set<unsigned long > custodyBundle = std::set<unsigned long >(custodySerial);
//		if (netwPkt->getSrcDist_NP_VPA() == 0){
////			std::set<unsigned long > custodyBundle = std::set<unsigned long >(storedBundle);
//			for(std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
//				custodyBundle.insert(*it);
//			}
//		}
//		netwPkt->setCustodySerials(custodyBundle);
//		nbrEntries+=custodyBundle.size();
//		if (custodyMode == Yes_WithACK){
//			netwPkt->setCustodyAcks(custodyAckSerial);
//			nbrEntries+= custodyAckSerial.size();
//		}
	}
	long helloControlBitLength = sizeof(unsigned long) * (nbrEntries) *8;
	int length = helloControlBitLength+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt,helloControlBitLength, 0, 0);
}

void GeoDtnICNetwLayer::handleHelloMsg(GeoDtnNetwPkt *netwPkt)
{
	// If not the same sector ignore message
	if (netwPkt->getVpaSectorId() != sectorId){
		return;
	}else{
		/*************************** Handling Hello Msg **********/
	    NetwRoute neighborEntry = NetwRoute(netwPkt->getSrcAddr(), netwPkt->getSrcMETD(), netwPkt->getSrcDist_NP_VPA(), simTime() , true, netwPkt->getSrcType(), netwPkt->getCurrentPos());
	    updateNeighborhoodTable(netwPkt->getSrcAddr(), neighborEntry);
	    std::set<unsigned long> receivedE2eAcks = netwPkt->getE2eAcks();
	    if (!receivedE2eAcks.empty()){
	    	updateStoredAcksForSession(netwPkt->getSrcAddr(), receivedE2eAcks);
	    	storeAckSerials(receivedE2eAcks);
//	    	for(std::set<unsigned long >::iterator it = receivedE2eAcks.begin(); it != receivedE2eAcks.end(); it++){
//	    		custodyAckSerial.erase(*it);
//	    	}
	    }
	    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
	    if (!storedBundle.empty()){
	    	updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
	    }
	    std::set<unsigned long > custodyBundle = netwPkt->getCustodySerials();
		if (withDistFwd && (custodyMode != No) && (!custodyBundle.empty()) && (custodyList != No_Diffuse)){
	    	for(std::set<unsigned long >::iterator it = custodyBundle.begin(); it != custodyBundle.end(); it++){
	    		std::pair<unsigned long, unsigned long > myCustodySerialWithTimeStamp = multiFunctions::inverseCantorPairingFunc((*it));
	    		storeCustodySerial(*it);
	    		if ((custodyList == Diffuse_Delete) && (getCurrentDist() != 0)){
	    			erase(myCustodySerialWithTimeStamp.first);
	    			//erase(*it);
	    		}
	    	}
	    }
//	    if ((custodyMode == Yes_WithACK)||(custodyMode == Yes_WithoutACK)){
//			/** if the encoutered node is going to pass by the VPA so no need to request theses bundles instead delete them */
//			if(( netwPkt->getSrcDist_NP_VPA() == 0)){
//				for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
//					erase(*it);
//				}
//			}
//	    }
//	    if (custodyMode == Yes_WithACK){
//		    std::set<unsigned long> custodyAcks = netwPkt->getCustodyAcks();
//		    if (!custodyAcks.empty()){
//		    	storeCustodyAckSerials(custodyAcks);
//		    }
//	    }
	    /*************************** Sending Bundle Msg **********/
		if (nodeType == Veh){
		    if (netwPkt->getSrcType() == VPA){
		    	sendingBundleMsgToVPA(netwPkt->getSrcAddr());
		    	vpaContactDistance.push_back(getCurrentPos().distance(netwPkt->getCurrentPos()));
		    }else if (netwPkt->getSrcType() == Veh){
		    	//sendingBundleMsg();
		    	newSendingBundleMsg(netwPkt);
		    }
	    }
	}
}

void GeoDtnICNetwLayer::newSendingBundleMsg(GeoDtnNetwPkt *netwPkt)
{
	// VERY IMPORTANT MUST RECHECK MECHANISM FOR CUSTODY AND WHETEVER TO SEND, sTO CUSTODY OR NOT

	bool forwardingDecision = makeForwardingDecision(netwPkt->getSrcDist_NP_VPA(), netwPkt->getSrcMETD());

	bool custodyDecision = makeCustodyDecision(netwPkt->getSrcDist_NP_VPA());

	// step 1 : Build bundle list to send before reordering
	if (forwardingDecision){
		// step 1 : Build bundle list to send before reordering
		std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
		for (std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.begin(); it != bundlesReplicaIndex.end(); it++){
			unsigned long serial = it->first;
			if (exist(serial)){
				for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
					if ((*it3)->getSerial() == serial){
						unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it->second));
						break;
					}
				}
			}
		}

		// step 2 : Reordering bundle list
		// step 3 : Filtering bundle to send

		std::vector<std::pair<WaveShortMessage*, int> > filtered_unsortedWSMPair;
		if (custodyDecision){
			filtered_unsortedWSMPair = unsortedWSMPair;
		}else{
			// Since there is some specials cases for GeoDtnICNetwLayer we perform an additional filtering in the current function
			filtered_unsortedWSMPair = specific_scheduleFilterBundles(unsortedWSMPair, netwPkt->getSrcAddr(), netwPkt->getSrcType());
		}

		// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
		std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(filtered_unsortedWSMPair, netwPkt->getSrcAddr(), netwPkt->getSrcType());

		// step 4 : Sending bundles
		for (std::vector<WaveShortMessage* >::iterator it = sentWSM.begin(); it != sentWSM.end(); it++){
			WaveShortMessage* wsm = *it;
			unsigned long serial = wsm->getSerial();
			GeoDtnNetwPkt* bundleMsg = new GeoDtnNetwPkt();
			prepareNetwPkt(bundleMsg, Bundle, netwPkt->getSrcAddr());
			bundleMsg->setCustodyTransfert(custodyDecision);
			bundleMsg->encapsulate(wsm->dup());
			sendDown(bundleMsg, 0, 0, 1);
			if ((custodyMode == No) || (custodyMode == Yes_WithoutACK)){
				bundlesReplicaIndex[serial]++;
				if (custodyDecision){
					erase(serial);
					if ((custodyList == Diffuse) || (custodyList == Diffuse_Delete)){
						unsigned long currentTSForSerial;
						double currentMETDForSerial = netwPkt->getSrcMETD();
						if ((currentMETDForSerial <= 0)||(currentMETDForSerial == maxDbl)) {
							currentTSForSerial = 0;
						}else{
							currentTSForSerial = (unsigned long) (ceil((currentMETDForSerial * majorationOfCustodyTimestamp + simTime().dbl())));
						}
						unsigned long serialWithTimestamp = multiFunctions::cantorPairingFunc(serial, currentTSForSerial);
						storeCustodySerial(serialWithTimestamp);
					}
				}
			}
		}
	}
}

void GeoDtnICNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
	for (std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.begin(); it != bundlesReplicaIndex.end(); it++){
		unsigned long serial = it->first;
		if (exist(serial)){
			for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
				if ((*it3)->getSerial() == serial){
					unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it->second));
					break;
				}
			}
		}
	}

	// step 2 : Reordering bundle list
	// step 3 : Filtering bundle to send

//	// Since there is some specials cases for GeoDtnICNetwLayer we perform an additional filtering in the current function
//	std::vector<std::pair<WaveShortMessage*, int> > filtered_unsortedWSMPair = specific_scheduleFilterBundles(unsortedWSMPair, vpaAddr, VPA);
//
//	// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
//	std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(filtered_unsortedWSMPair, vpaAddr, VPA);

	// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
	std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(unsortedWSMPair, vpaAddr, VPA);

	// step 4 : Sending bundles
	std::set<unsigned long > serialsToDelete;
	for (std::vector<WaveShortMessage* >::iterator it = sentWSM.begin(); it != sentWSM.end(); it++){
		WaveShortMessage* wsm = *it;
		unsigned long serial = wsm->getSerial();
		GeoDtnNetwPkt* bundleMsg = new GeoDtnNetwPkt();
		prepareNetwPkt(bundleMsg, Bundle, vpaAddr);
		bundleMsg->encapsulate(wsm->dup());
		sendDown(bundleMsg, 0, 0, 1);
		bundleSentPerVPA.insert(serial);
		if(!withExplicitE2EAck){
			serialsToDelete.insert(serial);
		}
	}

	if(!withExplicitE2EAck){
		storeAckSerials(serialsToDelete);
	}
}

void GeoDtnICNetwLayer::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);
		totalBundlesReceived++;

		std::set<unsigned long> finalReceivedWSM;
		
		std::set<unsigned long> receivedWSM;

		if (netwPkt->getCustodyTransfert() && (netwPkt->getFwdDist() == myNetwAddr)){
//			cout << "@" << myNetwAddr << " received custody transfer " << endl;
		}

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.insert(wsm->getSerial());
			bundlesReceived++;
			emit(receiveL3SignalId,bundlesReceived);
			storeAckSerial(wsm->getSerial());
//			cout << "(VPA) Node@: " << myNetwAddr << " NodeType: " << nodeType << "received WSM with serial: " << wsm->getSerial() << endl;
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			if ((!exist(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
				storeBundle(wsm);
				if (custodyMode == Yes_WithACK){
					receivedWSM.insert(wsm->getSerial());
				}
				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
				if (it == bundlesReplicaIndex.end()){
					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
					currentNbrIsrt++;
				}
				bundlesReceived++;
				emit(receiveL3SignalId,bundlesReceived);
			}
//			cout << "(Other) Node@: " << myNetwAddr << " NodeType: " << nodeType << "received WSM with serial: " << wsm->getSerial() << endl;
		}
//		if (!receivedWSM.empty() || !finalReceivedWSM.empty()){
//			sendingBundleAckMsg(netwPkt->getSrcAddr(), receivedWSM, finalReceivedWSM);
//		}
		if (!finalReceivedWSM.empty() && withExplicitE2EAck){
			sendingBundleE2EAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM);
		}

		if (!receivedWSM.empty()  && (custodyMode == Yes_WithACK)){
			sendingBundleH2HAckMsg(netwPkt->getSrcAddr(), receivedWSM, netwPkt->getCustodyTransfert());
		}
	}
}

void GeoDtnICNetwLayer::sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmFinalDeliverd)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	if (!withAddressedAck){
		prepareNetwPkt(netwPkt, Bundle_Ack, LAddress::L3BROADCAST);
	} else if (withAddressedAck){
		prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);
	}
	std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
	netwPkt->setE2eAcks(serialOfE2EAck);
	long otherControlBitLength = sizeof(unsigned long) * serialOfE2EAck.size() *8;
	int length = otherControlBitLength + netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}

void GeoDtnICNetwLayer::sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDeliverd, bool custodyTransfer)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	if (!withAddressedAck){
		prepareNetwPkt(netwPkt, Bundle_Ack, LAddress::L3BROADCAST);
	} else if (withAddressedAck){
		prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);
	}
	std::set<unsigned long> serialOfH2HAck = std::set<unsigned long>(wsmDeliverd);
	netwPkt->setH2hAcks(serialOfH2HAck);
	double myCurrentDist = getCurrentDist();
	double myCurrentMETD = getCurrentMETD();
	netwPkt->setSrcMETD(myCurrentMETD);
	netwPkt->setSrcDist_NP_VPA(myCurrentDist);
	netwPkt->setCustodyTransfert(custodyTransfer);
	long otherControlBitLength = sizeof(unsigned long) * serialOfH2HAck.size() *8;
	int length = otherControlBitLength + netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}


void GeoDtnICNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	if (withExplicitE2EAck){
		std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
		storeAckSerials(finalDelivredToBndl);
	}

	if (custodyMode == Yes_WithACK){
		std::set<unsigned long> delivredToBndl = netwPkt->getH2hAcks();
		for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
			bundlesReplicaIndex[*it] = bundlesReplicaIndex[*it]++;
			if (netwPkt->getCustodyTransfert()){
				erase(*it);
				if ((custodyList == Diffuse) || (custodyList == Diffuse_Delete)){
					unsigned long currentTSForSerial;
					double currentMETDForSerial = netwPkt->getSrcMETD();
					if ((currentMETDForSerial <= 0)||(currentMETDForSerial == maxDbl)) {
						currentTSForSerial = 0;
					}else{
						currentTSForSerial = (unsigned long) (ceil((currentMETDForSerial * majorationOfCustodyTimestamp + simTime().dbl())));
					}
					unsigned long serialWithTimestamp = multiFunctions::cantorPairingFunc((*it), currentTSForSerial);
					storeCustodySerial(serialWithTimestamp);
				}
			}
		}
	}
}

////////////////////////////////////////// Others methods /////////////////////////

void GeoDtnICNetwLayer::recordStatsFwds(std::pair<LAddress::L3Type,double> fwdDist, std::pair<LAddress::L3Type,double> fwdMETD)
{
	if ((fwdDist.first == 0) && (fwdMETD.first == 0)){
		nbr0ValidFwds++;
	}else if ((fwdDist.first != 0) != (fwdMETD.first != 0)){
		if (fwdDist.first != 0){
			coreEV << "Node:" << myNetwAddr << " Unique valid Forwarders : (Dist): " << fwdDist.first << std::endl;
//			std::cout << "Node:" << myNetwAddr << " Unique valid Forwarders : (Dist): " << fwdDist.first << std::endl;
		}
		if (fwdMETD.first != 0){
			coreEV << "Node:" << myNetwAddr << " Unique valid Forwarders : (METD): " << fwdMETD.first << std::endl;
//			std::cout << "Node:" << myNetwAddr << " Unique valid Forwarders : (METD): " << fwdMETD.first << std::endl;
		}
		nbr1ValidFwds++;
	}else {
		if (fwdDist.first != fwdMETD.first){
			coreEV << "Node:" << myNetwAddr << " Distinct Forwarders : (Dist: " << fwdDist.first << " , METD: " << fwdMETD.first << ")"<< std::endl;
//			std::cout << "Node:" << myNetwAddr << " Distinct Forwarders : (Dist: " << fwdDist.first << " , METD: " << fwdMETD.first << ")"<< std::endl;
			nbr2Fwds++;
		}else {
			coreEV << "Node:" << myNetwAddr << " Same Forwarders for both Metrics: " << fwdDist.first << std::endl;
//			std::cout << "Node:" << myNetwAddr << " Same Forwarders for both Metrics: " << fwdDist.first << std::endl;
			nbr1Fwds++;
		}
	}
}

void GeoDtnICNetwLayer::storeAckSerial(unsigned long  serial)
{
	if (ackSerial.count(serial) == 0){
		DtnNetwLayer::storeAckSerial(serial);
    	currentNbrIsrt++;
	}
}

void GeoDtnICNetwLayer::storeAckSerials(std::set<unsigned long > setOfSerials)
{
	DtnNetwLayer::storeAckSerials(setOfSerials);
	for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
		custodySerial.erase(*it);
	}
}

void GeoDtnICNetwLayer::storeCustodySerial(unsigned long  serial)
{
	std::pair<unsigned long, unsigned long > serialWithTimestamp = multiFunctions::inverseCantorPairingFunc(serial);
	unsigned long currentSerial = serialWithTimestamp.first;
	unsigned long currentTimestamp = serialWithTimestamp.second;
	if (withTTLForCtrl){
		if (custodySerial.count(currentSerial) == 0){
			custodySerial.insert(currentSerial);
	    	currentNbrIsrt++;
	    	std::map<unsigned long, double >::iterator it = custodySerialTimeStamp.find(currentSerial);
	    	if (it == custodySerialTimeStamp.end()){
	    		custodySerialTimeStamp.insert(std::pair<unsigned long, double>(currentSerial,(double)(currentTimestamp)));
	    	}else{
	    		// If new timestamp longer than previous we update it otherwise we do nothing
	    		if (it->second < currentTimestamp){
	    			custodySerialTimeStamp[currentSerial] = currentTimestamp;
	    		}
	    	}
		}
	}else{
		if (custodySerial.count(currentSerial) == 0){
			custodySerial.insert(currentSerial);
	    	currentNbrIsrt++;
		}
	}
}

void GeoDtnICNetwLayer::storeCustodySerials(std::set<unsigned long > setOfSerials)
{
	if (nodeType == Veh){
	    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
	    	unsigned long serial = (*it);
	    	std::pair<unsigned long, unsigned long > myCustodySerialWithTimeStamp = multiFunctions::inverseCantorPairingFunc(serial);
	    	unsigned long myCustodySerial = myCustodySerialWithTimeStamp.first;
	    	unsigned long myCustodyTimestamp = myCustodySerialWithTimeStamp.second;
	    	storeCustodySerial(serial);
	    	double currentMETD = maxDbl;
	    	if (withMETDFwd){
	    		currentMETD = geoTraci->getCurrentNp().getMetd();
	    	}
	    	if (currentMETD != 0){
	    		erase(myCustodySerial);
	    	}
	    }
	}
}

void GeoDtnICNetwLayer::updateNeighborhoodTable(LAddress::L3Type neighbor, NetwRoute neighborEntry)
{
	double currentTime = simTime().dbl();
	std::set<LAddress::L3Type> entriesToDelete;
	std::set<LAddress::L3Type> entriesToPend;

	// Check entries to delete or to set as Pending
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		if ((netwRouteExpirency > 0) && (!it->second.isStatus()) && ((currentTime - it->second.getTimestamp().dbl()) >= netwRouteExpirency)){
			// if entry is currently pending and has expired we delete it
			if (it->first != neighbor){
				entriesToDelete.insert(it->second.getDestAddr());
			}
		}

		if ((netwRoutePending > 0) && (it->second.isStatus()) && ((currentTime - it->second.getTimestamp().dbl()) >= netwRoutePending)){
			// if entry is currently active and has not been update since an amount of time =>>> set it as pending
			if (it->first != neighbor){
				entriesToPend.insert(it->second.getDestAddr());
			}
		}
	}

	// Update table entries (Deleting/Pending)
	for (std::set<LAddress::L3Type>::iterator it = entriesToDelete.begin(); it != entriesToDelete.end(); it++){
		std::map<LAddress::L3Type, NetwRoute>::iterator it2 = neighborhoodTable.find(*it);
		if (it2 != neighborhoodTable.end()){
			emitInRadioWithVPA(it2->first, it2->second.getNodeType(), -1);
		}

		neighborhoodTable.erase(*it);
		neighborhoodSession.erase(*it);
		NBHTableNbrDelete++;
	}

	for (std::set<LAddress::L3Type>::iterator it = entriesToPend.begin(); it != entriesToPend.end(); it++){
		std::map<LAddress::L3Type, NetwRoute>::iterator it2 = neighborhoodTable.find(*it);
		if (it2 == neighborhoodTable.end()){
			opp_error("NeighboorhoodTable entry not found");
		}else{
			it2->second.setStatus(false);
		}
		if ((nodeType == Veh) && (it2 != neighborhoodTable.end()) && (it2->second.getNodeType() == VPA) ){
			if (!vpaContactDuration.empty()){
				double lastContactTime = vpaContactDuration.back();
				if (lastContactTime <= 0){
					double contactDuration = simTime().dbl() - (-lastContactTime);
					vpaContactDuration.pop_back();
					vpaContactDuration.push_back(contactDuration);
				}else{
					opp_error("Error value is negative");
				}
			}
		}
	}

	// Adding the new entry
	std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.find(neighbor);
	if (it == neighborhoodTable.end()){
		// neighbor doesn't exist, add a new entry
		neighborhoodTable.insert(std::pair<LAddress::L3Type, NetwRoute>(neighbor, neighborEntry));
		NBHTableNbrInsert++;
		if (neighborEntry.getNodeType() == VPA){
			meetVPA = true;
		}
	}else{
		// neighbor exists, update the old entry
		neighborhoodTable[neighbor] = neighborEntry;
	}

	emitInRadioWithVPA(neighbor, neighborEntry.getNodeType(), 1);
	nbrNeighors += neighborhoodTable.size()-1;
	nbrCountForMeanNeighbors++;

}

GeoTraCIMobility *GeoDtnICNetwLayer::getGeoTraci()
{
	GeoTraCIMobility* geo;
	switch (nodeType) {
		case Veh:
			geo = GeoTraCIMobilityAccess().get(getParentModule());
			break;
		default:
			opp_error("GeoDtnICNetwLayer::getGeoTraci() - Unable to retrieve GeoTraCIMobility because node is not of type Veh");
			break;
	}
	return geo;
}


bool GeoDtnICNetwLayer::checkBeforeHelloMechanism()
{
	// Function that return bool for CheckBeforeHEllo (CBH) Mechanism

	if (withCBH){
		// if CBH is activated check nbr of insert and return corresponding value
		if (currentNbrIsrt != lastNbrIsrt){
			// check if we have new data before sending ACK+Bundle list in Hello msg
			lastNbrIsrt = currentNbrIsrt;
			return true;
		}else {
			return false;
		}
	}else{
		// if CBH is not activated return always true
		return true;
	}
}

void GeoDtnICNetwLayer::emitInRadioWithVPA(LAddress::L3Type neighbor, int neighborNodeType, int flagValue)
{
	std::stringstream ss1,ss2;
	if ((nodeType == Veh) && (neighborNodeType == VPA) ){
		ss1 << neighbor;
		ss2 << flagValue;
		std::string str = ss1.str()+":"+ss2.str();
		emit(inRadioWithVPA,str.c_str());
		if (flagValue == 1){
			vpaContactDuration.push_back(-simTime().dbl());
		}
	}
}

void GeoDtnICNetwLayer::updateInRadioWithVPA(short kind, int neighborNodeType){

	bool isInRadioWithVPA = false;

	// Detect if the current node is in radio with a vpa
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		if ((nodeType == Veh)&&(it->second.getNodeType() == VPA)&&(it->second.isStatus())){
			isInRadioWithVPA = true;
			break;
		}
	}
	// Updating stats if also in radio with other vehicles
	if ((nodeType == Veh) && isInRadioWithVPA && (neighborNodeType == Veh)){
		switch (kind) {
			case HELLO:
				receivedHWICVPA++;
				break;
			case Bundle:
				receivedBWICVPA++;
				break;
			case Bundle_Ack:
				receivedAWICVPA++;
				break;
			default:
				break;
		}
	}
}

double GeoDtnICNetwLayer::getCurrentMETD()
{
	double currentMETD;
	if (nodeType == Veh){
		currentMETD = getGeoTraci()->getCurrentMetd();
	}else if (nodeType == VPA){
		currentMETD = 0.0;
	}else {
		opp_error("Undefined NodeType");
	}
	return currentMETD;
}

std::vector<std::pair<WaveShortMessage*,int> > GeoDtnICNetwLayer::specific_scheduleFilterBundles(std::vector<std::pair<WaveShortMessage*,int> > unsortedWSMPair, LAddress::L3Type destAddr, int destType)
{
	std::vector<std::pair<WaveShortMessage*, int> > filtered_unsortedWSMPair;
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = unsortedWSMPair.begin(); it != unsortedWSMPair.end(); it++){
		WaveShortMessage* wsm = it->first;
		if (custodySerial.count(wsm->getSerial()) > 0) {continue;}
		filtered_unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>(it->first,it->second));
	}
	return filtered_unsortedWSMPair;
}

double GeoDtnICNetwLayer::getCurrentDist()
{
	double currentDist;
	if (nodeType == Veh){
		currentDist = getGeoTraci()->getCurrentNp().getDistanceNpVpa();
	}else if (nodeType == VPA){
		currentDist = 0.0;
	}else {
		opp_error("Undefined NodeType");
	}
	return currentDist;
}

bool GeoDtnICNetwLayer::makeForwardingDecision(double srcDist, double srcMETD)
{
	bool decisionBasedOnMETD = false, decisionBasedOnDist = false;
	bool decision = false;

	if (withMETDFwd){
		if (getCurrentMETD() > srcMETD){
			decisionBasedOnMETD = true;
		}
	}

	if (withDistFwd){
		if (getCurrentDist() > srcDist){
			decisionBasedOnDist = true;
		}else if (getCurrentDist() == srcDist){
			decisionBasedOnDist = decisionBasedOnMETD;
		}
	}

	if (withDistFwd && withMETDFwd){
		if (multiMetricFwdStrat == AtLeastOne){
			decision = decisionBasedOnMETD || decisionBasedOnDist;
		}else if (multiMetricFwdStrat == Both){
			decision = decisionBasedOnMETD && decisionBasedOnDist;
		}
	} else if (withDistFwd){
		decision = decisionBasedOnDist;
	} else if (withMETDFwd){
		decision = decisionBasedOnMETD;
	}

	if (decision == false){
		Fwd_No++;
	}else {
		if (decisionBasedOnDist && decisionBasedOnMETD){
			Fwd_Yes_Both++;
		}else if (decisionBasedOnDist){
			Fwd_Yes_Dist++;
		}else if (decisionBasedOnMETD){
			Fwd_Yes_METD++;
		}
	}
	return (decision);
}



bool GeoDtnICNetwLayer::makeCustodyDecision(double srcDist)
{
	bool decision = false;
	if ((withDistFwd) && ((custodyMode == Yes_WithoutACK) || (custodyMode == Yes_WithACK))){
		if ((getCurrentDist() > srcDist) && (srcDist == 0)){
			decision = true;
		}
	}
	return decision;
}

void GeoDtnICNetwLayer::deletedCustodySerials()
{
	std::set<unsigned long> entriesToDelete;
	double currentTime = simTime().dbl();

	//std::cout << "Total Size of CustodySerials: " << custodySerial.size() << '\n';

	for (std::map<unsigned long, double>::iterator it = custodySerialTimeStamp.begin(); it != custodySerialTimeStamp.end(); it++){
		if ((*it).second < currentTime){
		    //std::cout << (*it).first << " => " << (*it).second << '\n';
		    nbrCtrlDeletedWithTTL++;
		    custodySerial.erase((*it).first);
		    entriesToDelete.insert((*it).first);
		}
	}

	//std::cout << "Total Size of CustodySerials: " << custodySerial.size() << '\n';

	for(std::set<unsigned long>::iterator it = entriesToDelete.begin(); it != entriesToDelete.end(); it++){
		custodySerialTimeStamp.erase((*it));
	}
}

std::set<unsigned long> GeoDtnICNetwLayer::buildCustodySerialWithTimeStamp(){
	std::set<unsigned long> myCustodySerialsWithTimeStamp;

	for(std::set<unsigned long>::iterator it = custodySerial.begin(); it != custodySerial.end(); it++){
		std::map<unsigned long, double>::iterator it2 = custodySerialTimeStamp.find((*it));
		if (it2 != custodySerialTimeStamp.end()){
			unsigned long myCustodySerial = it2->first;
			unsigned long myCustodyTimestamp = (unsigned long) (ceil(it2->second));
			unsigned long myCustodyWithTimestamp = multiFunctions::cantorPairingFunc(myCustodySerial, myCustodyTimestamp);
			myCustodySerialsWithTimeStamp.insert(myCustodyWithTimestamp);
		}else{
			opp_error("GeoDtnICNetwLayer::buildCustodySerialWithTimeStamp Unable to find CustodySerial in custodySerialTimeStamp");
		}
	}

	return myCustodySerialsWithTimeStamp;
}
