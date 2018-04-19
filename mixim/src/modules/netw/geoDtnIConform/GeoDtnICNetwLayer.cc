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
		recomputeMyNetwRoute = false;
		withMETDFwd = par("withMETDFwd").boolValue();
		withDistFwd = par("withDistFwd").boolValue();
		if (!(withMETDFwd || withDistFwd)) {
			opp_error("No valid forwarder, please choose either one of them or both");
		}

		multiMetricFwdStrat = par("multiMetricFwdStrat");
		if ((multiMetricFwdStrat != AtLeastOne) && (multiMetricFwdStrat != Both)){
			opp_error("No valid Forwarding Strategy");
		}

		withExplicitE2EAck = par("withExplicitE2EAck").boolValue();

		withAddressedAck = par("withAddressedAck").boolValue();

		withCBH = par("withCBH").boolValue();

		initCustodyManagementOptions();

		currentNbrIsrt = 0;
		lastNbrIsrt = 0;

		Fwd_No  		= 0;
		Fwd_Yes_METD    = 0;
		Fwd_Yes_Dist    = 0;
		Fwd_Yes_Both    = 0;
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
				break;
			case Bundle:
				if (netwPkt->getDestAddr() == myNetwAddr){
					handleBundleMsg(netwPkt);
				}
				break;
			case Bundle_Ack:
				// if sent acks are addressed to me or they are broadcasted
				if ((withAddressedAck && (netwPkt->getDestAddr() == myNetwAddr)) || (!withAddressedAck)){
//				if (!withAddressedAck){
					handleBundleAckMsg(netwPkt);
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
		if (withTTLForCus && withTTLForCus){
			cusModule.deleteExpiredCustodys();
		}
		myNetwRoute = NetwRoute(myNetwAddr,getCurrentMETD(),getCurrentDist(), simTime(), true, nodeType, getCurrentPos());
		DtnNetwLayer::handleSelfMsg(msg);
	}
	if (msg == updateMsg){
		unsigned long nbrStoredCustody = cusModule.getNbrStoredCustodys();
		if (nbrStoredCustody != 0){
			nbrStoredCustodyVector.record(nbrStoredCustody);
		}
		DtnNetwLayer::handleSelfMsg(msg);
	}
}

void GeoDtnICNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	if (bndlModule.storeBundle(upper_msg)){
		currentNbrIsrt++;
	}
//	storeBundle(upper_msg);
//	std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(upper_msg->getSerial());
//	if (it == bundlesReplicaIndex.end()){
//		bundlesReplicaIndex.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), 0));
//		currentNbrIsrt++;
//	}
}

void GeoDtnICNetwLayer::finish()
{
	recordScalar("# No Forwarding", Fwd_No);
	recordScalar("# Yes Forwarding based on METD", Fwd_Yes_METD);
	recordScalar("# Yes Forwarding based on Dist", Fwd_Yes_Dist);
	recordScalar("# Yes Forwarding based on Both", Fwd_Yes_Both);

	recordScalar("# DeletedBundlesWithCustody", bndlModule.getNbrDeletedBundlesByCustody());

	if (withCustodyList){
		if (withAck){
			recordScalar("# DeletedCustodyWithAck", cusModule.getNbrDeletedCustByAck());
		}
		if (withTTLForCus){
			recordScalar("# DeletedCustodyWithTTL", cusModule.getNbrDeletedCustByTtl());
			recordScalar("# NbrMAJOfCustodyExpTime", cusModule.getNbrUpdatesForCustExpireTime());
		}
		recordScalar("# DeletedCustodyFIFO", cusModule.getNbrDeletedCustByFifo());
	}

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
	long helloControlBitLength = 0;
	if (checkBeforeHelloMechanism()){
//		std::set<unsigned long> storedAck = std::set<unsigned long>(ackSerial);
//		netwPkt->setE2eAcks(storedAck);
		std::map<unsigned long, double > ackSerialsWithExpTime = ackModule.getAckSerialsWithExpTime();
		netwPkt->setAckSerialsWithTimestamp(ackSerialsWithExpTime);
		std::set<unsigned long > storedBundle = bndlModule.getBundleSerialsAsSet();
		netwPkt->setH2hAcks(storedBundle);
//		std::set<unsigned long > storedBundle;
//		for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
//			storedBundle.insert((*it)->getSerial());
//		}
		std::map<unsigned long,double > custodyBundle;
		if (withDistFwd && (custodyMode != No) && (custodyList != No_Diffuse)){
			/***************** Cleaning CustodySerials from old entries *****/
			/***************** Cleaning CustodySerials from old entries *****/
			// If we use Dist metric, Custody mode is Yes and Custody list is > 0
//			custodyBundle = std::map<unsigned long,double >(custodySerial);
			updateNCustodySerial();
			custodyBundle = cusModule.getCustodySerialsWithExpTime();
			//custodyBundle = buildCustodySerialWithTimeStamp();
//			if (myCurrentDist == 0){
//				for(std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
//					unsigned long myCustodySerial = (*it);
//					double myCustodyTimestamp = 0;
//					if (  !	((myCurrentMETD <= 0)||(myCurrentMETD == maxDbl)) ) {
////						myCustodyTimestamp = myCurrentMETD * majorationOfCustodyTimestamp + simTime().dbl();
//						myCustodyTimestamp = myCurrentMETD * majorationOfTTLForCus + simTime().dbl();
//					}
//
//			    	std::map<unsigned long, double >::iterator it2 = custodyBundle.find(myCustodySerial);
//			    	if (it2 == custodyBundle.end()){
//			    		custodyBundle.insert(std::pair<unsigned long, double>(myCustodySerial,myCustodyTimestamp));
//			    	}else{
//			    		// If new timestamp longer than previous we update it otherwise we do nothing
//			    		if (it2->second < myCustodyTimestamp){
//			    			custodyBundle[myCustodySerial] = myCustodyTimestamp;
//			    		}
//			    	}
//				}
//			}
			netwPkt->setCustodySerialsWithTimestamp(custodyBundle);
		}
//		if (withTTLForCtrl){
//			nbrEntries = ackSerial.size()+ storedBundle.size()+ custodyBundle.size()*2;
//			long sizeHC_SB_Octets = sizeof(unsigned long) * storedBundle.size();
//			long sizeHC_SA_Octets = sizeof(unsigned long) * ackSerial.size();
//			long sizeHC_CL_Octets = sizeof(unsigned long) * custodyBundle.size() * 2;
//			emitSignalForHelloCtrlMsg(sizeHC_SB_Octets, sizeHC_SA_Octets, sizeHC_CL_Octets, 0);
//		}else{
//			nbrEntries = ackSerial.size()+ storedBundle.size()+ custodyBundle.size();
//			long sizeHC_SB_Octets = sizeof(unsigned long) * storedBundle.size();
//			long sizeHC_SA_Octets = sizeof(unsigned long) * ackSerial.size();
//			long sizeHC_CL_Octets = sizeof(unsigned long) * custodyBundle.size();
//			emitSignalForHelloCtrlMsg(sizeHC_SB_Octets, sizeHC_SA_Octets, sizeHC_CL_Octets, 0);
//		}

//		long sizeHC_SB_Octets = sizeof(unsigned long) * storedBundle.size();
//		long sizeHC_SA_Octets = 0, sizeHC_CL_Octets = 0;
//		if (withTTLForAck){
//			sizeHC_SA_Octets = (sizeof(unsigned long) + sizeof(double)) * ackSerialsWithExpTime.size();
//		}else{
//			sizeHC_SA_Octets = sizeof(unsigned long) * ackSerialsWithExpTime.size();
//		}
//
//		if (withTTLForCus){
//			sizeHC_CL_Octets = (sizeof(unsigned long) + sizeof(double)) * custodyBundle.size();
//		}else{
//			sizeHC_CL_Octets = sizeof(unsigned long) * custodyBundle.size();
//		}
//		emitSignalForHelloCtrlMsg(sizeHC_SB_Octets, sizeHC_SA_Octets, sizeHC_CL_Octets, 0);
//
//		helloControlBitLength = (sizeHC_SB_Octets + sizeHC_SA_Octets + sizeHC_CL_Octets) * 8;

		helloControlBitLength = estimateInBitsCtrlSize(true, &storedBundle, &ackSerialsWithExpTime, &custodyBundle, NULL);
		netwPkt->addBitLength(helloControlBitLength);


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
//	int length = helloControlBitLength+ netwPkt->getBitLength();
//	netwPkt->setBitLength(length);
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
//	    std::set<unsigned long> receivedE2eAcks = netwPkt->getE2eAcks();
//	    if (!receivedE2eAcks.empty()){
//	    	updateStoredAcksForSession(netwPkt->getSrcAddr(), receivedE2eAcks);
//	    	storeAckSerials(receivedE2eAcks);
////	    	for(std::set<unsigned long >::iterator it = receivedE2eAcks.begin(); it != receivedE2eAcks.end(); it++){
////	    		custodyAckSerial.erase(*it);
////	    	}
//	    }
		std::map<unsigned long, double > receivedAckSerials = netwPkt->getAckSerialsWithTimestamp();
		if (!receivedAckSerials.empty()){
			updateStoredAcksForSession(netwPkt->getSrcAddr(),receivedAckSerials);
			storeNAckSerial(receivedAckSerials);
		}
	    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
	    if (!storedBundle.empty()){
	    	updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
	    }
	    std::map<unsigned long, double > custodyBundle = netwPkt->getCustodySerialsWithTimestamp();
	    if (!custodyBundle.empty()){
			updateStoredCustodysForSession(netwPkt->getSrcAddr(),custodyBundle);
	    	storeNCustodySerial(custodyBundle);
	    }
//		if (withDistFwd && (custodyMode != No) && (!custodyBundle.empty()) && (custodyList != No_Diffuse)){
//	    	for(std::map<unsigned long, double >::iterator it = custodyBundle.begin(); it != custodyBundle.end(); it++){
//	    		storeCustodySerial(it->first, it->second);
//	    		if ((custodyList == Diffuse_Delete) && (getCurrentDist() != 0)){
////	    			erase(it->first);
//	    			bndlModule.deleteBundle(it->first);
//	    			//erase(*it);
//	    		}
//	    	}
//	    }
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
		    }else if (netwPkt->getSrcType() == Veh){
		    	sendingBundleMsg(netwPkt);
		    }
	    }
	}
}

void GeoDtnICNetwLayer::sendingBundleMsg(GeoDtnNetwPkt *netwPkt)
{
	// VERY IMPORTANT MUST RECHECK MECHANISM FOR CUSTODY AND WHETEVER TO SEND, sTO CUSTODY OR NOT

	bool forwardingDecision = makeForwardingDecision(netwPkt->getSrcDist_NP_VPA(), netwPkt->getSrcMETD());

	bool custodyDecision = makeCustodyDecision(netwPkt->getSrcDist_NP_VPA());

	// step 1 : Build bundle list to send before reordering
	if (forwardingDecision){
		// step 1 : Build bundle list to send before reordering
		std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
//		for (std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.begin(); it != bundlesReplicaIndex.end(); it++){
//			unsigned long serial = it->first;
//			if (exist(serial)){
//				for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
//					if ((*it3)->getSerial() == serial){
//						unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it->second));
//						break;
//					}
//				}
//			}
//		}
		unsortedWSMPair = bndlModule.getStoredBundlesWithReplica();

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
			emit(sentL3SignalId,1);
			if ((custodyMode == No) || (custodyMode == Yes_WithoutACK)){
				double currentMETDForSerial = netwPkt->getSrcMETD();
//				bundlesReplicaIndex[serial]++;
				bndlModule.updateSentReplica(serial);
				if (custodyDecision){
					//erase(serial);
					bndlModule.deleteBundleUponCustody(serial);
					gen1CustodySerial(serial, currentMETDForSerial);
//					if ((custodyList == Diffuse) || (custodyList == Diffuse_Delete)){
//						double currentTimestamp = 0;
//						double currentMETDForSerial = netwPkt->getSrcMETD();
//						if ( ! ((currentMETDForSerial <= 0)||(currentMETDForSerial == maxDbl)) ){
////							currentTimestamp = currentMETDForSerial * majorationOfCustodyTimestamp + simTime().dbl();
//							currentTimestamp = currentMETDForSerial * majorationOfTTLForCus + simTime().dbl();
//						}
//						storeCustodySerial(serial, currentTimestamp);
//					}
				}
			}
		}
	}
}

void GeoDtnICNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
//	for (std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.begin(); it != bundlesReplicaIndex.end(); it++){
//		unsigned long serial = it->first;
//		if (exist(serial)){
//			for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
//				if ((*it3)->getSerial() == serial){
//					unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it->second));
//					break;
//				}
//			}
//		}
//	}
	unsortedWSMPair = bndlModule.getStoredBundlesWithReplica();

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
		emit(sentL3SignalId,1);
		if(!withExplicitE2EAck){
//			serialsToDelete.insert(serial);
			gen1AckSerial(wsm);
		}
	}

//	if(!withExplicitE2EAck){
//		storeAckSerials(serialsToDelete);
//	}
}

void GeoDtnICNetwLayer::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);
		totalBundlesReceived++;

		if (withCtrlForSectorReAddr){
			wsm->setRecipientAddress(newSectorAddr);
		}

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
//			storeAckSerial(wsm->getSerial());
//			if (withTTLForCtrl){
//				emitSignalForAckLifeTime(wsm->getSerial(), simTime().dbl(), ttlForCtrl+simTime().dbl());
//			}
			gen1AckSerial(wsm);
//			cout << "(VPA) Node@: " << myNetwAddr << " NodeType: " << nodeType << "received WSM with serial: " << wsm->getSerial() << endl;
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
//			if ((!exist(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
//			if ((!bndlModule.existBundle(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
			if ((!bndlModule.existBundle(wsm->getSerial())) && (ackModule.existAck(wsm->getSerial()) == 0)){
				if(bndlModule.storeBundle(wsm)){
					currentNbrIsrt++;
				}
//				storeBundle(wsm);
//				if (custodyMode == Yes_WithACK){
//					receivedWSM.insert(wsm->getSerial());
//				}
//				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
//				if (it == bundlesReplicaIndex.end()){
//					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
//					currentNbrIsrt++;
//				}
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
//	std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
//	netwPkt->setE2eAcks(serialOfE2EAck);
//	long sizeOC_SA_Octets = sizeof(unsigned long) * serialOfE2EAck.size();

	std::map<unsigned long, double > ackSerialsWithExpTime = ackModule.getAckSerialsWithExpTime(wsmFinalDeliverd);
	netwPkt->setAckSerialsWithTimestamp(ackSerialsWithExpTime);

//	long sizeOC_SA_Octets = 0;
//	if (withTTLForAck){
//		sizeOC_SA_Octets = (sizeof(unsigned long) + sizeof(double)) * ackSerialsWithExpTime.size();
//	}else{
//		sizeOC_SA_Octets = (sizeof(unsigned long)) * ackSerialsWithExpTime.size();
//	}
//	emitSignalForOtherCtrlMsg(0, sizeOC_SA_Octets, 0, 0);
//
//	long otherControlBitLength = sizeOC_SA_Octets *8;
//	int length = otherControlBitLength + netwPkt->getBitLength();
//	netwPkt->setBitLength(length);
	long otherControlBitLength = estimateInBitsCtrlSize(false, NULL, &ackSerialsWithExpTime, NULL, NULL);
	netwPkt->addBitLength(otherControlBitLength);
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
//	long sizeOC_RCC_Octets = sizeof(unsigned long) * serialOfH2HAck.size();
//	emitSignalForOtherCtrlMsg(0, 0, 0, sizeOC_RCC_Octets);
//	long otherControlBitLength = sizeOC_RCC_Octets *8;
//	int length = otherControlBitLength + netwPkt->getBitLength();
//	netwPkt->setBitLength(length);
	long otherControlBitLength = estimateInBitsCtrlSize(false, NULL, NULL, NULL, &serialOfH2HAck);
	netwPkt->addBitLength(otherControlBitLength);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}


void GeoDtnICNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	if (withExplicitE2EAck){
//		std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
//		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
//		storeAckSerials(finalDelivredToBndl);
//		if (withTTLForCtrl){
//			for (std::set<unsigned long>::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
//				emitSignalForAckLifeTime((*it), -1, ttlForCtrl+simTime().dbl());
//			}
//		}
		std::map<unsigned long, double > finalDelivredToBndl = netwPkt->getAckSerialsWithTimestamp();
		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
		storeNAckSerial(finalDelivredToBndl);
	}

	if (custodyMode == Yes_WithACK){
		std::set<unsigned long> delivredToBndl = netwPkt->getH2hAcks();
		for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
			unsigned long serial = *it;
			double currentMETDForSerial = netwPkt->getSrcMETD();
//			bundlesReplicaIndex[*it] = bundlesReplicaIndex[*it]++;
			bndlModule.updateSentReplica(serial);
			if (netwPkt->getCustodyTransfert()){
//				erase(*it);
				bndlModule.deleteBundleUponCustody(serial);
				gen1CustodySerial(serial, currentMETDForSerial);
//				if ((custodyList == Diffuse) || (custodyList == Diffuse_Delete)){
//					double currentTimestamp = 0;
//					double currentMETDForSerial = netwPkt->getSrcMETD();
//					if ( ! ((currentMETDForSerial <= 0)||(currentMETDForSerial == maxDbl)) ){
////						currentTimestamp = currentMETDForSerial * majorationOfCustodyTimestamp + simTime().dbl();
//						currentTimestamp = currentMETDForSerial * majorationOfTTLForCus + simTime().dbl();
//					}
//					storeCustodySerial((*it),currentTimestamp);
//				}
			}
		}
	}
}

////////////////////////////////////////// Others methods /////////////////////////
bool GeoDtnICNetwLayer::store1AckSerial(unsigned long  serial, double expireTime)
{
	bool stored = ackModule.storeAck(serial,expireTime);

	if (stored){
		bndlModule.deleteBundleUponACK(serial);
		cusModule.deleteCustodyUponACK(serial);
	}

	return stored;
}

bool GeoDtnICNetwLayer::store1CustodySerial(unsigned long  serial, double expireTime, bool shouldDelete)
{
	bool stored = cusModule.storeCustody(serial,expireTime);

	if (shouldDelete){
		if (stored && (custodyList == Diffuse_Delete) && (getCurrentDist() != 0)){
			bndlModule.deleteBundleUponCustody(serial);
		}
	}

	return stored;
}

void GeoDtnICNetwLayer::storeNCustodySerial(std::map<unsigned long ,double> custodySerialsWithTimestamp)
{
	for (std::map<unsigned long, double >::iterator it = custodySerialsWithTimestamp.begin(); it != custodySerialsWithTimestamp.end(); it++){
		unsigned long serial = it->first;
		double expireTime = it->second;
		if (store1CustodySerial(serial,expireTime, true)){
			if (withTTLForCus){
				emitSignalForCustodyLifeTime(serial, -1, expireTime);
			}
		}
	}
}
//void GeoDtnICNetwLayer::storeAckSerial(unsigned long  serial)
//{
////	if (ackSerial.count(serial) == 0){
////		DtnNetwLayer::storeAckSerial(serial);
////    	currentNbrIsrt++;
////	}
//}

//void GeoDtnICNetwLayer::storeAckSerials(std::set<unsigned long > setOfSerials)
//{
////	DtnNetwLayer::storeAckSerials(setOfSerials);
////	for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
////		custodySerial.erase(*it);
////	}
//}

//void GeoDtnICNetwLayer::storeCustodySerial(unsigned long  serial, double timestamp)
//{
//	std::map<unsigned long, double >::iterator it = custodySerial.find(serial);
//	if (it == custodySerial.end()){
//		custodySerial.insert(std::pair<unsigned long, double>(serial,timestamp));
//		currentNbrIsrt++;
//	}else{
//		// If new timestamp longer than previous we update it otherwise we do nothing
//		if (it->second < timestamp){
//			custodySerial[serial] = timestamp;
//			currentNbrIsrt++;
//		}
//	}
//
////	if (withTTLForCtrl){
//////		if (custodySerial.count(serial) == 0){
//////			custodySerial.insert(serial);
//////	    	currentNbrIsrt++;
////	    	std::map<unsigned long, double >::iterator it = custodySerialTimeStamp.find(currentSerial);
////	    	if (it == custodySerialTimeStamp.end()){
////	    		custodySerialTimeStamp.insert(std::pair<unsigned long, double>(currentSerial,(double)(currentTimestamp)));
////	    	}else{
////	    		// If new timestamp longer than previous we update it otherwise we do nothing
////	    		if (it->second < currentTimestamp){
////	    			custodySerialTimeStamp[currentSerial] = currentTimestamp;
////	    		}
////	    	}
//////		}
////	}else{
////		if (custodySerial.count(serial) == 0){
////			custodySerial.insert(currentSerial);
////	    	currentNbrIsrt++;
////		}
////	}
//}

//void GeoDtnICNetwLayer::storeCustodySerials(std::map<unsigned long, double > setOfSerials)
//{
//	if (nodeType == Veh){
//	    for (std::map<unsigned long, double >::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
//	    	unsigned long serial = it->first;
//	    	double myCustodyTimestamp = it->second;
//	    	storeCustodySerial(serial, myCustodyTimestamp);
//	    	double currentMETD = maxDbl;
//	    	if (withMETDFwd){
//	    		currentMETD = geoTraci->getCurrentNp().getMetd();
//	    	}
//	    	if (currentMETD != 0){
////	    		erase(serial);
//	    		bndlModule.deleteBundle(serial);
//	    	}
//	    }
//	}
//}

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
//		if (custodySerial.count(wsm->getSerial()) > 0) {continue;}
		if (cusModule.existCustody(wsm->getSerial())) {continue;}
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

//void GeoDtnICNetwLayer::deletedCustodySerials()
//{
//	std::set<unsigned long> entriesToDelete;
//	double currentTime = simTime().dbl();
//
//	//std::cout << "Total Size of CustodySerials: " << custodySerial.size() << '\n';
//
//	for (std::map<unsigned long, double>::iterator it = custodySerial.begin(); it != custodySerial.end(); it++){
//		if ((*it).second < currentTime){
//		    //std::cout << (*it).first << " => " << (*it).second << '\n';
////		    nbrDeletedCtrlByTTL++;
//		    entriesToDelete.insert((*it).first);
//		}
//	}
//
//	//std::cout << "Total Size of CustodySerials: " << custodySerial.size() << '\n';
//
//	for(std::set<unsigned long>::iterator it = entriesToDelete.begin(); it != entriesToDelete.end(); it++){
//		custodySerial.erase((*it));
//	}
//}

std::set<unsigned long> GeoDtnICNetwLayer::buildCustodySerialWithTimeStamp(){
	std::set<unsigned long> myCustodySerialsWithTimeStamp;

//	for(std::set<unsigned long>::iterator it = custodySerial.begin(); it != custodySerial.end(); it++){
//		std::map<unsigned long, double>::iterator it2 = custodySerialTimeStamp.find((*it));
//		if (it2 != custodySerialTimeStamp.end()){
//			unsigned long myCustodySerial = it2->first;
//			unsigned long myCustodyTimestamp = (unsigned long) (ceil(it2->second));
//			unsigned long myCustodyWithTimestamp = multiFunctions::cantorPairingFunc(myCustodySerial, myCustodyTimestamp);
//			myCustodySerialsWithTimeStamp.insert(myCustodyWithTimestamp);
//		}else{
//			opp_error("GeoDtnICNetwLayer::buildCustodySerialWithTimeStamp Unable to find CustodySerial in custodySerialTimeStamp");
//		}
//	}

	return myCustodySerialsWithTimeStamp;
}

void GeoDtnICNetwLayer::emitSignalForCustodyLifeTime(unsigned long serial, double startTime, double endTime)
{
	string signalStr = lg2Str((long)(serial))+":"+dbl2Str(startTime)+":"+dbl2Str(endTime);
	emit(t_custodyLifeTime,signalStr.c_str());
}

//std::pair<unsigned long ,double> GeoDtnICNetwLayer::gen1CustodySerial(unsigned long  serial, double currentMETD)
//{
//	double currentTime = simTime().dbl();
//	double expireTime = maxDbl;
//
//	if (withDistFwd && ((custodyMode == Yes_WithoutACK) || (custodyMode == Yes_WithACK)) &&
//			((custodyList == Diffuse) || (custodyList == Diffuse_Delete))) {
//
//			if (withTTLForCus){
//				if (typeTTLForCus == Fixed_TTL){
//					expireTime = currentTime + ttlForCus;
//				}else if (typeTTLForCus == Adaptative_TTL){
//					if ( ! ((currentMETD <= 0)||(currentMETD == maxDbl)) ){
//						expireTime = currentTime + currentMETD * majorationOfTTLForCus;
//					}else{
//						expireTime = currentTime;
//					}
//				}
//
//				emitSignalForCustodyLifeTime(serial, currentTime, expireTime);
//			}
//
//			return std::pair<unsigned long, double>(serial, expireTime);
//	} else {
//		opp_error("GeoDtnICNetwLayer::gen1CustodySerial --- Calling function in inappropriate mode");
//	}
//}

void GeoDtnICNetwLayer::gen1CustodySerial(unsigned long  serial, double currentMETD)
{
	double currentTime = simTime().dbl();
	double expireTime = maxDbl;

	if (withTTLForCus){
		if (typeTTLForCus == Fixed_TTL){
			expireTime = currentTime + ttlForCus;
		}else if (typeTTLForCus == Adaptative_TTL){
			if ( ! ((currentMETD <= 0)||(currentMETD == maxDbl)) ){
				expireTime = currentTime + currentMETD * majorationOfTTLForCus;
			}else{
				expireTime = currentTime;
			}
		}
	}

	if (store1CustodySerial(serial, expireTime, false)){
		if (withTTLForCus){
			emitSignalForCustodyLifeTime(serial, currentTime, expireTime);
		}
	}
}

void GeoDtnICNetwLayer::updateNCustodySerial()
{
	if (getCurrentDist() == 0){
		std::set<unsigned long > storedBundle = bndlModule.getBundleSerialsAsSet();
		for(std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
			unsigned long myCustodySerial = (*it);
			gen1CustodySerial(myCustodySerial, getCurrentMETD());
//			if (  !	((myCurrentMETD <= 0)||(myCurrentMETD == maxDbl)) ) {
////						myCustodyTimestamp = myCurrentMETD * majorationOfCustodyTimestamp + simTime().dbl();
//				myCustodyTimestamp = myCurrentMETD * majorationOfTTLForCus + simTime().dbl();
//			}
//
//	    	std::map<unsigned long, double >::iterator it2 = custodyBundle.find(myCustodySerial);
//	    	if (it2 == custodyBundle.end()){
//	    		custodyBundle.insert(std::pair<unsigned long, double>(myCustodySerial,myCustodyTimestamp));
//	    	}else{
//	    		// If new timestamp longer than previous we update it otherwise we do nothing
//	    		if (it2->second < myCustodyTimestamp){
//	    			custodyBundle[myCustodySerial] = myCustodyTimestamp;
//	    		}
//	    	}
		}
	}
}

void GeoDtnICNetwLayer::initCustodyManagementOptions()
{
	custodyStructureSize = par("custodyStructureSize");
	if (custodyStructureSize<=0){
		opp_error("Size of the structure that store custodies can not be negative");
	}

	custodyList = par("custodyList");
	if ((custodyList != No_Diffuse) && (custodyList != Diffuse) && (custodyList != Diffuse_Delete)){
		opp_error("No valid CustodyList");
	}

	custodyMode = par("custodyMode");
	if ((custodyMode != No) && (custodyMode != Yes_WithoutACK) && (custodyMode != Yes_WithACK)){
		opp_error("No valid CustodyMode");
	}

	withCustodyList = false;
	if ( withDistFwd && ((custodyMode == Yes_WithoutACK) || (custodyMode == Yes_WithACK)) ){
		if ((custodyList == Diffuse) || (custodyList == Diffuse_Delete)){
			withCustodyList = true;
		}
	}

	withTTLForCus = par("withTTLForCus").boolValue();
	if (withTTLForCus){
		ttlForCus = par("ttlForCus").doubleValue();
	}else{
		ttlForCus = double(maxSimulationTime);
	}

	cusModule = CustStorageHelper(custodyStructureSize, withCustodyList, withTTLForCus);

	int ttlType = par("typeTTLForCus");
	typeTTLForCus = (TTLForCtrlType) ttlType;

	majorationOfTTLForCus = par("majorationOfTTLForCus").doubleValue();

	t_custodyLifeTime = registerSignal("custodyLifeTime");

	nbrStoredCustodyVector.setName("StoredCustody");
	nbrStoredCustodyVector.record(0);
}

void GeoDtnICNetwLayer::updateStoredCustodysForSession(LAddress::L3Type srcAddr, std::map<unsigned long ,double> custodysToStore)
{
	NetwSession currentSession;
	std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
	if (it2 == neighborhoodSession.end()){
		currentSession = NetwSession(srcAddr,0);
	}else{
		currentSession = it2->second;
	}
	currentSession.updateStoredCustody(custodysToStore);
	neighborhoodSession[srcAddr] = currentSession;
}




