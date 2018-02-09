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

#include "ProphetNCV2.h"

Define_Module(ProphetNCV2);

void ProphetNCV2::initialize(int stage)
{
	DtnNetwLayer::initialize(stage);
	if (stage==0){
		/*
		 * Initialization of ProphetV2 parameters
		 */
		PEncMax = par("PEncMax").doubleValue();
		PFirstContact = par("PFirstContact").doubleValue();
		PMinThreshold = par("PMinThreshold").doubleValue();
		I_TYP = par("I_TYP").doubleValue();
		Beta = par("Beta").doubleValue();
		GAMMA = par("GAMMA").doubleValue();
		secondsInTimeUnit = par("secondsInTimeUnit");
		/*
		 * Initialization of the forward strategy &
		 * the queuing strategy
		 */
		int fwd = par("fwdStrategy");
		fwdStrategy = static_cast<t_prophet_forward>(fwd);
		int q = par("qStrategy");
		qStrategy = static_cast<t_prophet_queuing>(q);
		/*
		 * Initialization of the used structures
		 */
		lastAgeUpdate = 0;

		I_Preds = par("I_Preds").doubleValue();

		/*
		 * Collecting data & metrics
		 */

	    nbrPredsVector.setName("Number of predictions");
	    predsMean.setName("Mean of predictions");
	    predsMax.setName("Maximum of predictions");
	    predsMin.setName("Minimum of predictions");
	    predsVariance.setName("Variance of predictions");
	    predForVPA.setName("Pred for vpa");

	    nbrSuccessfulContact = 0;
        nbrFailedContactBeforeRIB = 0;
        nbrFailedContactAtRIB= 0;
        nbrFailedContactAtBundle_Offer = 0;
        nbrFailedContactAtBundle_Response = 0;
        contactState = std::map<LAddress::L3Type, Prophetv2MessageKinds>();

        initAllClassifier();

        maxForRC = 5;

        nbrContactsForRCVect.setName("Evolution of nbrContact between RC");

        withPartialUpdate = par("withPartialUpdate").boolValue();

        withEMethod = par("withEMethod").boolValue();

        withPredLength = par("withPredLength").boolValue();

	}
	else if (stage==1){
		preds.insert(std::pair<LAddress::L3Type,double>(myNetwAddr,1));
	}
}

/*******************************************************************
**
** 							Core methods
**
********************************************************************/

void ProphetNCV2::handleLowerMsg(cMessage* msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    ProphetNCPkt *netwPkt = check_and_cast<ProphetNCPkt *>(m->decapsulate());

    coreEV << "Receiving ProphetNCPkt packet from " << netwPkt->getSrcAddr() << " Addressed to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

   	if (isEquiped){
		switch (netwPkt->getKind()) {
			case HELLO:
				handleHelloMsg(netwPkt);
				break;
			case Bundle_Offer:
				if (netwPkt->getDestAddr() == myNetwAddr){
					handleBundleOfferMsg(netwPkt);
				}
				break;
			case Bundle_Response:
				if (netwPkt->getDestAddr() == myNetwAddr){
					handleBundleResponseMsg(netwPkt);
				}
				break;
			case Bundle:
				if (netwPkt->getDestAddr() == myNetwAddr){
					handleBundleMsg(netwPkt);
				}
				break;
			case Bundle_Ack:
				if (netwPkt->getDestAddr() == myNetwAddr){
					handleBundleAckMsg(netwPkt);
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

void ProphetNCV2::handleSelfMsg(cMessage* msg)
{
	if (msg == heartBeatMsg){
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,maxDbl,maxDbl, simTime(), true, nodeType, getCurrentPos()));
		sendingHelloMsg();
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void ProphetNCV2::finish()
{
	recordScalar("# failed contacts before RIB", nbrFailedContactBeforeRIB);
	recordScalar("# failed contacts at RIB", nbrFailedContactAtRIB);
	recordScalar("# failed contacts at Bundle_Offer", nbrFailedContactAtBundle_Offer);
	recordScalar("# failed contacts at Bundle_Response", nbrFailedContactAtBundle_Response);
	recordScalar("# successful contacts", nbrSuccessfulContact);

	recordAllScalars();
	classifyAll();
	recordAllClassifier();

//	while (!bundles.empty()){
//		delete bundles.front();
//		bundles.pop_front();
//	}
}

void ProphetNCV2::sendingHelloMsg()
{
	ProphetNCPkt* netwPkt = new ProphetNCPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
	ageDeliveryPreds();
	std::map<LAddress::L3Type, double> predToSend = std::map<LAddress::L3Type, double>(preds);
	netwPkt->setPreds(predToSend);
	long helloControlBitLength = 0;
	if (withPredLength){
		int predsLength = (sizeof(int ) + sizeof(double)) * predToSend.size();
		long sizeHC_CL_Octets = predsLength;
		emitSignalForHelloCtrlMsg(0, 0, sizeHC_CL_Octets, 0);
		helloControlBitLength = (predsLength) *8;
		int length = helloControlBitLength + netwPkt->getBitLength();
		netwPkt->setBitLength(length);
	}
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt,helloControlBitLength, 0, 0);
}

void ProphetNCV2::handleHelloMsg(ProphetNCPkt *netwPkt)
{
	// If not the same sector ignore message
	if (netwPkt->getVpaSectorId() != sectorId){
		return;
	}else{
		/*************************** Handling Hello Msg **********/
	    NetwRoute neighborEntry = NetwRoute(netwPkt->getSrcAddr(), maxDbl, maxDbl, simTime() , true, netwPkt->getSrcType(), netwPkt->getCurrentPos());
	    updateNeighborhoodTable(netwPkt->getSrcAddr(), neighborEntry);
	    update(netwPkt);
	    /*************************** Sending BundleOffer Msg **********/
	    sendingBndlOfferMsg(netwPkt->getSrcAddr(),netwPkt->getPreds());
	}
}

void ProphetNCV2::sendingBndlOfferMsg(LAddress::L3Type nodeAddr, std::map<LAddress::L3Type, double> predsOfNode)
{
//	/*************************** E2E Acks **********/
	/***************** Cleaning AckSerials from old entries *****/
	if (withTTLForCtrl){
		deleteAckSerials();
	}
	/***************** Cleaning AckSerials from old entries *****/
	std::set<unsigned long> storedAck = std::set<unsigned long>(ackSerial);

//	/*************************** H2H Acks (stored bundles) **********/
	std::set<unsigned long > storedBundle;

	std::list<unsigned long > myBundles = bundleStocker.getBundleSerials();
	for (std::list<unsigned long>::iterator it = myBundles.begin(); it != myBundles.end(); it++){
		unsigned long wsmSerial = (*it);
		LAddress::L3Type destAddr = bundleStocker.getBundleBySerial(wsmSerial)->getRecipientAddress();
		if (destAddr == myNetwAddr){
			opp_error("ProphetNCV2::sendingBndlOfferMsg --- storing bundles addressed to current node");
		}

		predsIterator myPred = preds.find(destAddr);
		predsIterator otherPred = predsOfNode.find(destAddr);
		bool addToOffer = false;
		if (otherPred != predsOfNode.end()){
			if (myPred != preds.end()){
				if (myPred->second < otherPred->second){
					addToOffer = true;
				}
			}else{
				addToOffer = true;
			}
		}
		if (addToOffer){
			storedBundle.insert(wsmSerial);
		}
	}

//	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
//		LAddress::L3Type destAddr = (*it)->getRecipientAddress();
//		if (destAddr == myNetwAddr){
//			opp_error("storing bundles addressed to current node");
//		}
//		predsIterator myPred = preds.find((*it)->getRecipientAddress());
//		predsIterator otherPred = predsOfNode.find((*it)->getRecipientAddress());
//		bool addToOffer = false;
//		if (otherPred != predsOfNode.end()){
//			if (myPred != preds.end()){
//				if (myPred->second < otherPred->second){
//					addToOffer = true;
//				}
//			}else{
//				addToOffer = true;
//			}
//		}
//		if (addToOffer){
//			unsigned long wsmSerial = (*it)->getSerial();
//			storedBundle.insert(wsmSerial);
//		}
//	}

	if (!(storedBundle.empty() & storedAck.empty())){
		// if at least one of the two sets is not empty
		ProphetNCPkt *netwPkt = new ProphetNCPkt();
		prepareNetwPkt(netwPkt, Bundle_Offer, nodeAddr);
		netwPkt->setE2eAcks(storedAck);
		netwPkt->setH2hAcks(storedBundle);
		int nbrEntries = storedAck.size()+ storedBundle.size();
		long sizeOC_SB_Octets = sizeof(unsigned long) * storedBundle.size();
		long sizeOC_SA_Octets = sizeof(unsigned long) * storedAck.size();
		emitSignalForOtherCtrlMsg(sizeOC_SB_Octets, sizeOC_SA_Octets, 0, 0);
		long otherControlBitLength = sizeof(unsigned long) * nbrEntries *8;
		int length = otherControlBitLength + netwPkt->getBitLength();
		netwPkt->setBitLength(length);
		//cout << "Sending BundleOffer packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
		sendDown(netwPkt, 0, otherControlBitLength, 0);
	}
}

void ProphetNCV2::handleBundleOfferMsg(ProphetNCPkt *netwPkt)
{
	/*************************** E2E Acks **********/
    std::set<unsigned long> receivedE2eAcks = netwPkt->getE2eAcks();
    if (!receivedE2eAcks.empty()){
    	updateStoredAcksForSession(netwPkt->getSrcAddr(), receivedE2eAcks);
    	storeAckSerials(receivedE2eAcks);
    }

    /*************************** H2H Acks (stored bundles) **********/
	std::set<unsigned long> serialStoredBndl;
	std::set<unsigned long> serialResponseBndl;

	serialStoredBndl = netwPkt->getH2hAcks();
    if (!serialStoredBndl.empty()){
    	updateStoredBndlForSession(netwPkt->getSrcAddr(), serialStoredBndl);
    }

	for (std::set<unsigned long>::iterator it = serialStoredBndl.begin(); it != serialStoredBndl.end(); it++){
		unsigned long serial = *it;
//		if ((ackSerial.count(serial) >= 1) || (exist(serial))){
		if ((ackSerial.count(serial) >= 1) || (bundleStocker.existBundle(serial))){
			continue;
		}else{
			serialResponseBndl.insert(serial);
		}
	}

	if (!serialResponseBndl.empty()){
		sendingBundleResponseMsg(netwPkt->getSrcAddr(), serialResponseBndl);
	}
}

void ProphetNCV2::sendingBundleResponseMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmResponseBndl)
{
	ProphetNCPkt *netwPkt = new ProphetNCPkt();
	prepareNetwPkt(netwPkt, Bundle_Response, destAddr);
	netwPkt->setH2hAcks(wsmResponseBndl);
	long sizeOC_SB_Octets = sizeof(unsigned long) * wsmResponseBndl.size();
	emitSignalForOtherCtrlMsg(sizeOC_SB_Octets, 0, 0, 0);
	long otherControlBitLength = sizeof(unsigned long) * wsmResponseBndl.size() *8;
	int length = otherControlBitLength + netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}

void ProphetNCV2::handleBundleResponseMsg(ProphetNCPkt *netwPkt)
{
	std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();

	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
//	for (std::set<unsigned long>::iterator it = serialResponseBndl.begin(); it != serialResponseBndl.end(); it++){
//		unsigned long serial = *it;
//		if (exist(serial)){
//			std::map<unsigned long, int>::iterator it2 = bundlesReplicaIndex.find(serial);
//			if (it2 == bundlesReplicaIndex.end()){
//				opp_error("Bundle Found but not in rmg replica index");
//			}else{
//				for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
//					if ((*it3)->getSerial() == serial){
//						unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it2->second));
//						break;
//					}
//				}
//			}
//		}
//	}
	unsortedWSMPair = bundleStocker.getStoredBundlesWithReplica(serialResponseBndl);

	// step 2 : Reordering bundle list
	// step 3 : Filtering bundle to send

	// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
	std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(unsortedWSMPair, netwPkt->getSrcAddr(), netwPkt->getSrcType());

	// step 4 : Sending bundles
	sendingBundleMsg(netwPkt->getSrcAddr(),netwPkt->getSrcType(),sentWSM);
}

void ProphetNCV2::sendingBundleMsg(LAddress::L3Type destAddr, int destType, std::vector<WaveShortMessage* >  wsmToSend)
{
	for (std::vector<WaveShortMessage* >::iterator it = wsmToSend.begin(); it != wsmToSend.end(); it++){
		WaveShortMessage* wsm = *it;
		unsigned long serial = wsm->getSerial();
		ProphetNCPkt* bundleMsg = new ProphetNCPkt();
		prepareNetwPkt(bundleMsg, Bundle, destAddr);
		bundleMsg->encapsulate(wsm->dup());
		sendDown(bundleMsg, 0, 0, 1);
		emit(sentL3SignalId,1);
		if (destType == Veh){
//			bundlesReplicaIndex[serial]++;
			bundleStocker.updateSentReplica(serial);
		}
	}
}

void ProphetNCV2::handleBundleMsg(ProphetNCPkt *netwPkt)
{
	//	cout << "Receiving Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);
		totalBundlesReceived++;

		std::set<unsigned long> finalReceivedWSM;

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.insert(wsm->getSerial());
			bundlesReceived++;
			emit(receiveL3SignalId,bundlesReceived);
			storeAckSerial(wsm->getSerial());
			if (withTTLForCtrl){
				emitSignalForAckLifeTime(wsm->getSerial(), simTime().dbl(), ttlForCtrl+simTime().dbl());
			}
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
//			if ((!exist(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
			if ((!bundleStocker.existBundle(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
				bundleStocker.storeBundle(wsm);
//				storeBundle(wsm);
//				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
//				if (it == bundlesReplicaIndex.end()){
//					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
//				}
				bundlesReceived++;
				emit(receiveL3SignalId,bundlesReceived);
			}
		}
		if (!finalReceivedWSM.empty()){
			sendingBundleAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM);
		}
	}
}

void ProphetNCV2::sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long > wsmFinalDeliverd)
{
	ProphetNCPkt* netwPkt = new ProphetNCPkt();
	prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);
	std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
	long sizeOC_SA_Octets = sizeof(unsigned long) * serialOfE2EAck.size();
	emitSignalForOtherCtrlMsg(0, sizeOC_SA_Octets, 0, 0);
	long otherControlBitLength = sizeof(unsigned long) * serialOfE2EAck.size() *8;
	int length = otherControlBitLength + netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}

void ProphetNCV2::handleBundleAckMsg(ProphetNCPkt *netwPkt)
{
	std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
	updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
	storeAckSerials(finalDelivredToBndl);
	if (withTTLForCtrl){
		for (std::set<unsigned long>::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
			emitSignalForAckLifeTime((*it), -1, ttlForCtrl+simTime().dbl());
		}
	}
}

/*******************************************************************
**
** 							Methods for collecting  datas & stats
**
********************************************************************/

void ProphetNCV2::recordPredsStats()
{
	double min = DBL_MAX, max = DBL_MIN, mean = 0, sum = 0, variance = 0, varianceSum = 0;

	for (predsIterator it=preds.begin(); it!=preds.end();it++){
		if (it->first == myNetwAddr){
			continue;
		}

		// Minimum value
		if (it->second < min){
			min = it->second;
		}

		// Maximum value
		if (it->second > max){
			max = it->second;
		}

		// Summing values
		sum+=it->second;
	}

	predsIterator it2 = preds.find(myNetwAddr);
	if (it2 == preds.end()){
		nbrPredsVector.record(preds.size());
		if (preds.size() == 0){
			// Calculating mean
			mean = 0;

			// Calculating variance
			variance = 0;
		}else{
			// Calculating mean
			mean = sum / double (preds.size());

			// Calculating variance
			for (predsIterator it=preds.begin(); it!=preds.end();it++){
				if (it->first != myNetwAddr){
					varianceSum += pow((it->second - mean),2);
				}
			}
			variance = varianceSum / double (preds.size());
		}
	}else{
		if (preds.size() == 0 ){
			nbrPredsVector.record(preds.size());
			// Calculating mean
			mean = 0;

			// Calculating variance
			variance = 0;
		}else{
			nbrPredsVector.record(preds.size()-1);
			// Calculating mean
			mean = sum / double (preds.size()-1);

			// Calculating variance
			for (predsIterator it=preds.begin(); it!=preds.end();it++){
				if (it->first != myNetwAddr){
					varianceSum += pow((it->second - mean),2);
				}
			}
			variance = varianceSum / double (preds.size()-1);
		}
	}

	// recording values
	predsMin.record(min);
	predsMax.record(max);
	predsMean.record(mean);
	predsVariance.record(variance);

	std::map<LAddress::L3Type, int >::iterator it;
	std::map<LAddress::L3Type, cOutVector* >::iterator it3;

	for (it = nbrRepeatedContact.begin(); it != nbrRepeatedContact.end(); it++){
		if (it->second >= maxForRC){
			it3 = predsForRC.find(it->first);
			stringstream flux1;
			std::string tmpStr;
			cOutVector* tmp;
			if (it3 == predsForRC.end()){
				flux1 << it2->first;
				tmpStr = "Evolution of Preds for RC of @"+ flux1.str();
				tmp = new cOutVector(tmpStr.c_str());
			}else{
				tmp = it3->second;
			}
			it2 = preds.find(it->first);
			if (it2 == preds.end()){
				tmp->record(0);
			}else{
				tmp->record(it2->second);
			}
			predsForRC[it->first] = tmp;
		}
	}
}

void ProphetNCV2::recordEndContactStats(LAddress::L3Type addr, double time)
{
	double duration = time - contacts.find(addr)->second;;
	sumOfContactDur+=duration;
	contactDurVector.record(sumOfContactDur/ double (nbrContacts));

	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		switch (it->second) {
		case RIB:
			nbrFailedContactAtRIB++;
			break;
		case Bundle_Offer:
			nbrFailedContactAtBundle_Offer++;
			break;
		case Bundle_Response:
			nbrFailedContactAtBundle_Response++;
			break;
		case Bundle:
			nbrSuccessfulContact++;
			break;
		default:
			nbrFailedContactBeforeRIB++;
			break;
		}

		contacts.erase(addr);
		contactState.erase(addr);

		endContactTime[addr] = time;

		std::map<LAddress::L3Type, int>::iterator it2 = nbrRepeatedContact.find(addr);
		int nbrReContact;
		if (it2 != nbrRepeatedContact.end()){
			nbrReContact = it2->second;
			std::map<int, std::list<double> >::iterator it3 = contactDurForRC.find(nbrReContact);
			std::list<double> durationList;
			if (it3 != contactDurForRC.end()){
				durationList = it3->second;
			}
			durationList.push_back(duration);
			contactDurForRC[nbrReContact] = durationList;

			std::map<LAddress::L3Type, std::list<double> >::iterator it4 = contactDurByAddr.find(addr);
			std::list<double> durationListByAddr;
			if (it4 != contactDurByAddr.end()){
				durationListByAddr = it4->second;
			}
			durationListByAddr.push_back(duration);
			contactDurByAddr[addr] = durationListByAddr;
		}

		it2 = nbrContactsForRC.find(addr);
		int nbrContactSince = 0;
		if (it2 == nbrContactsForRC.end()){
			nbrContactSince = nbrContacts - 0;
		}else{
			nbrContactSince = nbrContacts - nbrContactsForRC[addr];
		}
		nbrContactsForRC[addr] = nbrContactSince;
		nbrContactsForRCVect.record(nbrContactSince);
	}
}

void ProphetNCV2::recordRecontactStats(LAddress::L3Type addr, double time)
{
	std::map<LAddress::L3Type, double>::iterator it = endContactTime.find(addr);
	double duration = 0;
	if (it != endContactTime.end()){
		// updating nbr repeated contacts nodes
		nbrRecontacts++;
		// updating vector stats for recontacted nodes
		duration = time - it->second;

		sumOfInterContactDur+=duration;
		intercontactDurVector.record(sumOfInterContactDur/ double (nbrRecontacts));

		std::map<LAddress::L3Type, int>::iterator it2 = nbrRepeatedContact.find(addr);

		int nbrReContact;
		if (it2 == nbrRepeatedContact.end()){
			nbrReContact = 1;
		}else{
			nbrReContact = it2->second;
			nbrReContact++;
		}
		nbrRepeatedContact[addr] = nbrReContact;

		std::map<int, std::list<double> >::iterator it3 = interContactDurForRC.find(nbrReContact);
		std::list<double> durationList;
		if (it3 != interContactDurForRC.end()){
			durationList = it3->second;
		}
		durationList.push_back(duration);
		interContactDurForRC[nbrReContact] = durationList;

		std::map<LAddress::L3Type, std::list<double> >::iterator it4 = interContactDurByAddr.find(addr);
		std::list<double> durationListByAddr;
		if (it4 != interContactDurByAddr.end()){
			durationListByAddr = it4->second;
		}
		durationListByAddr.push_back(duration);
		interContactDurByAddr[addr] = durationListByAddr;

		it2 = nbrContactsForRC.find(addr);
		int nbrContactSince = 0;
		if (it2 == nbrContactsForRC.end()){
			nbrContactSince = nbrContacts - 0;
		}else{
			nbrContactSince = nbrContacts - nbrContactsForRC[addr];
		}
		nbrContactsForRC[addr] = nbrContactSince;
		nbrContactsForRCVect.record(nbrContactSince);
	}
}

void ProphetNCV2::updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind)
{
	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		contactState.erase(addr);
	}
	contactState.insert(std::pair<LAddress::L3Type, Prophetv2MessageKinds>(addr,kind));
}

void ProphetNCV2::classify(SimpleContactStats* newContact)
{
	DtnNetwLayer::classify(newContact);
	if (!newContact->isSuccessfulContact()){
		switch (newContact->getState()) {
			case RIB:
				if (withFailRIB){
					FailRIB.update(newContact);
				}
				break;
			case Bundle_Offer:
				if (withFailBndlOffer){
					FailBndlOffer.update(newContact);
				}
				break;
			case Bundle_Response:
				if (withFailBndlResp){
					FailBndlResp.update(newContact);
				}
				break;
			case Bundle:
				if (withFailBndl){
					FailBndl.update(newContact);
				}
				break;
			case Bundle_Ack:
				if ((withAck)&&(withFailBndlAck)){
					FailBndlAck.update(newContact);
				}
				break;
			default:
				break;
		}
	}
}

void ProphetNCV2::recordAllClassifier()
{
	DtnNetwLayer::recordAllClassifier();

	if (withFailRIB){
		recordClassifier(FailRIB);
	}
	if (withFailBndlOffer){
		recordClassifier(FailBndlOffer);
	}
	if (withFailBndlResp){
		recordClassifier(FailBndlResp);
	}
	if (withFailBndl){
		recordClassifier(FailBndl);
	}
	if ((withFailBndlAck)&&(withAck)){
		recordClassifier(FailBndlAck);
	}
}

void ProphetNCV2::initAllClassifier()
{
	DtnNetwLayer::initAllClassifier();

	withFailRIB = par("withFailRIBClassifier");
	withCDFForFailRIB = par("CDFForFailRIBClassifier");
	if (withFailRIB){
	    FailRIB = ClassifiedContactStats("RIBFail",false,withCDFForFailRIB);
	}


	withFailBndlOffer = par("withFailBndlOfferClassifier");
	withCDFForFailBndlOffer = par("CDFForFailBndlOfferClassifier");
	if (withFailBndlOffer){
		FailBndlOffer = ClassifiedContactStats("BndlOfferFail",false,withCDFForFailBndlOffer);
	}


	withFailBndlResp = par("withFailBndlRespClassifier");
	withCDFForFailBndlResp = par("CDFForFailBndlRespClassifier");
	if (withFailBndlResp){
		FailBndlResp = ClassifiedContactStats("BndlRespFail",false,withCDFForFailBndlResp);
	}


	withFailBndl = par("withFailBndlClassifier");
	withCDFForFailBndl = par("CDFForFailBndlClassifier");
	if (withFailBndl){
		FailBndl = ClassifiedContactStats("BndlFail",false,withCDFForFailBndl);
	}


	withFailBndlAck = par("withFailBndlAckClassifier");
	withCDFForFailBndlAck = par("CDFForFailBndlAckClassifier");
	if ((withFailBndlAck)&&(withAck)){
		FailBndlAck = ClassifiedContactStats("BndlAckFail",false,withCDFForFailBndlAck);
	}
}

/*******************************************************************
**
** 							Methods related to predictions
**
********************************************************************/

void ProphetNCV2::updateDeliveryPredsFor(const LAddress::L3Type BAdress)
{
	if (BAdress == myNetwAddr){
		opp_warning("Cannot Update Delivery Predictions for its own Address(ProphetNCV2::updateDeliveryPredsFor)");
	}else{
		double PEnc,lastEncTime, predsForB;
			/*
			 * before calculating predsForB, we must age preds.
			 */
			ageDeliveryPreds();
			double encTime = simTime().dbl();
			predsIterator it= preds.find(BAdress);
			predsIterator it2= lastEncouterTime.find(BAdress);

			if (!withEMethod){
				if (it2==lastEncouterTime.end()){
					/*
					 * if iterator is equal map.end(), it means that this node had not been meet yet
					 * so lastEncTime equal 0 and therefore P equals PFirstContact
					 */
					predsForB = PFirstContact;
				}else {
					/*
					 * if iterator is not equal map.end(), it means that this node had been meet before
					 */
					if (it==preds.end()){
						predsForB = 0;
					}else {
						predsForB = it->second;
					}
					lastEncTime = it2->second;

					if (simTime().dbl()-lastEncTime<I_TYP){
						/*
						 * if the node has been encountered recently then don't use PEncMax
						 */
						PEnc = PEncMax * (( simTime().dbl()-lastEncTime)/I_TYP);
					}else {
						/*
						 * if the node has been encountered long ago then use PEncMax
						 */
						PEnc = PEncMax;
					}
					predsForB = predsForB + (1-predsForB)*PEnc;
				}
			}else{
				if (it==preds.end()){
					predsForB = PFirstContact;
					if (it2==lastEncouterTime.end()){
						lastEncTime = 0;
					}else {
						lastEncTime = it2->second;
					}
				}else {
					predsForB = it->second;
					if (it2==lastEncouterTime.end()){
						lastEncTime = 0;
					}else {
						lastEncTime = it2->second;
					}

					if (simTime().dbl()-lastEncTime<I_TYP){
						PEnc = PEncMax * (( simTime().dbl()-lastEncTime)/I_TYP);
					}else {
						PEnc = PEncMax;
					}
					predsForB = predsForB + (1-predsForB)*PEnc;
				}
			}

			preds[BAdress] = predsForB;
			if (BAdress == 10){
				predForVPA.record(predsForB);
			}
			lastEncouterTime[BAdress] = encTime;
			//cout << "@: " << myNetwAddr << " encountered@: " <<  BAdress << " T: " << encTime << " Pred: " << predsForB << endl;
	}
}

void ProphetNCV2::updateTransitivePreds(const LAddress::L3Type BAdress, std::map<LAddress::L3Type,double> Bpreds)
{

	double predsForB,predsForC,BpredsForC;
	LAddress::L3Type CAdress;
	predsIterator tmp_it;
	/*
	 * before calculating transitive predictions, we must age preds.
	 */
	ageDeliveryPreds();
	for (predsIterator it=Bpreds.begin(); it!=Bpreds.end();it++){

		if ((it->first==myNetwAddr)||(it->first==BAdress)){
			continue;
		}

		CAdress = it->first;
		tmp_it= preds.find(CAdress);
		if (tmp_it == preds.end()){
			predsForC = 0;
		} else {
			predsForC=tmp_it->second;
		}

		tmp_it= Bpreds.find(CAdress);
		BpredsForC=tmp_it->second;

		tmp_it= preds.find(BAdress);
		predsForB=tmp_it->second;

		if (predsForC<(predsForB*BpredsForC*Beta)){
			predsForC = predsForB*BpredsForC*Beta;
		}

		preds[CAdress] = predsForC;
		if (CAdress ==10){
			//cout << "@: " << myNetwAddr << " encountered@: " <<  BAdress << " T: " << simTime().dbl() << " Pred after aging: " << predsForB << "Pred for VPA:" << predsForC <<endl;
			predForVPA.record(predsForC);
		}
	}
}

void ProphetNCV2::ageDeliveryPreds()
{
	predsIterator it2;
	double time = simTime().dbl();
	int  timeDiff = int (time-lastAgeUpdate)/secondsInTimeUnit;
	if (timeDiff==0){
		return;
	}else {
		double mult = std::pow(GAMMA, timeDiff);
		std::vector<LAddress::L3Type> predsToDelete;

		for (predsIterator it=preds.begin();it!=preds.end();it++){

			if (it->first==myNetwAddr){
				continue;
			}

			it->second = it->second * mult;


			if (it->second < PMinThreshold){
				predsToDelete.push_back(it->first);
			}

			if (it->first ==10){
				predForVPA.record(it->second);
			}
		}

		for (std::vector<LAddress::L3Type>::iterator it2 = predsToDelete.begin(); it2 != predsToDelete.end(); it2++){
			LAddress::L3Type tmp = *it2;
			preds.erase(tmp);
		}

		lastAgeUpdate = simTime().dbl();
	}
}

void ProphetNCV2::update(ProphetNCPkt *prophetPkt)
{
	updateDeliveryPredsFor(prophetPkt->getSrcAddr());
	partialUpdate(prophetPkt);
}

void ProphetNCV2::partialUpdate(ProphetNCPkt *prophetPkt)
{
	updateTransitivePreds(prophetPkt->getSrcAddr(),prophetPkt->getPreds());
	recordPredsStats();
}

/*******************************************************************
**
** 							Unused functions
**
********************************************************************/

ProphetNCV2::~ProphetNCV2()
{

}
