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
#include "algorithm"

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

		nbr2Fwds = 0;
		nbr1Fwds = 0;
		nbr0ValidFwds = 0;
		nbr1ValidFwds = 0;

		totalBundlesReceived = 0;
		bndlSentToVPA = 0;
		totalBndlSentToVPA = 0;

		currentNbrIsrt = 0;
		lastNbrIsrt = 0;

		firstSentToVPA = false;

		bndlInterestVec.setName("Evolve of interesting bundle of neighborhood");

		missedOpprVec.setName("Evolve of missed opportunities");

		meetVPA = false;

		inRadioWithVPA = registerSignal("InContact");

		gDistFwd = 0;
		bDistFwd = 0;
		gMETDFwd = 0;
		bMETDFwd = 0;

	}
}

void GeoDtnICNetwLayer::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    GeoDtnNetwPkt *netwPkt = check_and_cast<GeoDtnNetwPkt *>(m->decapsulate());

    coreEV << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Addressed to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

    switch (netwPkt->getKind()) {
		case HELLO:
			handleHelloMsg(netwPkt);
			updateInRadioWithVPA(HELLO,netwPkt->getSrcType());
			break;
		case Bundle:
			if ((netwPkt->getDestAddr() == myNetwAddr) ||
			((netwPkt->getDestAddr() == LAddress::L3BROADCAST) && ((netwPkt->getFwdDist() == myNetwAddr)||(netwPkt->getFwdMETD() == myNetwAddr)))){
				handleBundleMsg(netwPkt);
				updateInRadioWithVPA(Bundle,netwPkt->getSrcType());
			}
			break;
		case Bundle_Ack:
			handleBundleAckMsg(netwPkt);
			updateInRadioWithVPA(Bundle_Ack,netwPkt->getSrcType());
			break;
		default:
			break;
	}

    updatingL3Received();

    delete netwPkt;
    delete m;
}

void GeoDtnICNetwLayer::handleSelfMsg(cMessage *msg)
{
	if (msg == heartBeatMsg){
		double currentMETD, currentDist;
		if (nodeType == Veh){
			currentMETD = getGeoTraci()->getCurrentMetd();
			currentDist = getGeoTraci()->getCurrentNp().getDistanceNpVpa();
		}else if (nodeType == VPA){
			currentMETD = 0.0;
			currentDist = 0.0;
		}else {
			opp_error("Undefined NodeType");
		}
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,currentMETD,currentDist, simTime(), true, nodeType, getCurrentPos()));
		sendingHelloMsg(currentDist, currentMETD, getCurrentPos());
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

	recordScalar("# Redundant Bundle at L3", (totalBundlesReceived- bundlesReceived));

	recordScalar("# Bndl Sent to VPA (total)", totalBndlSentToVPA);
	recordScalar("# Bndl Sent to VPA (first)", bndlSentToVPA);

	recordScalar("Bad choice of METD Fwd", bMETDFwd);
	recordScalar("Bad choice of Dist Fwd", bDistFwd);
	recordScalar("Good choice of METD Fwd", gMETDFwd);
	recordScalar("Good choice of Dist Fwd", gMETDFwd);

	recordAllScalars();
}

void GeoDtnICNetwLayer::sendingHelloMsg(double distance, double METD, Coord currentPos)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
	netwPkt->setSrcMETD(METD);
	netwPkt->setSrcDist_NP_VPA(distance);
	netwPkt->setCurrentPos(currentPos);
	int nbrEntries = 0;
	if (checkBeforeHelloMechanism()){
		netwPkt->setE2eAcks(ackSerial);
		std::set<unsigned long > storedBundle;
		for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
			storedBundle.insert((*it)->getSerial());
		}
		netwPkt->setH2hAcks(storedBundle);
		nbrEntries = ackSerial.size()+ storedBundle.size();
		if (custodyMode == Yes_WithACK){
			netwPkt->setCustodyAcks(custodyAckSerial);
			nbrEntries+= custodyAckSerial.size();
		}

	}
	int length = sizeof(unsigned long) * (nbrEntries)+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
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
	    }
	    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
	    if (!storedBundle.empty()){
	    	updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
	    }
	    if ((custodyMode == Yes_WithACK)||(custodyMode == Yes_WithoutACK)){
			/** if the encoutered node is going to pass by the VPA so no need to request theses bundles instead delete them */
			if(( netwPkt->getSrcDist_NP_VPA() == 0)){
				for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
					erase(*it);
				}
			}
	    }
	    if (custodyMode == Yes_WithACK){
		    std::set<unsigned long> custodyAcks = netwPkt->getCustodyAcks();
		    if (!custodyAcks.empty()){
		    	storeCustodyAckSerials(custodyAcks);
		    }
	    }
	    /*************************** Sending Bundle Msg **********/
		if (nodeType == Veh){
		    if (netwPkt->getSrcType() == VPA){
		    	sendingBundleMsgToVPA(netwPkt->getSrcAddr());
		    	vpaContactDistance.push_back(getCurrentPos().distance(netwPkt->getCurrentPos()));
		    }else if (netwPkt->getSrcType() == Veh){
		    	sendingBundleMsg();
		    }
	    }
	}
}

void GeoDtnICNetwLayer::sendingBundleMsg()
{
	// step 1 : define forwarders than bundle to forward to them
	std::pair<LAddress::L3Type, double> fwdDist = std::pair<LAddress::L3Type, double>(LAddress::L3NULL,maxDbl);
	std::pair<LAddress::L3Type, double> fwdMETD = std::pair<LAddress::L3Type, double>(LAddress::L3NULL,maxDbl);
	std::vector<std::pair<WaveShortMessage*, int> > bundleForFwdDist;
	std::vector<std::pair<WaveShortMessage*, int> > bundleForFwdMETD;
	std::vector<std::pair<WaveShortMessage*, int> > bundlesForBoth;
	if (withDistFwd){
          fwdDist = getBestFwdDist();
          bundleForFwdDist = bundleFor1Fwd(fwdDist.first);
	}
	if (withMETDFwd){
          fwdMETD = getBestFwdMETD();
          bundleForFwdMETD = bundleFor1Fwd(fwdMETD.first);
	}

	if (withDistFwd && withMETDFwd){
		// Must concatenate both bundles list
		bundlesForBoth.insert(bundlesForBoth.end(),bundleForFwdDist.begin(),bundleForFwdDist.end());
		bundlesForBoth.insert(bundlesForBoth.end(),bundleForFwdMETD.begin(),bundleForFwdMETD.end());
		std::sort(bundlesForBoth.begin(), bundlesForBoth.end(), comparatorRCAscObject);
		bundlesForBoth.erase(std::unique( bundlesForBoth.begin(), bundlesForBoth.end() ), bundlesForBoth.end());
	}else if (withDistFwd){
		bundlesForBoth.insert(bundlesForBoth.end(),bundleForFwdDist.begin(),bundleForFwdDist.end());
	}else if (withMETDFwd){
		bundlesForBoth.insert(bundlesForBoth.end(),bundleForFwdMETD.begin(),bundleForFwdMETD.end());
	}

	std::vector<WaveShortMessage*> bundleToSent;
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = bundlesForBoth.begin(); it != bundlesForBoth.end(); it++){
		WaveShortMessage* wsm = it->first;
		bundleToSent.push_back(wsm);
	}
	recordStatsFwds(fwdDist, fwdMETD);

	bool haveToSend = true;
	bool haveToCustody = false;
	if ((custodyMode == Yes_WithACK) || (custodyMode == Yes_WithoutACK)){
		if ((fwdDist.first == myNetwAddr) && (fwdDist.second == 0.0)){
			haveToSend = false;
//			cout << "@" << myNetwAddr << " Current Node will pass by the VPA, no need to forward until finding a better forwarder" << endl;
			if ((fwdMETD.first != myNetwAddr) && (fwdMETD.first != LAddress::L3NULL)){
				std::map<LAddress::L3Type, NetwRoute>::iterator itMETD = neighborhoodTable.find(fwdMETD.first);
				if (itMETD != neighborhoodTable.end()){
					std::map<LAddress::L3Type, NetwRoute>::iterator itDist = neighborhoodTable.find(myNetwAddr);
					if (itDist != neighborhoodTable.end()){
						if ((itMETD->second.getDestDist() == 0.0) && (itMETD->second.getDestMetd() <= itDist->second.getDestMetd())){
//							cout << "@" << fwdMETD.first << " Is a good forwarder, so forward even if both will pass by the VPA" << endl;
							haveToSend = true;
						}
					}else{
						opp_error("Dist Forwarder not found in Neighborhood Table");
					}
				}else{
					opp_error("METD Forwarder not found in Neighborhood Table");
				}
			}
		}
		if ((fwdDist.first != myNetwAddr) && (fwdDist.second == 0.0)){
			std::map<LAddress::L3Type, NetwRoute>::iterator itDist = neighborhoodTable.find(fwdDist.first);
			if (itDist != neighborhoodTable.end()) {
				if (itDist->second.getNodeType() != VPA){
//					cout << "@" << myNetwAddr << " must transfer custody of bundle to this node @" << fwdDist.first << endl;
					haveToCustody = true;
				}
			}else{
				opp_error("Dist Forwarder not found in Neighborhood Table");
			}
		}
	}


	// step 2 : send bundles
	if (haveToSend){
		for (std::vector<WaveShortMessage* >::iterator it = bundleToSent.begin(); it != bundleToSent.end(); it++){
			WaveShortMessage* wsm = *it;
			GeoDtnNetwPkt* bundleMsg = new GeoDtnNetwPkt();
			prepareNetwPkt(bundleMsg, Bundle, LAddress::L3BROADCAST);
			if (haveToCustody){
				bundleMsg->setCustodyTransfert(true);
			}
			if (withDistFwd){
				bundleMsg->setFwdDist(fwdDist.first);
			}
			if (withMETDFwd){
				bundleMsg->setFwdMETD(fwdMETD.first);
			}
			bundleMsg->encapsulate(wsm->dup());
//			cout << "Sending Bundle packet from " << bundleMsg->getSrcAddr() << " addressed to 2Fwds " << fwdDist.first << " & " << fwdMETD.first << " Bundle Serial " << wsm->getSerial() << std::endl;
			sendDown(bundleMsg);
			bundlesReplicaIndex[(*it)->getSerial()]++;
		}
	}
}

void GeoDtnICNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : check if we have any bundle that are addressed to @vpaAddr
//	std::vector<WaveShortMessage* > wsmToSend = bundleForVPA(vpaAddr);
	std::vector<WaveShortMessage* > wsmToSend = bundleForNode(vpaAddr);
	if (!firstSentToVPA){
		bndlSentToVPA+=wsmToSend.size();
		firstSentToVPA = true;
	}
	totalBndlSentToVPA+=wsmToSend.size();
//	cout << "Current@: " << myNetwAddr << " nbrToSent: " << wsmToSend.size() << " current time: " << simTime().dbl() << endl;


	// step 2 : send bundles
	for (std::vector<WaveShortMessage*>::iterator it = wsmToSend.begin(); it!= wsmToSend.end(); it++){
		WaveShortMessage* wsm = *it;
		GeoDtnNetwPkt* bundleForVPA = new GeoDtnNetwPkt();
		prepareNetwPkt(bundleForVPA, Bundle, vpaAddr);
		bundleForVPA->encapsulate(wsm->dup());
//		cout << "Sending Bundle packet from " << bundleForVPA->getSrcAddr() << " addressed to VPA " << bundleForVPA->getDestAddr() << std::endl;
		sendDown(bundleForVPA);
		bundlesReplicaIndex[(*it)->getSerial()]++;
		bundleSentPerVPA.insert(wsm->getSerial());
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
				if (netwPkt->getCustodyTransfert()){
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
		if (!receivedWSM.empty() || !finalReceivedWSM.empty()){
			sendingBundleAckMsg(netwPkt->getSrcAddr(), receivedWSM, finalReceivedWSM);
		}
	}
}

void GeoDtnICNetwLayer::sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDelivred, std::set<unsigned long> wsmFinalDeliverd)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Ack, LAddress::L3BROADCAST);
	std::set<unsigned long> serialOfH2HAck = std::set<unsigned long>(wsmDelivred);
	netwPkt->setH2hAcks(serialOfH2HAck);

	std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * serialOfE2EAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void GeoDtnICNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
	updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
	storeAckSerials(finalDelivredToBndl);

	std::set<unsigned long> delivredToBndl = netwPkt->getH2hAcks();
	updateStoredBndlForSession(netwPkt->getSrcAddr(), delivredToBndl);
	if (custodyMode == Yes_WithoutACK){
		for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
			erase(*it);
		}
	}else if (custodyMode == Yes_WithACK){
		storeAckSerials(delivredToBndl);
	}
}

////////////////////////////////////////// Others methods /////////////////////////

std::vector<WaveShortMessage*> GeoDtnICNetwLayer::bundleForNode(LAddress::L3Type node)
{
	std::vector<WaveShortMessage* > sentWSM;

	// step 1 : check if we have any bundle that are addressed to @fwdDist or @fwdMETD
	std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair;
	for (std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.begin();it != bundlesReplicaIndex.end(); it++){
		for (std::list<WaveShortMessage*>::iterator it2 = bundles.begin(); it2 != bundles.end(); it2++){
			if ((*it2)->getSerial() == it->first){
				sortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it2), it->second));
				break;
			}
		}
	}
	std::sort(sortedWSMPair.begin(), sortedWSMPair.end(), comparatorRCAscObject);


	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = sortedWSMPair.begin(); it != sortedWSMPair.end(); it++){
		WaveShortMessage* wsm = it->first;
		if (ackSerial.count(wsm->getSerial()) > 0) {continue;}
		if (custodyAckSerial.count(wsm->getSerial()) > 0) {continue;}
		std::map<LAddress::L3Type, NetwSession>::iterator itNode = neighborhoodSession.find(node);
		if ((itNode != neighborhoodSession.end())){
			NetwSession sessionFwdDist = itNode->second;
			if ((sessionFwdDist.getStoredBndl().count(wsm->getSerial()) > 0)){
				continue;
			}else if ((sessionFwdDist.getDelivredToBndl().count(wsm->getSerial()) > 0)){
				continue;
			}else if ((sessionFwdDist.getDelivredToVpaBndl().count(wsm->getSerial()) > 0)){
				continue;
			}
		}
		sentWSM.push_back(wsm);
	}

	return sentWSM;
}

std::vector<std::pair<WaveShortMessage*, int> > GeoDtnICNetwLayer::bundleFor1Fwd(LAddress::L3Type fwdNode)
{
	std::vector<WaveShortMessage* > sentWSM;
	std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair;
	if((fwdNode == myNetwAddr) || (fwdNode == LAddress::L3NULL)){
//		cout << "Forwarders are the same as me" << endl;
		return sortedWSMPair;
	}

	// step 1 : create a list of all Bundles
	for (std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.begin();it != bundlesReplicaIndex.end(); it++){
		for (std::list<WaveShortMessage*>::iterator it2 = bundles.begin(); it2 != bundles.end(); it2++){
			WaveShortMessage* wsm = (*it2);
			if (wsm->getSerial() == it->first){
				if (ackSerial.count(wsm->getSerial()) > 0) {continue;}
				if (custodyAckSerial.count(wsm->getSerial()) > 0) {continue;}
				std::map<LAddress::L3Type, NetwSession>::iterator itFwd = neighborhoodSession.find(fwdNode);
				if (itFwd != neighborhoodSession.end()){
					NetwSession sessionFwdDist = itFwd->second;
					if (sessionFwdDist.getStoredBndl().count(wsm->getSerial()) > 0) {
						break;
					}else if (sessionFwdDist.getDelivredToBndl().count(wsm->getSerial()) > 0) {
						break;
					}else if (sessionFwdDist.getDelivredToVpaBndl().count(wsm->getSerial()) > 0) {
						break;
					}
				}
				sortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it2), it->second));
				break;
			}
		}
	}
	std::sort(sortedWSMPair.begin(), sortedWSMPair.end(), comparatorRCAscObject);

	return sortedWSMPair;

}

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

void GeoDtnICNetwLayer::storeCustodyAckSerial(unsigned long  serial)
{
	if (custodyAckSerial.count(serial) == 0){
		custodyAckSerial.insert(serial);
    	currentNbrIsrt++;
	}
}

void GeoDtnICNetwLayer::storeCustodyAckSerials(std::set<unsigned long > setOfSerials)
{
	if (nodeType == Veh){
	    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
	    	unsigned long serial = (*it);
	    	storeCustodyAckSerial(serial);
	    	double currentMETD = maxDbl;
	    	if (withMETDFwd){
	    		currentMETD = geoTraci->getCurrentNp().getMetd();
	    	}
	    	if (currentMETD != 0){
	    		erase(serial);
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

std::pair<LAddress::L3Type, double> GeoDtnICNetwLayer::getBestFwdMETD()
{
	std::vector<double > bestValues;
	std::multimap<double, LAddress::L3Type >bestForwarders;
	LAddress::L3Type bestForwarder = LAddress::L3NULL;
	double bestMETD = maxDbl;
//	cout << "Current Node @: " << myNetwAddr << " METDs: " << endl;
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		NetwRoute entry = it->second;
//		cout << "@: " << it->first << " value " <<  entry.getDestMetd() << endl;
		if ((bestMETD != maxDbl)&&(entry.isStatus())&&(entry.getDestMetd() == bestMETD)){
			bestForwarders.insert(std::pair<double, LAddress::L3Type>(entry.getDestMetd(), entry.getDestAddr()));
		}
		if ((entry.isStatus())&&(entry.getDestMetd() < bestMETD)){
			bestMETD = entry.getDestMetd();
			bestForwarder = entry.getDestAddr();
			bestValues.push_back(bestMETD);
			bestForwarders.insert(std::pair<double, LAddress::L3Type>(bestMETD, bestForwarder));
		}
	}
	if (bestForwarder != LAddress::L3NULL){
		coreEV << "Node:" << myNetwAddr << " BestForwarder based on METD: " << bestForwarder << " CurrentMETD: " << bestMETD << std::endl;
//		std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.find(myNetwAddr);
//		if (it != neighborhoodTable.end()){
//			if ((myNetwAddr != bestForwarder) && (it->second.getDestMetd() == bestMETD)){
//				bMETDFwd++;
//				bestForwarder = myNetwAddr;
//			}else{
//				gMETDFwd++;
//			}
//		}else{
//			opp_error("No entry found for itself");
//		}
//		std::cout << "Node:" << myNetwAddr <<" BestForwarder based on METD: " << bestForwarder << " CurrentMETD: " << bestMETD << std::endl;
	}

	for(std::vector<double>::iterator it = bestValues.begin(); it != bestValues.end(); it++){
		double value = *it;
//		cout << "Value: " << value;
		std::pair<std::multimap<double, LAddress::L3Type >::iterator, std::multimap<double, LAddress::L3Type >::iterator >  pairIterator = bestForwarders.equal_range(value);
		for (std::multimap<double, LAddress::L3Type >::iterator it2 = pairIterator.first; it2 != pairIterator.second; it2++){
//			cout << " @:" << it2->second << " ";
		}
//		cout << endl;
	}
//	cout << "Chosen forwarder @: " << bestForwarder << " value: "<< bestMETD << endl;


	return std::pair<LAddress::L3Type, double>(bestForwarder, bestMETD);
}

std::pair<LAddress::L3Type, double> GeoDtnICNetwLayer::getBestFwdDist()
{
	std::vector<double > bestValues;
	std::multimap<double, LAddress::L3Type >bestForwarders;
	LAddress::L3Type bestForwarder = LAddress::L3NULL;
	double bestDist = maxDbl;
//	cout << "Current Node @: " << myNetwAddr << " Dists: " << endl;
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		NetwRoute entry = it->second;
//		cout << "@: " << it->first << " value " <<  entry.getDestDist() << endl;
		if ((bestDist != maxDbl)&&(entry.isStatus())&&(entry.getDestDist() == bestDist)){
			bestForwarders.insert(std::pair<double, LAddress::L3Type>(entry.getDestDist(), entry.getDestAddr()));
		}
		if ((entry.isStatus())&&(entry.getDestDist() < bestDist)){
			bestDist = entry.getDestDist();
			bestForwarder = entry.getDestAddr();
			bestValues.push_back(bestDist);
			bestForwarders.insert(std::pair<double, LAddress::L3Type>(bestDist, bestForwarder));
		}
	}
	if (bestForwarder != LAddress::L3NULL){
		coreEV << "Node:" << myNetwAddr << " BestForwarder based on Dist: " << bestForwarder << " CurrentDist: " << bestDist << std::endl;
		std::cout << "Node:" << myNetwAddr << " BestForwarder based on Dist: " << bestForwarder << " CurrentDist: " << bestDist << std::endl;
//		std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.find(myNetwAddr);
//		if (it != neighborhoodTable.end()){
//			if ((myNetwAddr != bestForwarder) && (it->second.getDestDist() == bestDist)){
//				bDistFwd++;
//				bestForwarder = myNetwAddr;
//			}else{
//				gDistFwd++;
//			}
//		}else{
//			opp_error("No entry found for itself");
//		}
	}

	for(std::vector<double>::iterator it = bestValues.begin(); it != bestValues.end(); it++){
		double value = *it;
		cout << "Value: " << value;
		std::pair<std::multimap<double, LAddress::L3Type >::iterator, std::multimap<double, LAddress::L3Type >::iterator >  pairIterator = bestForwarders.equal_range(value);
		for (std::multimap<double, LAddress::L3Type >::iterator it2 = pairIterator.first; it2 != pairIterator.second; it2++){
			cout << " @:" << it2->second << " ";
		}
		cout << endl;
	}
//	cout << "Chosen forwarder @: " << bestForwarder << " value: "<< bestDist << endl;

	return std::pair<LAddress::L3Type, double>(bestForwarder, bestDist);
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
