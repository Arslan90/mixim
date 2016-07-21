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

#include "GeoDtnNetwLayer.h"
#include "FindModule.h"
#include "VEHICLEpOpp.h"
#include "VPApOpp.h"
#include "algorithm"
#include "list"

Define_Module(GeoDtnNetwLayer);

void GeoDtnNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
	if (stage == 0){
		DefineNodeType();
		netwRouteExpirency = par("netwRouteExpirency").doubleValue();
		netwRoutePending = par("netwRoutePending").doubleValue();
		heartBeatMsgPeriod = par("heartBeatMsgPeriod").doubleValue();

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

		NBHTableNbrInsert    = 0;
		NBHTableNbrDelete    = 0;
		NBHAddressNbrInsert  = 0;
		NBHAddressNbrDelete  = 0;

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

	}else if (stage == 1){

	}else if (stage == 2){
		if (nodeType == Veh){
			geoTraci = GeoTraCIMobilityAccess().get(getParentModule());
			sectorId = geoTraci->getCurrentSector();
		}else{
			geoTraci = NULL;
			sectorId = this->getParentModule()->getIndex();
		}
		heartBeatMsg = new cMessage("heartBeatMsg");
		scheduleAt(simTime(), heartBeatMsg);
	}
}

void GeoDtnNetwLayer::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    GeoDtnNetwPkt *netwPkt = check_and_cast<GeoDtnNetwPkt *>(m->decapsulate());

    coreEV << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;
//    std::cout << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

    switch (netwPkt->getKind()) {
		case HELLO:
			handleHelloMsg(netwPkt);
			break;
		case Bundle:
			if ((netwPkt->getDestAddr() == myNetwAddr) ||
			((netwPkt->getDestAddr() == LAddress::L3BROADCAST) && ((netwPkt->getFwdDist() == myNetwAddr)||(netwPkt->getFwdMETD() == myNetwAddr)))){
				handleBundleMsg(netwPkt);
			}
			break;
		case Bundle_Ack:
			handleBundleAckMsg(netwPkt);
			break;
		default:
			break;
	}

    updatingL3Received();

    delete netwPkt;
    delete m;
}



void GeoDtnNetwLayer::handleSelfMsg(cMessage *msg)
{
	double currentMETD, currentDist;
	if (msg == heartBeatMsg){
		if (nodeType == Veh){
			currentMETD = geoTraci->getCurrentMetd();
			currentDist = geoTraci->getCurrentNp().getDistanceNpVpa();
		}else if (nodeType == VPA){
			currentMETD = 0.0;
			currentDist = 0.0;
		}else {
			opp_error("Undefined NodeType");
		}
		if (nodeType == Veh){
			sectorId = geoTraci->getCurrentSector();
		}
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,currentMETD,currentDist, simTime(), true, nodeType));
		GeoDtnNetwPkt* netwPkt;
		sendingHelloMsg(netwPkt, currentDist, currentMETD);
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void GeoDtnNetwLayer::handleUpperMsg(cMessage *msg)
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

void GeoDtnNetwLayer::finish()
{
	recordScalar("# insertOper Oracle", NBHAddressNbrInsert);
	recordScalar("# delOper Oracle", NBHAddressNbrDelete);
	recordScalar("# insertOper NBHTable", NBHTableNbrInsert);
	recordScalar("# delOper NBHTable", NBHTableNbrInsert);

	recordScalar("# Distinct Forwarders", nbr2Fwds);
	recordScalar("# Same Forwarders", nbr1Fwds);
	recordScalar("# Unique valid Forwarders", nbr1ValidFwds);
	recordScalar("# No valid Forwarders", nbr0ValidFwds);

	recordScalar("# Redundant Bundle at L3", (totalBundlesReceived- bundlesReceived));

	recordScalar("# Bndl Sent to VPA (total)", totalBndlSentToVPA);
	recordScalar("# Bndl Sent to VPA (first)", bndlSentToVPA);

	recordAllScalars();
}

void GeoDtnNetwLayer::sendingHelloMsg(GeoDtnNetwPkt *netwPkt, double distance, double METD)
{

	if (nodeType == Veh){
		sectorId = geoTraci->getCurrentSector();
		netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
	}else if (nodeType == VPA){
		netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
	}else {
		opp_error("Undefined NodeType");
	}
	netwPkt->setSrcMETD(METD);
	netwPkt->setSrcDist_NP_VPA(distance);
	bool newData = false;
	if (withCBH){
		// check if we have new data before sending ACK+Bundle list in Hello msg
		if (currentNbrIsrt != lastNbrIsrt){
			lastNbrIsrt = currentNbrIsrt;
			newData = true;
		}
	}else{
		newData = true;
	}

	if (newData){
		netwPkt->setE2eAcks(ackSerial);
		std::set<unsigned long > storedBundle;
		for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
			storedBundle.insert((*it)->getSerial());
		}
		netwPkt->setH2hAcks(storedBundle);
		int nbrEntries = ackSerial.size()+ storedBundle.size();
		int length = sizeof(unsigned long) * (nbrEntries)+ netwPkt->getBitLength();
		netwPkt->setBitLength(length);
//		cout << "Sending Hello packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << " length " << netwPkt->getBitLength() << " with New Data (Ack+Stored)" << " (Nbr Entries) " << nbrEntries << std::endl;
	}else{
//		cout << "Sending Hello packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << " length " << netwPkt->getBitLength() << std::endl;
	}

	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;


	sendDown(netwPkt);
}

void GeoDtnNetwLayer::handleHelloMsg(GeoDtnNetwPkt *netwPkt)
{
	// If not the same sector ignore message
	if (netwPkt->getVpaSectorId() != sectorId){
		return;
	}else{
		/*************************** Handling Hello Msg **********/
//	    cout << "Receiving Hello packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	    NetwRoute neighborEntry = NetwRoute(netwPkt->getSrcAddr(), netwPkt->getSrcMETD(), netwPkt->getSrcDist_NP_VPA(), simTime() , true, netwPkt->getSrcType());
	    updateNeighborhoodTable(netwPkt->getSrcAddr(), neighborEntry);
	    std::set<unsigned long> receivedE2eAcks = netwPkt->getE2eAcks();
	    if (!receivedE2eAcks.empty()){
	    	storeAckSerial(receivedE2eAcks);
	    }
	    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
	    if (!storedBundle.empty()){
	    	updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
	    }
		/** if the encoutered node is going to pass by the VPA so no need to request theses bundles instead delete them */
		if(( netwPkt->getSrcDist_NP_VPA() == 0)){
			for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
				erase(*it);
			}
		}

	    if (nodeType == VPA){
	    	return;
	    }else{
	    	/*************************** Sending Bundle Msg **********/

		    if (netwPkt->getSrcType() == VPA){
		    	sendingBundleMsgToVPA(netwPkt->getSrcAddr());
		    }else if (netwPkt->getSrcType() == Veh){
		    	sendingBundleMsg();
		    }
	    }
	}
}

void GeoDtnNetwLayer::sendingBundleMsg()
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
			cout << "@" << myNetwAddr << " Current Node will pass by the VPA, no need to forward until finding a better forwarder" << endl;
			if (fwdMETD.first != myNetwAddr){
				std::map<LAddress::L3Type, NetwRoute>::iterator itMETD = neighborhoodTable.find(fwdMETD.first);
				if (itMETD != neighborhoodTable.end()){
					std::map<LAddress::L3Type, NetwRoute>::iterator itDist = neighborhoodTable.find(myNetwAddr);
					if (itDist != neighborhoodTable.end()){
						if ((itMETD->second.getDestDist() == 0.0) && (itMETD->second.getDestMetd() <= itDist->second.getDestMetd())){
							cout << "@" << fwdMETD.first << " Is a good forwarder, so forward even if both will pass by the VPA" << endl;
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
			if ((itDist != neighborhoodTable.end()) && (itDist->second.getNodeType() != VPA)){
				cout << "@" << myNetwAddr << " must transfer custody of bundle to this node @" << fwdDist.first << endl;
				haveToCustody = true;
			}else{
				opp_error("METD Forwarder not found in Neighborhood Table");
			}
		}
	}


	// step 2 : send bundles
	if (haveToSend){
		for (std::vector<WaveShortMessage* >::iterator it = bundleToSent.begin(); it != bundleToSent.end(); it++){
			WaveShortMessage* wsm = *it;
			GeoDtnNetwPkt* bundleMsg;
			sectorId = geoTraci->getCurrentSector();
			bundleMsg = prepareNetwPkt(Bundle,myNetwAddr, nodeType, LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
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
	//		cout << "Sending Bundle packet from " << bundleMsg->getSrcAddr() << " addressed to 2Fwds " << fwdDist.first << " & " << fwdMETD.first << std::endl;
			sendDown(bundleMsg);
			bundlesReplicaIndex[(*it)->getSerial()]++;
		}
	}
}

void GeoDtnNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
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
		GeoDtnNetwPkt* bundleForVPA;
		sectorId = geoTraci->getCurrentSector();
		bundleForVPA = prepareNetwPkt(Bundle,myNetwAddr, nodeType, vpaAddr, sectorId ,LAddress::L3BROADCAST);
		bundleForVPA->encapsulate(wsm->dup());
//		cout << "Sending Bundle packet from " << bundleForVPA->getSrcAddr() << " addressed to VPA " << bundleForVPA->getDestAddr() << std::endl;
		sendDown(bundleForVPA);
		bundlesReplicaIndex[(*it)->getSerial()]++;
	}
}


void GeoDtnNetwLayer::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
{
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);
		totalBundlesReceived++;

		GeoDtnNetwPkt* bundleAckMsg;
		std::list<unsigned long> receivedWSM;
		std::list<unsigned long> finalReceivedWSM;

		bool receivedCustody = netwPkt->getCustodyTransfert();
		if ((receivedCustody) && (netwPkt->getFwdDist() == myNetwAddr)){
			cout << "@" << myNetwAddr << " received custody transfer " << endl;
		}

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.push_back(wsm->getSerial());
			bundlesReceived++;
			storeAckSerial(wsm->getSerial());
//			cout << "(VPA) Node@: " << myNetwAddr << " NodeType: " << nodeType << "received WSM with serial: " << wsm->getSerial() << endl;
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			if ((!exist(wsm)) && (ackSerial.count(wsm->getSerial()) == 0)){
				storeBundle(wsm);
				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
				if (it == bundlesReplicaIndex.end()){
					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
					currentNbrIsrt++;
				}
				bundlesReceived++;
			}
			if (receivedCustody){
				receivedWSM.push_back(wsm->getSerial());
			}
//			cout << "(Other) Node@: " << myNetwAddr << " NodeType: " << nodeType << "received WSM with serial: " << wsm->getSerial() << endl;
		}
		if (!receivedWSM.empty() || !finalReceivedWSM.empty()){
			bundleAckMsg = prepareNetwPkt(Bundle_Ack,myNetwAddr, nodeType, LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
			sendingBundleAckMsg(bundleAckMsg, receivedWSM, finalReceivedWSM);
		}
	}
}

void GeoDtnNetwLayer::sendingBundleAckMsg(GeoDtnNetwPkt *netwPkt, std::list<unsigned long> wsmDelivred, std::list<unsigned long> wsmFinalDeliverd)
{
	std::set<unsigned long> serialOfH2HAck;
	for (std::list<unsigned long >::iterator it = wsmDelivred.begin(); it != wsmDelivred.end(); it++){
		serialOfH2HAck.insert(*it);
	}
	netwPkt->setH2hAcks(serialOfH2HAck);

	std::set<unsigned long> serialOfE2EAck;
	for (std::list<unsigned long >::iterator it = wsmFinalDeliverd.begin(); it != wsmFinalDeliverd.end(); it++){
		serialOfE2EAck.insert(*it);
	}
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * (serialOfH2HAck.size()+ serialOfE2EAck.size())+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void GeoDtnNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> delivredToBndl = netwPkt->getH2hAcks();
	std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(netwPkt->getSrcAddr());
	if (it2 == neighborhoodSession.end()){
		NetwSession newSession = NetwSession(netwPkt->getSrcAddr(),0);
		for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
			newSession.insertInDelivredToBndl(*it);
		}
		neighborhoodSession.insert(std::pair<LAddress::L3Type, NetwSession>(netwPkt->getSrcAddr(), newSession));
	}else{
		NetwSession newSession = it2->second;
		for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
			newSession.insertInDelivredToBndl(*it);
		}
		neighborhoodSession[netwPkt->getSrcAddr()] = newSession;
	}
	for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
		erase(*it);
	}

	std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
	it2 = neighborhoodSession.find(netwPkt->getSrcAddr());
	if (it2 == neighborhoodSession.end()){
		NetwSession newSession = NetwSession(netwPkt->getSrcAddr(),0);
		for (std::set<unsigned long >::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
			newSession.insertInDelivredToVpaBndl(*it);
		}
		neighborhoodSession.insert(std::pair<LAddress::L3Type, NetwSession>(netwPkt->getSrcAddr(), newSession));
	}else{
		NetwSession newSession = it2->second;
		for (std::set<unsigned long >::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
			newSession.insertInDelivredToVpaBndl(*it);
		}
		neighborhoodSession[netwPkt->getSrcAddr()] = newSession;
	}

	for (std::set<unsigned long >::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
		storeAckSerial(*it);
		erase(*it);
	}
}

GeoDtnNetwPkt *GeoDtnNetwLayer::prepareNetwPkt(short  kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, LAddress::L3Type vpaAddr)
{
	int realPktLength = 0;
	GeoDtnNetwPkt *myNetwPkt = new GeoDtnNetwPkt();
	myNetwPkt->setKind(kind);
	myNetwPkt->setSrcAddr(srcAddr);
	myNetwPkt->setSrcType(srcType);
	myNetwPkt->setDestAddr(destAddr);
	myNetwPkt->setVpaSectorId(vpaSectorId);
	myNetwPkt->setVpaAddr(vpaAddr);


	realPktLength = sizeof(kind)+sizeof(srcAddr)+sizeof(destAddr)+sizeof(unsigned long) * 2 + sizeof(int);
	realPktLength *= 8;

	myNetwPkt->setBitLength(realPktLength);

	return myNetwPkt;
}

void GeoDtnNetwLayer::sendDown(cMessage *msg)
{
	BaseLayer::sendDown(msg);
	updatingL3Sent();
}

std::vector<WaveShortMessage*> GeoDtnNetwLayer::bundleForVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : check if we have any bundle that are addressed to @vpaAddr
	std::vector<WaveShortMessage* > sortedWSM;
	bundlesIndexIterator it = bundlesIndex.find(vpaAddr);
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		for (innerIndexIterator it2 = innerMap.begin(); it2 !=innerMap.end(); it2++){
			WaveShortMessage* wsm = it2->second;
			if (wsm != NULL){
				sortedWSM.push_back(wsm);
			}
		}
	}

	// step 2 : sort the list and return it;
	std::sort(sortedWSM.begin(), sortedWSM.end(), comparatorRLAscObject);
	std::vector<WaveShortMessage* > sentWSM;
	for (std::vector<WaveShortMessage*>::iterator it = sortedWSM.begin(); it!= sortedWSM.end(); it++){
		WaveShortMessage* wsm = *it;
		if (ackSerial.count(wsm->getSerial()) > 0) {continue;}
		std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(vpaAddr);
		if (it2 != neighborhoodSession.end()){
			NetwSession newSession = it2->second;
			if (newSession.getStoredBndl().count(wsm->getSerial()) > 0){
				continue;
			}else if (newSession.getDelivredToBndl().count(wsm->getSerial()) > 0){
				continue;
			}else if (newSession.getDelivredToVpaBndl().count(wsm->getSerial()) > 0){
				continue;
			}
		}
		sentWSM.push_back(wsm);
		int modID = this->getParentModule()->getIndex();
//		cout << "(ForVPA) Node@: " << myNetwAddr << " NodeType: " << nodeType << " module ID " << modID << " sent WSM with serial: " << wsm->getSerial() << endl;
	}

	return sentWSM;
}

std::vector<WaveShortMessage*> GeoDtnNetwLayer::bundleForNode(LAddress::L3Type node)
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


std::vector<WaveShortMessage*> GeoDtnNetwLayer::bundleForFwds(LAddress::L3Type fwdDist, LAddress::L3Type fwdMETD)
{
	std::vector<WaveShortMessage* > sentWSM;
	if((fwdDist == myNetwAddr) && (fwdMETD == myNetwAddr)){
//		cout << "Forwarders are the same as me" << endl;
		return sentWSM;
	}

	if((fwdDist == myNetwAddr)){
//		cout << "Should not send to this neighbors" << endl;
		return sentWSM;
	}

//	if((fwdMETD == myNetwAddr)){
//		cout << "METD forwarder is myself" << endl;
//	}

	if((fwdDist == myNetwAddr) || (fwdMETD == myNetwAddr)){
//		cout << "SelfNode is one of the 2 fwds" << endl;
		return sentWSM;
	}

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
		std::map<LAddress::L3Type, NetwSession>::iterator itFwdDist = neighborhoodSession.find(fwdDist);
		std::map<LAddress::L3Type, NetwSession>::iterator itFwdMETD = neighborhoodSession.find(fwdMETD);
		if ((itFwdDist != neighborhoodSession.end()) && (itFwdMETD != neighborhoodSession.end())){
			NetwSession sessionFwdDist = itFwdDist->second;
			NetwSession sessionFwdMETD = itFwdMETD->second;
			if ((sessionFwdDist.getStoredBndl().count(wsm->getSerial()) > 0) &&
					(sessionFwdMETD.getStoredBndl().count(wsm->getSerial()) > 0)){
				continue;
			}else if ((sessionFwdDist.getDelivredToBndl().count(wsm->getSerial()) > 0) &&
					(sessionFwdMETD.getDelivredToBndl().count(wsm->getSerial()) > 0)){
				continue;
			}else if ((sessionFwdDist.getDelivredToVpaBndl().count(wsm->getSerial()) > 0) ||
					(sessionFwdDist.getDelivredToVpaBndl().count(wsm->getSerial()) > 0)){
				continue;
			}
		}
		sentWSM.push_back(wsm);
//		cout << "(ForFWDs) Node@: " << myNetwAddr << " NodeType: " << nodeType << "sent WSM with serial: " << wsm->getSerial() << endl;
	}

	return sentWSM;
}


std::vector<std::pair<WaveShortMessage*, int> > GeoDtnNetwLayer::bundleFor1Fwd(LAddress::L3Type fwdNode)
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

void GeoDtnNetwLayer::recordStatsFwds(std::pair<LAddress::L3Type,double> fwdDist, std::pair<LAddress::L3Type,double> fwdMETD)
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

bool GeoDtnNetwLayer::existInNetwSession(WaveShortMessage *wsm)
{
	bool found = false;

	for (std::map<LAddress::L3Type, NetwSession>::iterator it = neighborhoodSession.begin(); it != neighborhoodSession.end(); it++){
		if (it->second.getDelivredToVpaBndl().count(wsm->getSerial()) > 0){
			found = true;
			break;
		}
	}
	return found;
}

void GeoDtnNetwLayer::storeAckSerial(unsigned long  serial)
{
	if (ackSerial.count(serial) == 0){
		ackSerial.insert(serial);
	}
}



void GeoDtnNetwLayer::storeAckSerial(std::set<unsigned long > setOfSerials)
{
    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
    	storeAckSerial(*it);
    	currentNbrIsrt++;
    }
}

bool GeoDtnNetwLayer::erase(unsigned long serial)
{
	bool found = false;

	WaveShortMessage* wsm = NULL;
	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
		if (serial == (*it)->getSerial()){
			wsm = (*it);
			break;
		}
	}

	if (wsm !=NULL){
		found = DtnNetwLayer::erase(wsm);
		if (found){
			bundlesReplicaIndex.erase(serial);
		}
	}
	return found;
}

void GeoDtnNetwLayer::updateStoredBndlForSession(LAddress::L3Type srcAddr, std::set<unsigned long > storedBundle)
{
	std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
	if (it2 == neighborhoodSession.end()){
		NetwSession newSession = NetwSession(srcAddr,0);
		for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
			newSession.insertInDelivredToBndl(*it);
		}
		neighborhoodSession.insert(std::pair<LAddress::L3Type, NetwSession>(srcAddr, newSession));
	}else{
		NetwSession newSession = it2->second;
		for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
			newSession.insertInDelivredToBndl(*it);
		}
		neighborhoodSession[srcAddr] = newSession;
	}
}


std::set<LAddress::L3Type> GeoDtnNetwLayer::getKnownNeighbors()
{
	std::set<LAddress::L3Type> currentNeighbors;
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		if (it->second.getDestAddr() == myNetwAddr){ continue;}
		if (it->second.isStatus()){
			// if entry is currently active we add it to the list
			currentNeighbors.insert(it->second.getDestAddr());
		}
	}
	return currentNeighbors;
}

void GeoDtnNetwLayer::updateNeighborhoodTable(LAddress::L3Type neighbor, NetwRoute neighborEntry)
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
			if (it->second.getNodeType() == VPA){
				meetVPA = true;
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
	}

	// Adding the new entry
	std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.find(neighbor);
	if (it == neighborhoodTable.end()){
		// neighbor doesn't exist, add a new entry
		neighborhoodTable.insert(std::pair<LAddress::L3Type, NetwRoute>(neighbor, neighborEntry));
		NBHTableNbrInsert++;
	}else{
		// neighbor exists, update the old entry
		neighborhoodTable[neighbor] = neighborEntry;
	}

}

std::pair<LAddress::L3Type, double> GeoDtnNetwLayer::getBestFwdMETD()
{
	LAddress::L3Type bestForwarder = LAddress::L3NULL;
	double bestMETD = maxDbl;
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		NetwRoute entry = it->second;
		if ((entry.isStatus())&&(entry.getDestMetd() < maxDbl)){
			bestMETD = entry.getDestMetd();
			bestForwarder = entry.getDestAddr();
		}
	}
	if (bestForwarder != LAddress::L3NULL){
		coreEV << "Node:" << myNetwAddr << " BestForwarder based on METD: " << bestForwarder << " CurrentMETD: " << bestMETD << std::endl;
//		std::cout << "Node:" << myNetwAddr <<" BestForwarder based on METD: " << bestForwarder << " CurrentMETD: " << bestMETD << std::endl;
	}
	return std::pair<LAddress::L3Type, double>(bestForwarder, bestMETD);
}

std::pair<LAddress::L3Type, double> GeoDtnNetwLayer::getBestFwdDist()
{
	LAddress::L3Type bestForwarder = LAddress::L3NULL;
	double bestDist = maxDbl;
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		NetwRoute entry = it->second;
		if ((entry.isStatus())&&(entry.getDestDist() < maxDbl)){
			bestDist = entry.getDestDist();
			bestForwarder = entry.getDestAddr();
		}
	}
	if (bestForwarder != LAddress::L3NULL){
		coreEV << "Node:" << myNetwAddr << " BestForwarder based on Dist: " << bestForwarder << " CurrentDist: " << bestDist << std::endl;
//		std::cout << "Node:" << myNetwAddr << " BestForwarder based on Dist: " << bestForwarder << " CurrentDist: " << bestDist << std::endl;
	}
	return std::pair<LAddress::L3Type, double>(bestForwarder, bestDist);
}

void GeoDtnNetwLayer::DefineNodeType()
{
	cModule* parentModule = this->getParentModule();
	if (parentModule->findSubmodule("appl")!=-1){
		VPApOpp* VPAModule = FindModule<VPApOpp*>::findSubModule(parentModule);
		VEHICLEpOpp* VehicleModule = FindModule<VEHICLEpOpp*>::findSubModule(parentModule);

		if (VPAModule != NULL){
			nodeType = VPA;
		} else if (VehicleModule != NULL){
			nodeType = Veh;
		} else {
			opp_error("GeoDtnNetwLayer::DefineNodeType() - Unable to define NodeType please check existence of appl module in NED file");
		}
	}
}

void GeoDtnNetwLayer::handleLowerControl(cMessage *msg)
{
	if (isEquiped) {

	switch (msg->getKind()) {
		case NEWLY_CONNECTED:
			canITransmit = true;
			break;
		case NEW_NEIGHBOR:
			{
				if (canITransmit){

					/*
					 * Extracting destAddress and Time from controlMsgName
					 */
					char* msgName = strdup(msg->getName());

					LAddress::L3Type addr = getAddressFromName((const char*)strtok(msgName,":"));

					msgName = strtok(NULL,":");

					double time = getTimeFromName((const char*) msgName);

					/** Repeated contacts stats			*/
					recordRecontactStats(addr,time);

					/** Contacts duration stats				*/
					recordBeginContactStats(addr,time);

					if (recordContactStats){
						unsigned long contactID = startRecordingContact(addr,time);
					}

					neighborsAddress.insert(addr);
					NBHAddressNbrInsert++;
				}
			}
			break;
		case NO_NEIGHBOR_AND_DISCONNECTED:
			canITransmit = false;
			break;
		case NEW_NEIGHBOR_GONE:
			{
				/*
				 * Extracting destAddress and Time from controlMsgName
				 */
				char* msgName = strdup(msg->getName());

				LAddress::L3Type addr = getAddressFromName((const char*)strtok(msgName,":"));

				msgName = strtok(NULL,":");

				double time = getTimeFromName((const char*) msgName);

				if (recordContactStats){
					endRecordingContact(addr,time);
				}

				/** Contacts duration stats				 */
				recordEndContactStats(addr,time);

				lastBundleProposal.erase(addr);

				neighborsAddress.erase(addr);
				NBHAddressNbrDelete++;
			}
			break;
	}

	}
	delete msg;
}
