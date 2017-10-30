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

#include "GeoSprayNetwLayer.h"

Define_Module(GeoSprayNetwLayer);

void GeoSprayNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
	if (stage == 0){
		nbrReplica = par("nbrReplica");

		totalBundlesReceived = 0;
		bndlSentToVPA = 0;
		totalBndlSentToVPA = 0;

		firstSentToVPA = false;

		meetVPA = false;

        withEMethod = par("withEMethod").boolValue();

        withExplicitE2EAck = par("withExplicitE2EAck").boolValue();

        withExplicitH2HAck = par("withExplicitH2HAck").boolValue();
	}
}

void GeoSprayNetwLayer::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    GeoDtnNetwPkt *netwPkt = check_and_cast<GeoDtnNetwPkt *>(m->decapsulate());

    coreEV << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Addressed to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

    switch (netwPkt->getKind()) {
		case HELLO:
			handleHelloMsg(netwPkt);
			break;
		case Bundle_Ack:
			if (netwPkt->getDestAddr() == myNetwAddr){
				handleBundleAckMsg(netwPkt);
			}
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
		default:
			break;
	}

    updatingL3Received();

    delete netwPkt;
    delete m;
}

void GeoSprayNetwLayer::handleSelfMsg(cMessage *msg)
{
	if (msg == heartBeatMsg){
		double currentMETD;
		if (nodeType == Veh){
			currentMETD = getGeoTraci()->getCurrentMetd();
		}else if (nodeType == VPA){
			currentMETD = 0.0;
		}else {
			opp_error("Undefined NodeType");
		}
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,currentMETD,maxDbl, simTime(), true, nodeType, getCurrentPos()));
		sendingHelloMsg();
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void GeoSprayNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
	std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(upper_msg->getSerial());
	if (it == bundlesReplicaIndex.end()){
		bundlesReplicaIndex.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), 0));
		bundlesRmgReplica.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), nbrReplica));
	}
}

void GeoSprayNetwLayer::finish()
{
	recordScalar("# Redundant Bundle at L3", (totalBundlesReceived- bundlesReceived));

	recordScalar("# Bndl Sent to VPA (total)", totalBndlSentToVPA);
	recordScalar("# Bndl Sent to VPA (first)", bndlSentToVPA);

	recordAllScalars();
}

void GeoSprayNetwLayer::sendingHelloMsg()
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
	std::set<unsigned long> storedAck = std::set<unsigned long>(ackSerial);
	netwPkt->setE2eAcks(storedAck);
	int nbrEntries = storedAck.size();
	int length = sizeof(unsigned long) * (nbrEntries)+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleHelloMsg(GeoDtnNetwPkt *netwPkt)
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
	    /*************************** Sending Bundle Msg **********/
		if (nodeType == Veh){
			if (netwPkt->getSrcType() == VPA){
				sendingBundleMsgToVPA(netwPkt->getSrcAddr());
			}else if (netwPkt->getSrcType() == Veh){
				sendingBundleOfferMsg(netwPkt->getSrcAddr());
			}
		}
	}
}

void GeoSprayNetwLayer::sendingBundleOfferMsg(LAddress::L3Type destAddr)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Offer, destAddr);
	std::set<unsigned long> serialOfH2hAck;
	for (std::list<WaveShortMessage* >::iterator it = bundles.begin(); it != bundles.end(); it++){
		serialOfH2hAck.insert((*it)->getSerial());
	}
	netwPkt->setH2hAcks(serialOfH2hAck);
	int length = sizeof(unsigned long) * serialOfH2hAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleBundleOfferMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> serialStoredBndl;
	std::set<unsigned long> serialResponseBndl;

	serialStoredBndl = netwPkt->getH2hAcks();
    if (!serialStoredBndl.empty()){
    	updateStoredBndlForSession(netwPkt->getSrcAddr(), serialStoredBndl);
    }

	for (std::set<unsigned long>::iterator it = serialStoredBndl.begin(); it != serialStoredBndl.end(); it++){
		unsigned long serial = *it;
		if ((ackSerial.count(serial) == 1) || (exist(serial))){
			continue;
		}else{
			serialResponseBndl.insert(serial);
		}
	}

	if (!serialResponseBndl.empty()){
		sendingBundleResponseMsg(netwPkt->getSrcAddr(), serialResponseBndl);
	}
}

void GeoSprayNetwLayer::sendingBundleResponseMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmResponseBndl)
{
	double currentMETD;
	if (nodeType == Veh){
		currentMETD = getGeoTraci()->getCurrentMetd();
	}else if (nodeType == VPA){
		currentMETD = 0.0;
	}else {
		opp_error("Undefined NodeType");
	}
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Response, destAddr);
	netwPkt->setH2hAcks(wsmResponseBndl);
	netwPkt->setSrcMETD(currentMETD);
	int length = sizeof(unsigned long) * wsmResponseBndl.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();
	double currentMETD = getGeoTraci()->getCurrentMetd();
	if (currentMETD > netwPkt->getSrcMETD()){
		// step 1 : Build bundle list to send before reordering
		std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
		for (std::set<unsigned long>::iterator it = serialResponseBndl.begin(); it != serialResponseBndl.end(); it++){
			unsigned long serial = *it;
			if (exist(serial)){
				std::map<unsigned long, int>::iterator it2 = bundlesReplicaIndex.find(serial);
				if (it2 == bundlesReplicaIndex.end()){
					opp_error("Bundle Found but not in rmg replica index");
				}else{
					for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
						if ((*it3)->getSerial() == serial){
							unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it2->second));
							break;
						}
					}
				}
			}
		}

		// step 2 : Reordering bundle list
		std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair = compAsFn_schedulingStrategy(unsortedWSMPair);


		// step 3 : Sending bundles with NbrReplica to transfer
		std::vector<WaveShortMessage* > sentWSM;
		std::vector<unsigned long > oldWSM;
		for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = sortedWSMPair.begin(); it != sortedWSMPair.end(); it++){
			WaveShortMessage* wsm = it->first;
			if (ackSerial.count(wsm->getSerial()) > 0) {continue;}
			std::map<LAddress::L3Type, NetwSession>::iterator itNode = neighborhoodSession.find(netwPkt->getSrcAddr());
			if ((itNode != neighborhoodSession.end())){
				NetwSession sessionNode = itNode->second;
				if ((sessionNode.getStoredBndl().count(wsm->getSerial()) > 0)){
					continue;
				}else if ((sessionNode.getDelivredToBndl().count(wsm->getSerial()) > 0)){
					continue;
				}else if ((sessionNode.getDelivredToVpaBndl().count(wsm->getSerial()) > 0)){
					continue;
				}
			}
			if (withTTL){
				double duration = (simTime()-wsm->getTimestamp()).dbl();
				if (duration > ttl){
					oldWSM.push_back(wsm->getSerial());
					continue;
				}
			}

			// Fixing the number of replica to send
			bool custodyTransfert = false;
			int nbrReplicaToSend = 0;
			std::map<unsigned long, int>::iterator it2 = bundlesRmgReplica.find(wsm->getSerial());
			if (it2 == bundlesRmgReplica.end()){
				opp_error("Remaining Replica not found");
			}else{
				int remainingCopy = it2->second;
				if ((remainingCopy <= 0 ) || (remainingCopy > nbrReplica )){
					opp_error("Invalid remaining replica");
				}else if (remainingCopy == 1 ){
					nbrReplicaToSend = 1;
					custodyTransfert = true;
				}else{
					nbrReplicaToSend =  remainingCopy / 2;
				}
			}

			// sending bundles
			sendingBundleMsg(netwPkt->getSrcAddr(), wsm->dup(), nbrReplicaToSend, custodyTransfert);

		}

		for (std::vector<unsigned long >::iterator it = oldWSM.begin(); it != oldWSM.end(); it++){
			if (erase(*it)){
				nbrDeletedWithTTL++;
			}
		}
	}
}

void GeoSprayNetwLayer::sendingBundleMsg(LAddress::L3Type destAddr, WaveShortMessage* wsm,  int nbrReplica, bool custodyTransfer)
{
	GeoDtnNetwPkt *netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle, destAddr);
	netwPkt->setNbrReplica(nbrReplica);
	netwPkt->encapsulate(wsm);
	sendDown(netwPkt);
	if (!withExplicitH2HAck){
		bundlesReplicaIndex[wsm->getSerial()] = bundlesReplicaIndex[wsm->getSerial()] + nbrReplica;
		bundlesRmgReplica[wsm->getSerial()] = bundlesRmgReplica[wsm->getSerial()] - nbrReplica;
		if (custodyTransfer){
			erase(wsm->getSerial());
		}
	}
}

void GeoSprayNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
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
	std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair = compAsFn_schedulingStrategy(unsortedWSMPair);

	// step 3 : Sending bundles with NbrReplica to transfer
	std::vector<WaveShortMessage* > sentWSM;
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = sortedWSMPair.begin(); it != sortedWSMPair.end(); it++){
		WaveShortMessage* wsm = it->first;
		if (ackSerial.count(wsm->getSerial()) > 0) {continue;}
		std::map<LAddress::L3Type, NetwSession>::iterator itNode = neighborhoodSession.find(vpaAddr);
		if ((itNode != neighborhoodSession.end())){
			NetwSession sessionNode = itNode->second;
			if ((sessionNode.getStoredBndl().count(wsm->getSerial()) > 0)){
				continue;
			}else if ((sessionNode.getDelivredToBndl().count(wsm->getSerial()) > 0)){
				continue;
			}else if ((sessionNode.getDelivredToVpaBndl().count(wsm->getSerial()) > 0)){
				continue;
			}
		}
		sentWSM.push_back(wsm);
	}

	if (!firstSentToVPA){
		bndlSentToVPA+=sentWSM.size();
		firstSentToVPA = true;
	}
	totalBndlSentToVPA+=sentWSM.size();

	for (std::vector<WaveShortMessage* >::iterator it = sentWSM.begin(); it != sentWSM.end(); it++){
		WaveShortMessage* wsm = *it;
		unsigned long serial = wsm->getSerial();
		GeoDtnNetwPkt *bundleMsg = new GeoDtnNetwPkt();
		prepareNetwPkt(bundleMsg, Bundle, vpaAddr);
		bundleMsg->encapsulate(wsm->dup());
		sendDown(bundleMsg);
		if(!withExplicitE2EAck){
			storeAckSerial(serial);
			erase(serial);
		}
	}

}

void GeoSprayNetwLayer::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);
		totalBundlesReceived++;

		std::set<unsigned long> finalReceivedWSM;

		std::set<unsigned long> receivedWSM;

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.insert(wsm->getSerial());
			bundlesReceived++;
			emit(receiveL3SignalId,bundlesReceived);
			storeAckSerial(wsm->getSerial());
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			if ((!exist(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
				storeBundle(wsm);
				receivedWSM.insert(wsm->getSerial());
				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
				if (it == bundlesReplicaIndex.end()){
					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
					bundlesRmgReplica.insert(std::pair<unsigned long, int>(wsm->getSerial(), netwPkt->getNbrReplica()));
				}
				bundlesReceived++;
				emit(receiveL3SignalId,bundlesReceived);
			}
		}
		if (!finalReceivedWSM.empty() && withExplicitE2EAck){
			sendingBundleE2EAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM);
		}

		if (!receivedWSM.empty()  && withExplicitH2HAck){
			sendingBundleH2HAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM, netwPkt->getNbrReplica(), netwPkt->getCustodyTransfert());
		}
	}
}

void GeoSprayNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	if (withExplicitE2EAck){
		std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
		storeAckSerials(finalDelivredToBndl);
	}

	if (withExplicitH2HAck){
		std::set<unsigned long> delivredToBndl = netwPkt->getH2hAcks();
		if (delivredToBndl.size()>1){
			opp_warning("GeoSprayNetwLayer::handleBundleAckMsg - Unsupported handling of more then a single H2HAck");
		}
		for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
			bundlesReplicaIndex[*it] = bundlesReplicaIndex[*it] + nbrReplica;
			bundlesRmgReplica[*it] = bundlesRmgReplica[*it] - nbrReplica;
			if (netwPkt->getCustodyTransfert()){
				erase(*it);
			}
		}
	}
}

void GeoSprayNetwLayer::sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmFinalDeliverd)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);
	std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * serialOfE2EAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDeliverd, int nbrReplica, bool custodyTransfer)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);
	std::set<unsigned long> serialOfH2HAck = std::set<unsigned long>(wsmDeliverd);
	netwPkt->setH2hAcks(serialOfH2HAck);
	netwPkt->setNbrReplica(nbrReplica);
	netwPkt->setCustodyTransfert(custodyTransfer);
	int length = sizeof(unsigned long) * serialOfH2HAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}
////////////////////////////////////////// Others methods /////////////////////////

bool GeoSprayNetwLayer::erase(unsigned long serial)
{
	bool found = DtnNetwLayer::erase(serial);

	if (found){
		bundlesReplicaIndex.erase(serial);
		bundlesRmgReplica.erase(serial);
	}

	return found;
}

GeoTraCIMobility *GeoSprayNetwLayer::getGeoTraci()
{
	GeoTraCIMobility* geo;
	switch (nodeType) {
		case Veh:
			geo = GeoTraCIMobilityAccess().get(getParentModule());
			break;
		default:
			opp_error("GeoSprayNetwLayer::getGeoTraci() - Unable to retrieve GeoTraCIMobility because node is not of type Veh");
			break;
	}
	return geo;
}
