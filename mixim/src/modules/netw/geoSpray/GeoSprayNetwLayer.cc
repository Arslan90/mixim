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
#include "FindModule.h"
#include "VEHICLEpOpp.h"
#include "VPApOpp.h"
#include "algorithm"
#include "list"
#include "Coord.h"

Define_Module(GeoSprayNetwLayer);

void GeoSprayNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
	if (stage == 0){
		nbrReplica = par("nbrReplica");
		withSentTrack = par("withSentTrack").boolValue();

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

		firstSentToVPA = false;

		bndlInterestVec.setName("Evolve of interesting bundle of neighborhood");

		missedOpprVec.setName("Evolve of missed opportunities");

		meetVPA = false;

        withEMethod = par("withEMethod").boolValue();

        withExplicitE2EAck = par("withExplicitE2EAck").boolValue();

        withExplicitH2HAck = par("withExplicitH2HAck").boolValue();

	}else if (stage == 1){

	}else if (stage == 2){
		sectorId = getCurrentSector();
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
		double currentMETD, currentDist;
		if (nodeType == Veh){
			currentMETD = getGeoTraci()->getCurrentMetd();
		}else if (nodeType == VPA){
			currentMETD = 0.0;
		}else {
			opp_error("Undefined NodeType");
		}
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,currentMETD,maxDbl, simTime(), true, nodeType, getCurrentPos()));
		GeoDtnNetwPkt* netwPkt;
		sendingHelloMsg(netwPkt);
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

void GeoSprayNetwLayer::sendingHelloMsg(GeoDtnNetwPkt *netwPkt)
{
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
	netwPkt->setE2eAcks(ackSerial);
	int nbrEntries = ackSerial.size();
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
	    	storeAckSerial(receivedE2eAcks);
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
	GeoDtnNetwPkt *netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle_Offer,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
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
	GeoDtnNetwPkt *netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle_Response,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
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
		std::vector<unsigned long > oldWSM;

		// step 3 : Sending bundles with NbrReplica to transfer
		std::vector<WaveShortMessage* > sentWSM;
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
	GeoDtnNetwPkt *netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
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
		GeoDtnNetwPkt* bundleMsg;
		sectorId = getCurrentSector();
		bundleMsg = prepareNetwPkt(Bundle,myNetwAddr, nodeType, vpaAddr, sectorId ,LAddress::L3BROADCAST);
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

		std::list<unsigned long> finalReceivedWSM;

		std::list<unsigned long> receivedWSM;

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.push_back(wsm->getSerial());
			bundlesReceived++;
			emit(receiveL3SignalId,bundlesReceived);
			storeAckSerial(wsm->getSerial());
			missedOpportunities.erase(wsm->getSerial());
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			if ((!exist(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
				storeBundle(wsm);
				receivedWSM.push_back(wsm->getSerial());
				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
				if (it == bundlesReplicaIndex.end()){
					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
					bundlesRmgReplica.insert(std::pair<unsigned long, int>(wsm->getSerial(), netwPkt->getNbrReplica()));
				}
				bundlesReceived++;
				emit(receiveL3SignalId,bundlesReceived);
				missedOpportunities.erase(wsm->getSerial());
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

void GeoSprayNetwLayer::sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::list<unsigned long> wsmFinalDeliverd)
{
	GeoDtnNetwPkt* netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle_Ack,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
	std::set<unsigned long> serialOfE2EAck;
	for (std::list<unsigned long >::iterator it = wsmFinalDeliverd.begin(); it != wsmFinalDeliverd.end(); it++){
		serialOfE2EAck.insert(*it);
	}
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * serialOfE2EAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::list<unsigned long> wsmDeliverd, int nbrReplica, bool custodyTransfer)
{
	GeoDtnNetwPkt* netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle_Ack,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
	std::set<unsigned long> serialOfH2HAck;
	for (std::list<unsigned long >::iterator it = wsmDeliverd.begin(); it != wsmDeliverd.end(); it++){
		serialOfH2HAck.insert(*it);
	}
	netwPkt->setH2hAcks(serialOfH2HAck);
	netwPkt->setNbrReplica(nbrReplica);
	netwPkt->setCustodyTransfert(custodyTransfer);
	int length = sizeof(unsigned long) * serialOfH2HAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	if (withExplicitE2EAck){
		std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);

		for (std::set<unsigned long >::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
			storeAckSerial(*it);
			erase(*it);
		}
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

////////////////////////////////////////// Others methods /////////////////////////

void GeoSprayNetwLayer::updateNeighborhoodTable(LAddress::L3Type neighbor, NetwRoute neighborEntry)
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

GeoDtnNetwPkt *GeoSprayNetwLayer::prepareNetwPkt(short  kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, LAddress::L3Type vpaAddr)
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

void GeoSprayNetwLayer::storeAckSerial(unsigned long  serial)
{
	if (ackSerial.count(serial) == 0){
		ackSerial.insert(serial);
	}
}

void GeoSprayNetwLayer::storeAckSerial(std::set<unsigned long > setOfSerials)
{
    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
    	storeAckSerial(*it);
    	erase(*it);
    }
}

bool GeoSprayNetwLayer::erase(unsigned long serial)
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
			bundlesRmgReplica.erase(serial);
		}
	}
	return found;
}

bool GeoSprayNetwLayer::exist(unsigned long serial)
{
	bool found = false;

	for (bundlesIndexIterator it = bundlesIndex.begin(); it != bundlesIndex.end(); it++){
		innerIndexMap innerMap = it->second;
		for (innerIndexIterator it2 = innerMap.begin(); it2 != innerMap.end(); it2++){
			if (serial == it2->first){
				found = true;
				break;
			}
		}
	}
	return found;
}

void GeoSprayNetwLayer::handleLowerControl(cMessage *msg)
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


