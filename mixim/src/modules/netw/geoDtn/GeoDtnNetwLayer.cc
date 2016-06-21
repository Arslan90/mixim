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

void GeoDtnNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
	std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(upper_msg->getSerial());
	if (it == bundlesReplicaIndex.end()){
		bundlesReplicaIndex.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), 0));
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
//		if ((entry.getDestMetd() < maxDbl)){
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
//		if ((entry.getDestDist() < maxDbl)){
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
	if (!getKnownNeighbors().empty()){
		netwPkt->setKnownNeighbors(getKnownNeighbors());
	}
	netwPkt->setSrcMETD(METD);
	netwPkt->setSrcDist_NP_VPA(distance);
	netwPkt->setE2eAcks(ackSerial);
	std::set<unsigned long > storedBundle;
	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
		storedBundle.insert((*it)->getSerial());
	}
	netwPkt->setH2hAcks(storedBundle);
	int length = sizeof(unsigned long) * (ackSerial.size()+ storedBundle.size())+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
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
	    NetwRoute neighborEntry = NetwRoute(netwPkt->getSrcAddr(), netwPkt->getSrcMETD(), netwPkt->getSrcDist_NP_VPA(), simTime() , true, netwPkt->getSrcType());
	    updateNeighborhoodTable(netwPkt->getSrcAddr(), neighborEntry);
	    std::set<unsigned long> receivedE2eAcks = netwPkt->getE2eAcks();
	    storeAckSerial(receivedE2eAcks);
	    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
		updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
//		if(( netwPkt->getSrcDist_NP_VPA() == 0) && (netwPkt->getSrcType() == Veh)){
		if(( netwPkt->getSrcDist_NP_VPA() == 0)){
			for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
				erase(*it);
			}
		}

		std::pair<LAddress::L3Type, double> fwdDist = getBestFwdDist();
		std::pair<LAddress::L3Type, double> fwdMETD = getBestFwdMETD();

//		if ((fwdDist.first == myNetwAddr) || (fwdMETD.first == myNetwAddr)){
		if ((fwdDist.first == myNetwAddr)){
			std::set<unsigned long> interestingBndl;
			for (std::map<LAddress::L3Type, NetwSession>::iterator it = neighborhoodSession.begin(); it != neighborhoodSession.end(); it++){
				if (it->first == myNetwAddr){continue;}
				NetwSession session = it->second;
				std::set<unsigned long> delivredBndl = session.getDelivredToBndl();
				if (!delivredBndl.empty()){
					for (std::set<unsigned long >::iterator it2 = delivredBndl.begin(); it2 != delivredBndl.end(); it2++){
						unsigned long serial = (*it2);
						bool found = false;
						if ((!ackSerial.empty()) && (ackSerial.count(serial) > 0)){
							found = true;
						}
						if ((!found) && (!bundles.empty())){
							for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
								if (serial == (*it3)->getSerial()){
									found = true;
									break;
								}
							}
						}
						if (!found){
							interestingBndl.insert(serial);
							if ((fwdDist.second == 0) && (!meetVPA)){
								missedOpportunities.insert(serial);
							}
						}
					}
				}
			}
			if (!interestingBndl.empty()){
//				cout << "Current @: " << myNetwAddr << " neighboors contains " << interestingBndl.size() << " that interest me" << endl;
				bndlInterestVec.record(interestingBndl.size());
			}

			if (!missedOpportunities.empty()  && (!meetVPA)){
				missedOpprVec.record(missedOpportunities.size());
			}
		}

	    if (nodeType == VPA){
	    	return;
	    }else{
	    	/*************************** Sending Bundle Msg **********/
	    	sendingBundleMsg();

//			std::vector<WaveShortMessage*> bundleToSent = bundleForNode(netwPkt->getSrcAddr());
//	//		cout << "Current@: " << myNetwAddr << " nbrToSent: " << bundleToSent.size() << " current time: " << simTime().dbl() << endl;
//
//			for (std::vector<WaveShortMessage* >::iterator it = bundleToSent.begin(); it != bundleToSent.end(); it++){
//				GeoDtnNetwPkt* bundleMsg;
//				if (nodeType == Veh){
//					sectorId = geoTraci->getCurrentSector();
//					bundleMsg = prepareNetwPkt(Bundle,myNetwAddr, nodeType, netwPkt->getSrcAddr() , sectorId ,LAddress::L3BROADCAST);
//				}else{
//					opp_error("Undefined NodeType");
//				}
//				bundleMsg->encapsulate((*it)->dup());
//				sendDown(bundleMsg);
//				bundlesReplicaIndex[(*it)->getSerial()]++;
//			}
	    }
	}
}

void GeoDtnNetwLayer::sendingBundleMsg()
{
	// Check if VPA is a neighbor
	bool vpaInNeighborhood = false;
	std::pair<LAddress::L3Type, NetwRoute> vpaEntry;
	for (std::map<LAddress::L3Type, NetwRoute>::iterator it = neighborhoodTable.begin(); it != neighborhoodTable.end(); it++){
		if (it->second.getNodeType() == VPA){
			vpaInNeighborhood = true;
			vpaEntry = *it;
			break;
		}
	}
	std::vector<WaveShortMessage*> bundleToSent;
	if (vpaInNeighborhood){
		bundleToSent = bundleForVPA(vpaEntry.second.getDestAddr());
//		cout << "Current@: " << myNetwAddr << " nbrToSent: " << bundleToSent.size() << " current time: " << simTime().dbl() << endl;
		if (!firstSentToVPA){
			bndlSentToVPA+=bundleToSent.size();
			firstSentToVPA = true;
		}
		totalBndlSentToVPA+=bundleToSent.size();

		for (std::vector<WaveShortMessage* >::iterator it = bundleToSent.begin(); it != bundleToSent.end(); it++){
			GeoDtnNetwPkt* bundleMsg;
			if (nodeType == Veh){
				sectorId = geoTraci->getCurrentSector();
				bundleMsg = prepareNetwPkt(Bundle,myNetwAddr, nodeType, vpaEntry.second.getDestAddr(), sectorId ,LAddress::L3BROADCAST);
			}else{
				opp_error("Undefined NodeType");
			}
			bundleMsg->encapsulate((*it)->dup());
			sendDown(bundleMsg);
			bundlesReplicaIndex[(*it)->getSerial()]++;
		}

	}else{
		// Define bundle to Forward to selected forwarders
		std::pair<LAddress::L3Type, double> fwdDist = getBestFwdDist();
		std::pair<LAddress::L3Type, double> fwdMETD = getBestFwdMETD();

		recordStatsFwds(fwdDist, fwdMETD);
		bundleToSent = bundleForFwds(fwdDist.first, fwdMETD.first);

		for (std::vector<WaveShortMessage* >::iterator it = bundleToSent.begin(); it != bundleToSent.end(); it++){
			GeoDtnNetwPkt* bundleMsg;
			if (nodeType == Veh){
				sectorId = geoTraci->getCurrentSector();
				bundleMsg = prepareNetwPkt(Bundle,myNetwAddr, nodeType, LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
				bundleMsg->setFwdDist(fwdDist.first);
				bundleMsg->setFwdMETD(fwdMETD.first);
			}else{
				opp_error("Undefined NodeType");
			}
			bundleMsg->encapsulate((*it)->dup());
			sendDown(bundleMsg);
			bundlesReplicaIndex[(*it)->getSerial()]++;
		}
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

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.push_back(wsm->getSerial());
			bundlesReceived++;
			storeAckSerial(wsm->getSerial());
//			cout << "(VPA) Node@: " << myNetwAddr << " NodeType: " << nodeType << "received WSM with serial: " << wsm->getSerial() << endl;
			missedOpportunities.erase(wsm->getSerial());
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
//			if ( ! (exist(wsm)||(ackSerial.count(wsm->getSerial()) > 0))){
			if ((!exist(wsm)) && (ackSerial.count(wsm->getSerial()) == 0)){
				storeBundle(wsm);
				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
				if (it == bundlesReplicaIndex.end()){
					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
				}
				bundlesReceived++;
				missedOpportunities.erase(wsm->getSerial());
			}
			receivedWSM.push_back(wsm->getSerial());
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
