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

#include "EpidemicNetwLayerV2.h"
#include "FindModule.h"
#include "VEHICLEpOpp.h"
#include "VPApOpp.h"
#include "algorithm"
#include "list"
#include "limits"

Define_Module(EpidemicNetwLayerV2);

void EpidemicNetwLayerV2::initialize(int stage)
{
    // TODO - Generated method body
	LEG_DtnNetwLayer::initialize(stage);
	if (stage == 0){
		NBHAddressNbrInsert  = 0;
		NBHAddressNbrDelete  = 0;

		totalBundlesReceived = 0;
		bndlSentToVPA = 0;
		totalBndlSentToVPA = 0;

		firstSentToVPA = false;

		meetVPA = false;

	}else if (stage == 1){

	}else if (stage == 2){
		sectorId = getCurrentSector();
	}
}

void EpidemicNetwLayerV2::handleLowerMsg(cMessage *msg)
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
			if ((netwPkt->getDestAddr() == myNetwAddr) || (netwPkt->getDestAddr() == LAddress::L3BROADCAST)){
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

void EpidemicNetwLayerV2::handleSelfMsg(cMessage *msg)
{
	if (msg == heartBeatMsg){
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,maxDbl,maxDbl, simTime(), true, nodeType, getCurrentPos()));
		GeoDtnNetwPkt* netwPkt;
		sendingHelloMsg(netwPkt);
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void EpidemicNetwLayerV2::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
	std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(upper_msg->getSerial());
	if (it == bundlesReplicaIndex.end()){
		bundlesReplicaIndex.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), 0));
	}
}

void EpidemicNetwLayerV2::finish()
{
	recordScalar("# insertOper Oracle", NBHAddressNbrInsert);
	recordScalar("# delOper Oracle", NBHAddressNbrDelete);
	recordScalar("# insertOper NBHTable", NBHTableNbrInsert);
	recordScalar("# delOper NBHTable", NBHTableNbrDelete);

	recordScalar("# Redundant Bundle at L3", (totalBundlesReceived- bundlesReceived));

	recordScalar("# Bndl Sent to VPA (total)", totalBndlSentToVPA);
	recordScalar("# Bndl Sent to VPA (first)", bndlSentToVPA);

	recordAllScalars();
}

void EpidemicNetwLayerV2::sendingHelloMsg(GeoDtnNetwPkt *netwPkt)
{
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
	netwPkt->setE2eAcks(ackSerial);
	std::set<unsigned long > storedBundle;
//	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
//		storedBundle.insert((*it)->getSerial());
//	}
//	netwPkt->setH2hAcks(storedBundle);
	int nbrEntries = ackSerial.size()+ storedBundle.size();
	int length = sizeof(unsigned long) * (nbrEntries)+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void EpidemicNetwLayerV2::handleHelloMsg(GeoDtnNetwPkt *netwPkt)
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
	    	storeAckSerial(receivedE2eAcks);
	    }
//	    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
//	    if (!storedBundle.empty()){
//	    	updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
//	    }
		if (nodeType == VPA){
			return;
		}else{
			/*************************** Sending Bundle Msg **********/

		    if (netwPkt->getSrcType() == VPA){
		    	sendingBundleMsgToVPA(netwPkt->getSrcAddr());
		    	vpaContactDistance.push_back(getCurrentPos().distance(netwPkt->getCurrentPos()));
		    }else if (netwPkt->getSrcType() == Veh){
//		    	sendingBundleMsg(netwPkt->getSrcAddr());
			    GeoDtnNetwPkt* bundleOfferMsg;
			    std::list<unsigned long> wsmStoredBndl;
			    sendingBundleOfferMsg(bundleOfferMsg, netwPkt->getSrcAddr(), wsmStoredBndl);
		    }
		}
	}
}

void EpidemicNetwLayerV2::sendingBundleOfferMsg(GeoDtnNetwPkt *netwPkt, LAddress::L3Type destAddr, std::list<unsigned long > wsmStoredBndl)
{
	netwPkt = prepareNetwPkt(Bundle_Offer,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);

	std::set<unsigned long> serialOfH2hAck;
	for (std::list<WaveShortMessage* >::iterator it = bundles.begin(); it != bundles.end(); it++){
		serialOfH2hAck.insert((*it)->getSerial());
	}

	netwPkt->setH2hAcks(serialOfH2hAck);
	int length = sizeof(unsigned long) * serialOfH2hAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
//	cout << "Sending BundleOffer packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void EpidemicNetwLayerV2::handleBundleOfferMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving BundleOffer spacket from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;

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

	GeoDtnNetwPkt* bundleResponseMsg;
	sendingBundleResponseMsg(bundleResponseMsg, netwPkt->getSrcAddr(), serialResponseBndl);
}

void EpidemicNetwLayerV2::sendingBundleResponseMsg(GeoDtnNetwPkt *netwPkt, LAddress::L3Type destAddr, std::set<unsigned long > wsmResponseBndl)
{
	netwPkt = prepareNetwPkt(Bundle_Response,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
	netwPkt->setH2hAcks(wsmResponseBndl);
	int length = sizeof(unsigned long) * wsmResponseBndl.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);

//	cout << "Sending BundleResponse packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void EpidemicNetwLayerV2::handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt)
{
	if (nodeType == Veh){
		std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();

		// step 1 : Build bundle list to send before reordering
		std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair;
		for (std::set<unsigned long>::iterator it = serialResponseBndl.begin(); it != serialResponseBndl.end(); it++){
			unsigned long serial = *it;
			if (exist(serial)){
				std::map<unsigned long, int>::iterator it2 = bundlesReplicaIndex.find(serial);
				if (it2 == bundlesReplicaIndex.end()){
					opp_error("Bundle Found but not in replica index");
				}else{
					for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
						if ((*it3)->getSerial() == serial){
							sortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it2->second));
							break;
						}
					}
				}
			}
		}

		std::sort(sortedWSMPair.begin(), sortedWSMPair.end(), comparatorRCAscObject);

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
			sentWSM.push_back(wsm);
		}
		sendingBundleMsg(netwPkt->getSrcAddr(), sentWSM);
	}
}

void EpidemicNetwLayerV2::sendingBundleMsg(LAddress::L3Type destAddr, std::vector<WaveShortMessage* > wsmToSend)
{
	// step 1 : define bundles to forward
//	std::vector<WaveShortMessage* > wsmToSend = bundleForNode(destAddr);

	// step 2 : send bundles
	for (std::vector<WaveShortMessage* >::iterator it = wsmToSend.begin(); it != wsmToSend.end(); it++){
		WaveShortMessage* wsm = *it;
		GeoDtnNetwPkt* bundleMsg;
		sectorId = getCurrentSector();
		bundleMsg = prepareNetwPkt(Bundle,myNetwAddr, nodeType, LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
		bundleMsg->encapsulate(wsm->dup());
//			cout << "Sending Bundle packet from " << bundleMsg->getSrcAddr() << " addressed to 2Fwds " << fwdDist.first << " & " << fwdMETD.first << " Bundle Serial " << wsm->getSerial() << std::endl;
		sendDown(bundleMsg);
		bundlesReplicaIndex[(*it)->getSerial()]++;
	}
}

void EpidemicNetwLayerV2::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : check if we have any bundle that are addressed to @vpaAddr
//	std::vector<WaveShortMessage* > wsmToSend = bundleForVPA(vpaAddr);
    std::vector<WaveShortMessage* > wsmToSend = bundleForNode(vpaAddr);
	if (!firstSentToVPA){
		bndlSentToVPA+=wsmToSend.size();
		firstSentToVPA = true;
	}
	totalBndlSentToVPA+=wsmToSend.size();

	// step 2 : send bundles
	for (std::vector<WaveShortMessage*>::iterator it = wsmToSend.begin(); it!= wsmToSend.end(); it++){
		WaveShortMessage* wsm = *it;
		GeoDtnNetwPkt* bundleForVPA;
		bundleForVPA = prepareNetwPkt(Bundle,myNetwAddr, nodeType, vpaAddr, sectorId ,LAddress::L3BROADCAST);
		bundleForVPA->encapsulate(wsm->dup());
//		cout << "Sending Bundle packet from " << bundleForVPA->getSrcAddr() << " addressed to VPA " << bundleForVPA->getDestAddr() << std::endl;
		sendDown(bundleForVPA);
	}
}

void EpidemicNetwLayerV2::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);
		totalBundlesReceived++;

		GeoDtnNetwPkt* bundleAckMsg;
		std::list<unsigned long> finalReceivedWSM;

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.push_back(wsm->getSerial());
			bundlesReceived++;
			emit(receiveL3SignalId,bundlesReceived);
			storeAckSerial(wsm->getSerial());

		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			if ((!exist(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
				storeBundle(wsm);
				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
				if (it == bundlesReplicaIndex.end()){
					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
				}
				bundlesReceived++;
				emit(receiveL3SignalId,bundlesReceived);
			}
		}
		if (!finalReceivedWSM.empty()){
			bundleAckMsg = prepareNetwPkt(Bundle_Ack,myNetwAddr, nodeType, netwPkt->getSrcAddr(), sectorId ,LAddress::L3BROADCAST);
			sendingBundleAckMsg(bundleAckMsg, finalReceivedWSM);
		}
	}
}

void EpidemicNetwLayerV2::sendingBundleAckMsg(GeoDtnNetwPkt *netwPkt, std::list<unsigned long> wsmFinalDeliverd)
{
	std::set<unsigned long> serialOfE2EAck;
	for (std::list<unsigned long >::iterator it = wsmFinalDeliverd.begin(); it != wsmFinalDeliverd.end(); it++){
		serialOfE2EAck.insert(*it);
	}
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * serialOfE2EAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void EpidemicNetwLayerV2::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	std::map<LAddress::L3Type, NetwSession>::iterator it2;
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
		ackReceivedPerVPA.insert(*it);
	}
}

////////////////////////////////////////// Others methods /////////////////////////

GeoDtnNetwPkt *EpidemicNetwLayerV2::prepareNetwPkt(short  kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, LAddress::L3Type vpaAddr)
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

std::vector<WaveShortMessage*> EpidemicNetwLayerV2::bundleForVPA(LAddress::L3Type vpaAddr)
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
	}

	return sentWSM;
}

std::vector<WaveShortMessage*> EpidemicNetwLayerV2::bundleForNode(LAddress::L3Type node)
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

	return sentWSM;
}

void EpidemicNetwLayerV2::storeAckSerial(unsigned long  serial)
{
	if (ackSerial.count(serial) == 0){
		ackSerial.insert(serial);
	}
}

void EpidemicNetwLayerV2::storeAckSerial(std::set<unsigned long > setOfSerials)
{
    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
    	storeAckSerial(*it);
    	erase(*it);
    }
}

bool EpidemicNetwLayerV2::erase(unsigned long serial)
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
		found = LEG_DtnNetwLayer::erase(wsm);
	}
	return found;
}

bool EpidemicNetwLayerV2::exist(unsigned long serial)
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

void EpidemicNetwLayerV2::handleLowerControl(cMessage *msg)
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

