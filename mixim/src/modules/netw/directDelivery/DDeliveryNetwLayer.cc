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

#include "DDeliveryNetwLayer.h"
#include "FindModule.h"
#include "VEHICLEpOpp.h"
#include "VPApOpp.h"
#include "algorithm"
#include "list"
#include "limits"

Define_Module(DDeliveryNetwLayer);

void DDeliveryNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
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

void DDeliveryNetwLayer::handleLowerMsg(cMessage *msg)
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

void DDeliveryNetwLayer::handleSelfMsg(cMessage *msg)
{
	if (msg == heartBeatMsg){
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,maxDbl,maxDbl, simTime(), true, nodeType, getCurrentPos()));
		GeoDtnNetwPkt* netwPkt;
		sendingHelloMsg(netwPkt);
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void DDeliveryNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
}

void DDeliveryNetwLayer::finish()
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

void DDeliveryNetwLayer::sendingHelloMsg(GeoDtnNetwPkt *netwPkt)
{
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
//	cout << "Sending Hello packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void DDeliveryNetwLayer::handleHelloMsg(GeoDtnNetwPkt *netwPkt)
{
	// If not the same sector ignore message
	if (netwPkt->getVpaSectorId() != sectorId){
		return;
	}else{
		/*************************** Handling Hello Msg **********/
	    NetwRoute neighborEntry = NetwRoute(netwPkt->getSrcAddr(), netwPkt->getSrcMETD(), netwPkt->getSrcDist_NP_VPA(), simTime() , true, netwPkt->getSrcType(), netwPkt->getCurrentPos());
	    updateNeighborhoodTable(netwPkt->getSrcAddr(), neighborEntry);

		if (nodeType == VPA){
			return;
		}else{
			/*************************** Sending Bundle Msg **********/

			if (netwPkt->getSrcType() == VPA){
				sendingBundleMsgToVPA(netwPkt->getSrcAddr());
				vpaContactDistance.push_back(getCurrentPos().distance(netwPkt->getCurrentPos()));
			}
		}
	}
}

void DDeliveryNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : check if we have any bundle that are addressed to @vpaAddr
	std::vector<WaveShortMessage* > wsmToSend = bundleForVPA(vpaAddr);
//	std::vector<WaveShortMessage* > wsmToSend = bundleForNode(vpaAddr);
	if (!firstSentToVPA){
		bndlSentToVPA+=wsmToSend.size();
		firstSentToVPA = true;
	}
	totalBndlSentToVPA+=wsmToSend.size();


//	std::vector<WaveShortMessage* > wsmToSend;
//	bundlesIndexIterator it = bundlesIndex.find(vpaAddr);
//	if (it != bundlesIndex.end()){
//		innerIndexMap innerMap(it->second);
//		for (innerIndexIterator it2 = innerMap.begin(); it2 !=innerMap.end(); it2++){
//			WaveShortMessage* wsm = it2->second;
//			if (wsm != NULL){
//				wsmToSend.push_back(wsm);
//			}
//		}
//	}

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

void DDeliveryNetwLayer::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
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
			opp_error("DDeliveryNetwLayer::handleBundleMsg() - Reception of bundle by not recipient address not allowed");
		}
		if (!finalReceivedWSM.empty()){
			bundleAckMsg = prepareNetwPkt(Bundle_Ack,myNetwAddr, nodeType, netwPkt->getSrcAddr(), sectorId ,LAddress::L3BROADCAST);
			sendingBundleAckMsg(bundleAckMsg, finalReceivedWSM);
		}
	}
}

void DDeliveryNetwLayer::sendingBundleAckMsg(GeoDtnNetwPkt *netwPkt, std::list<unsigned long> wsmFinalDeliverd)
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

void DDeliveryNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
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

GeoDtnNetwPkt *DDeliveryNetwLayer::prepareNetwPkt(short  kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, LAddress::L3Type vpaAddr)
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

std::vector<WaveShortMessage*> DDeliveryNetwLayer::bundleForVPA(LAddress::L3Type vpaAddr)
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

void DDeliveryNetwLayer::storeAckSerial(unsigned long  serial)
{
	if (ackSerial.count(serial) == 0){
		ackSerial.insert(serial);
	}
}

void DDeliveryNetwLayer::storeAckSerial(std::set<unsigned long > setOfSerials)
{
    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
    	storeAckSerial(*it);
    	erase(*it);
    }
}

bool DDeliveryNetwLayer::erase(unsigned long serial)
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
	}
	return found;
}

bool DDeliveryNetwLayer::exist(unsigned long serial)
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

void DDeliveryNetwLayer::handleLowerControl(cMessage *msg)
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

