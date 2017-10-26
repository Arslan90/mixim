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

Define_Module(DDeliveryNetwLayer);

void DDeliveryNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
	if (stage == 0){
		totalBundlesReceived = 0;
		bndlSentToVPA = 0;
		totalBndlSentToVPA = 0;

		firstSentToVPA = false;

		meetVPA = false;

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
		sendingHelloMsg();
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void DDeliveryNetwLayer::finish()
{
	recordScalar("# Redundant Bundle at L3", (totalBundlesReceived- bundlesReceived));

	recordScalar("# Bndl Sent to VPA (total)", totalBndlSentToVPA);
	recordScalar("# Bndl Sent to VPA (first)", bndlSentToVPA);

	recordAllScalars();
}

void DDeliveryNetwLayer::sendingHelloMsg()
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
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

		/*************************** Sending Bundle Msg **********/
		if (nodeType == Veh){
			if (netwPkt->getSrcType() == VPA){
				sendingBundleMsg(netwPkt->getSrcAddr());
				vpaContactDistance.push_back(getCurrentPos().distance(netwPkt->getCurrentPos()));
			}
		}
	}
}

void DDeliveryNetwLayer::sendingBundleMsg(LAddress::L3Type destAddr)
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
	std::vector<unsigned long > oldWSM;
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = sortedWSMPair.begin(); it != sortedWSMPair.end(); it++){
		WaveShortMessage* wsm = it->first;
		if (ackSerial.count(wsm->getSerial()) > 0) {continue;}
		std::map<LAddress::L3Type, NetwSession>::iterator itNode = neighborhoodSession.find(destAddr);
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
		sentWSM.push_back(wsm);
	}

	if (!firstSentToVPA){
		bndlSentToVPA+=sentWSM.size();
		firstSentToVPA = true;
	}
	totalBndlSentToVPA+=sentWSM.size();

	for (std::vector<WaveShortMessage* >::iterator it = sentWSM.begin(); it != sentWSM.end(); it++){
		WaveShortMessage* wsm = *it;
		GeoDtnNetwPkt* bundleMsg = new GeoDtnNetwPkt();
		prepareNetwPkt(bundleMsg, Bundle, destAddr);
		bundleMsg->encapsulate(wsm->dup());
		sendDown(bundleMsg);
	}

	for (std::vector<unsigned long >::iterator it = oldWSM.begin(); it != oldWSM.end(); it++){
		if (erase(*it)){
			nbrDeletedWithTTL++;
		}
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

		std::set<unsigned long> finalReceivedWSM;

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
//			opp_error("DDeliveryNetwLayer::handleBundleMsg() - Reception of bundle by not recipient address not allowed");
		}
		if (!finalReceivedWSM.empty()){
			sendingBundleAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM);
		}
	}
}

void DDeliveryNetwLayer::sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long > wsmFinalDeliverd)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);
	std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * serialOfE2EAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void DDeliveryNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
	updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
	storeAckSerials(finalDelivredToBndl);
}

////////////////////////////////////////// Others methods /////////////////////////
