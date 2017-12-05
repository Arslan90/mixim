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

#include "EpidemicNetwLayer.h"

Define_Module(EpidemicNetwLayer);

void EpidemicNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
	if (stage == 0){
		// Nothing to do
	}
}

void EpidemicNetwLayer::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    GeoDtnNetwPkt *netwPkt = check_and_cast<GeoDtnNetwPkt *>(m->decapsulate());

    coreEV << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Addressed to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

   	if (isEquiped){
		switch (netwPkt->getKind()) {
			case HELLO:
				handleHelloMsg(netwPkt);
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

void EpidemicNetwLayer::handleSelfMsg(cMessage *msg)
{
	if (msg == heartBeatMsg){
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,maxDbl,maxDbl, simTime(), true, nodeType, getCurrentPos()));
		sendingHelloMsg();
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void EpidemicNetwLayer::finish()
{
	recordAllScalars();
}

void EpidemicNetwLayer::sendingHelloMsg()
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
	std::set<unsigned long> storedAck = std::set<unsigned long>(ackSerial);
	netwPkt->setE2eAcks(storedAck);
	std::set<unsigned long > storedBundle;
	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
		storedBundle.insert((*it)->getSerial());
	}
	netwPkt->setH2hAcks(storedBundle);
	int nbrEntries = ackSerial.size()+ storedBundle.size();
	long helloControlBitLength = sizeof(unsigned long) * (nbrEntries) *8;
	int length = helloControlBitLength+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
		coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt,helloControlBitLength, 0, 0);
}

void EpidemicNetwLayer::handleHelloMsg(GeoDtnNetwPkt *netwPkt)
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
		/*************************** Sending Bundle Msg **********/
		sendingBndlResponseMsg(netwPkt->getSrcAddr(), storedBundle);
	}
}

void EpidemicNetwLayer::sendingBndlResponseMsg(LAddress::L3Type nodeAddr, std::set<unsigned long > wsmResponseBndl)
{
	std::set<unsigned long> serialResponseBndl;
	for (std::set<unsigned long>::iterator it = wsmResponseBndl.begin(); it != wsmResponseBndl.end(); it++){
		unsigned long serial = *it;
		if ((ackSerial.count(serial) == 1) || (exist(serial))){
			continue;
		}else{
			serialResponseBndl.insert(serial);
		}
	}

	if (!serialResponseBndl.empty()){
		GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
		prepareNetwPkt(netwPkt, Bundle_Response, nodeAddr);
		netwPkt->setH2hAcks(serialResponseBndl);
		long otherControlBitLength = sizeof(unsigned long) * (serialResponseBndl.size()) *8;
		int length = otherControlBitLength + netwPkt->getBitLength();
		netwPkt->setBitLength(length);
		sendDown(netwPkt, 0, otherControlBitLength, 0);
	}
}

void EpidemicNetwLayer::handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();

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
	// step 3 : Filtering bundle to send

	// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
	std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(unsortedWSMPair, netwPkt->getSrcAddr(), netwPkt->getSrcType());

	// step 4 : Sending bundles
	sendingBundleMsg(netwPkt->getSrcAddr(),netwPkt->getSrcType(),sentWSM);
}

void EpidemicNetwLayer::sendingBundleMsg(LAddress::L3Type destAddr, int destType, std::vector<WaveShortMessage* >  wsmToSend)
{
	for (std::vector<WaveShortMessage* >::iterator it = wsmToSend.begin(); it != wsmToSend.end(); it++){
		WaveShortMessage* wsm = *it;
		unsigned long serial = wsm->getSerial();
		GeoDtnNetwPkt* bundleMsg = new GeoDtnNetwPkt();
		prepareNetwPkt(bundleMsg, Bundle, destAddr);
		bundleMsg->encapsulate(wsm->dup());
		sendDown(bundleMsg, 0, 0, 1);
		if (destType == Veh){
			bundlesReplicaIndex[serial]++;
		}
	}
}

void EpidemicNetwLayer::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
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
			sendingBundleAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM);
		}
	}
}

void EpidemicNetwLayer::sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long > wsmFinalDeliverd)
{
	if (withAck){
		GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
		prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);
		std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
		netwPkt->setE2eAcks(serialOfE2EAck);
		long otherControlBitLength = sizeof(unsigned long) * serialOfE2EAck.size() *8;
		int length = otherControlBitLength + netwPkt->getBitLength();
		netwPkt->setBitLength(length);
		sendDown(netwPkt, 0, otherControlBitLength, 0);
	}
}

void EpidemicNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	if (withAck){
		std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
		storeAckSerials(finalDelivredToBndl);
	}
}

////////////////////////////////////////// Others methods /////////////////////////
