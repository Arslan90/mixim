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
			case INIT:
				if (netwPkt->getDestAddr() == myNetwAddr){
					handleInitMsg(netwPkt);
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

void EpidemicNetwLayer::finish()
{
	recordAllScalars();
}

void EpidemicNetwLayer::sendingHelloMsg()
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt,0, 0, 0);
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
		/*************************** Sending Bundle Init Msg **********/
	    sendingInitMsg(netwPkt->getSrcAddr());
	}
}

void EpidemicNetwLayer::sendingInitMsg(LAddress::L3Type nodeAddr)
{
	std::map<unsigned long, double > ackSerialsWithExpTime = getUnStoredAcksForSession(nodeAddr, ackModule.getAckSerialsWithExpTime());
	std::set<unsigned long > storedBundle = getUnStoredBndlForSession(nodeAddr, bndlModule.getBundleSerialsAsSet());

	if (! (ackSerialsWithExpTime.empty() && storedBundle.empty())){
		GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
		prepareNetwPkt(netwPkt, INIT, nodeAddr);

		netwPkt->setAckSerialsWithTimestamp(ackSerialsWithExpTime);
		netwPkt->setH2hAcks(storedBundle);

		long helloControlBitLength = estimateInBitsCtrlSize(true, &storedBundle, &ackSerialsWithExpTime, NULL, NULL);
		netwPkt->addBitLength(helloControlBitLength);
		coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
		sendDown(netwPkt,helloControlBitLength, 0, 0);
	}
}

void EpidemicNetwLayer::handleInitMsg(GeoDtnNetwPkt *netwPkt)
{
	/*************************** Handling Init Msg **********/
	std::map<unsigned long, double > receivedAckSerials = netwPkt->getAckSerialsWithTimestamp();
	if (!receivedAckSerials.empty()){
		updateStoredAcksForSession(netwPkt->getSrcAddr(),receivedAckSerials);
		storeNAckSerial(receivedAckSerials);
	}
	std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
	if (!storedBundle.empty()){
		updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
	}
	/*************************** Sending Bundle Msg **********/
	sendingBndlResponseMsg(netwPkt->getSrcAddr(), storedBundle);
}

void EpidemicNetwLayer::sendingBndlResponseMsg(LAddress::L3Type nodeAddr, std::set<unsigned long > wsmResponseBndl)
{
	std::set<unsigned long> serialResponseBndl;
	for (std::set<unsigned long>::iterator it = wsmResponseBndl.begin(); it != wsmResponseBndl.end(); it++){
		unsigned long serial = *it;
		if ((ackModule.existAck(serial)) || (bndlModule.existBundle(serial))){
			continue;
		}else{
			serialResponseBndl.insert(serial);
		}
	}

	if (!serialResponseBndl.empty()){
		GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
		prepareNetwPkt(netwPkt, Bundle_Response, nodeAddr);
		netwPkt->setH2hAcks(serialResponseBndl);
		long otherControlBitLength = estimateInBitsCtrlSize(false, &serialResponseBndl, NULL, NULL, NULL);
		netwPkt->addBitLength(otherControlBitLength);
		sendDown(netwPkt, 0, otherControlBitLength, 0);
	}
}

void EpidemicNetwLayer::handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();

	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
	unsortedWSMPair = bndlModule.getStoredBundlesWithReplica(serialResponseBndl);

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
		emit(sentL3SignalId,1);
		if (destType == Veh){
			bndlModule.updateSentReplica(serial);
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

		if (withCtrlForSectorReAddr){
			wsm->setRecipientAddress(newSectorAddr);
		}

		std::set<unsigned long> finalReceivedWSM;

		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			finalReceivedWSM.insert(wsm->getSerial());
			bundlesReceived++;
			emit(receiveL3SignalId,bundlesReceived);
			gen1AckSerial(wsm);
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			if ((!bndlModule.existBundle(wsm->getSerial())) && (ackModule.existAck(wsm->getSerial()) == 0)){
				bndlModule.storeBundle(wsm);
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

		std::map<unsigned long, double > ackSerialsWithExpTime = ackModule.getAckSerialsWithExpTime(wsmFinalDeliverd);
		netwPkt->setAckSerialsWithTimestamp(ackSerialsWithExpTime);

		long otherControlBitLength = estimateInBitsCtrlSize(false, NULL, &ackSerialsWithExpTime, NULL, NULL);
		netwPkt->addBitLength(otherControlBitLength);
		sendDown(netwPkt, 0, otherControlBitLength, 0);
	}
}

void EpidemicNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	if (withAck){
		std::map<unsigned long, double > finalDelivredToBndl = netwPkt->getAckSerialsWithTimestamp();
		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
		storeNAckSerial(finalDelivredToBndl);
	}
}

////////////////////////////////////////// Others methods /////////////////////////
