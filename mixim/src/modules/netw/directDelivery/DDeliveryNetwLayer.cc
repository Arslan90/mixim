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
		// Nothing to do
	}
}

void DDeliveryNetwLayer::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    GeoDtnNetwPkt *netwPkt = check_and_cast<GeoDtnNetwPkt *>(m->decapsulate());

    coreEV << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Addressed to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

   	if (isEquiped){
		switch (netwPkt->getKind()) {
			case HELLO:
				handleHelloMsg(netwPkt);
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

void DDeliveryNetwLayer::finish()
{
	recordAllScalars();
}

void DDeliveryNetwLayer::sendingHelloMsg()
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt,0, 0, 0);
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
				sendingBundleMsg(netwPkt->getSrcAddr(),netwPkt->getSrcType());
			}
		}
	}
}

void DDeliveryNetwLayer::sendingBundleMsg(LAddress::L3Type destAddr, int destType)
{
	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
//	for (std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.begin(); it != bundlesReplicaIndex.end(); it++){
//		unsigned long serial = it->first;
//		if (exist(serial)){
//			for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
//				if ((*it3)->getSerial() == serial){
//					unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it->second));
//					break;
//				}
//			}
//		}
//	}
	unsortedWSMPair = bndlModule.getStoredBundlesWithReplica();

	// step 2 : Reordering bundle list
	// step 3 : Filtering bundle to send

	// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
	std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(unsortedWSMPair, destAddr, destType);

	// step 4 : Sending bundles
	for (std::vector<WaveShortMessage* >::iterator it = sentWSM.begin(); it != sentWSM.end(); it++){
		WaveShortMessage* wsm = *it;
		unsigned long serial = wsm->getSerial();
		GeoDtnNetwPkt* bundleMsg = new GeoDtnNetwPkt();
		prepareNetwPkt(bundleMsg, Bundle, destAddr);
		bundleMsg->encapsulate(wsm->dup());
		sendDown(bundleMsg, 0, 0, 1);
		emit(sentL3SignalId,1);
		bndlModule.updateSentReplica(serial);
//		if (destType == Veh){
//			bundlesReplicaIndex[serial]++;
//		}
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
			//			storeAckSerial(wsm->getSerial());
			//			if (withTTLForCtrl){
			//				emitSignalForAckLifeTime(wsm->getSerial(), simTime().dbl(), ttlForCtrl+simTime().dbl());
			//			}
			gen1AckSerial(wsm);
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			opp_error("DDeliveryNetwLayer::handleBundleMsg() - Reception of bundle by not recipient address not allowed");
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
//	std::set<unsigned long> serialOfE2EAck = std::set<unsigned long>(wsmFinalDeliverd);
//	netwPkt->setE2eAcks(serialOfE2EAck);
//	long sizeOC_SA_Octets = sizeof(unsigned long) * serialOfE2EAck.size();

	std::map<unsigned long, double > ackSerialsWithExpTime = ackModule.getAckSerialsWithExpTime(wsmFinalDeliverd);
	netwPkt->setAckSerialsWithTimestamp(ackSerialsWithExpTime);

//	long sizeOC_SA_Octets = 0;
//	if (withTTLForAck){
//		sizeOC_SA_Octets = (sizeof(unsigned long) + sizeof(double)) * ackSerialsWithExpTime.size();
//	}else{
//		sizeOC_SA_Octets = (sizeof(unsigned long)) * ackSerialsWithExpTime.size();
//	}
//	emitSignalForOtherCtrlMsg(0, sizeOC_SA_Octets, 0, 0);
//
//	long otherControlBitLength = sizeOC_SA_Octets * 8;

	long otherControlBitLength = estimateInBitsCtrlSize(false, NULL, &ackSerialsWithExpTime, NULL, NULL);
	netwPkt->addBitLength(otherControlBitLength);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}

void DDeliveryNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
//	std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
//	updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
	//	storeAckSerials(finalDelivredToBndl);
	//	if (withTTLForCtrl){
	//		for (std::set<unsigned long>::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
	//			emitSignalForAckLifeTime((*it), -1, ttlForCtrl+simTime().dbl());
	//		}
	//	}

	std::map<unsigned long, double > finalDelivredToBndl = netwPkt->getAckSerialsWithTimestamp();
	updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
	storeNAckSerial(finalDelivredToBndl);
}

////////////////////////////////////////// Others methods /////////////////////////
