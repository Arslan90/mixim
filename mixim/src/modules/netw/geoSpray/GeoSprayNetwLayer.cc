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
		recomputeMyNetwRoute = false;

		nbrReplica = par("nbrReplica");
		bndlModule.reInitWithLimitedReplica(nbrReplica);

        withExplicitE2EAck = par("withExplicitE2EAck").boolValue();

        withExplicitH2HAck = par("withExplicitH2HAck").boolValue();
	}
}

void GeoSprayNetwLayer::handleLowerMsg(cMessage *msg)
{
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    GeoDtnNetwPkt *netwPkt = check_and_cast<GeoDtnNetwPkt *>(m->decapsulate());

    coreEV << "Receiving GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Addressed to " << netwPkt->getDestAddr() << " by current node " << myNetwAddr << std::endl;

   	if (isEquiped){
		switch (netwPkt->getKind()) {
			case HELLO:
				handleHelloMsg(netwPkt);
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

void GeoSprayNetwLayer::handleSelfMsg(cMessage *msg)
{
	if (msg == heartBeatMsg){
		myNetwRoute = NetwRoute(myNetwAddr,getCurrentMETD(),maxDbl, simTime(), true, nodeType, getCurrentPos());
		DtnNetwLayer::handleSelfMsg(msg);
	}
	if (msg == updateMsg){
		DtnNetwLayer::handleSelfMsg(msg);
	}
}

void GeoSprayNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	bndlModule.storeBundle(upper_msg, nbrReplica);
}

void GeoSprayNetwLayer::finish()
{
	recordScalar("# DeletedBundlesWithCustody", bndlModule.getNbrDeletedBundlesByCustody());
	recordScalar("# DeletedBundlesDueToNRR", bndlModule.getNbrDeletedBundlesByNoRmgReplica());

	recordAllScalars();
}

void GeoSprayNetwLayer::sendingHelloMsg()
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, HELLO, LAddress::L3BROADCAST);

	std::map<unsigned long, double > ackSerialsWithExpTime = ackModule.getAckSerialsWithExpTime();
	netwPkt->setAckSerialsWithTimestamp(ackSerialsWithExpTime);

	long helloControlBitLength = estimateInBitsCtrlSize(true, NULL, &ackSerialsWithExpTime, NULL, NULL);
	netwPkt->addBitLength(helloControlBitLength);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt,helloControlBitLength, 0, 0);
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

		std::map<unsigned long, double > receivedAckSerials = netwPkt->getAckSerialsWithTimestamp();
		if (!receivedAckSerials.empty()){
			updateStoredAcksForSession(netwPkt->getSrcAddr(),receivedAckSerials);
			storeNAckSerial(receivedAckSerials);
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
	std::set<unsigned long> serialOfH2hAck = bndlModule.getBundleSerialsAsSet();

	netwPkt->setH2hAcks(serialOfH2hAck);

	long otherControlBitLength = estimateInBitsCtrlSize(false, &serialOfH2hAck, NULL, NULL, NULL);
	netwPkt->addBitLength(otherControlBitLength);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}

void GeoSprayNetwLayer::handleBundleOfferMsg(GeoDtnNetwPkt *netwPkt)
{
	/*************************** E2E Acks **********/
    // We don't receive any E2E Acks here
    /*************************** H2H Acks (stored bundles) **********/
	std::set<unsigned long> serialStoredBndl;
	std::set<unsigned long> serialResponseBndl;

	serialStoredBndl = netwPkt->getH2hAcks();
    if (!serialStoredBndl.empty()){
    	updateStoredBndlForSession(netwPkt->getSrcAddr(), serialStoredBndl);
    }

	for (std::set<unsigned long>::iterator it = serialStoredBndl.begin(); it != serialStoredBndl.end(); it++){
		unsigned long serial = *it;
		if ((ackModule.existAck(serial)) || (bndlModule.existBundle(serial))){
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
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Response, destAddr);
	netwPkt->setH2hAcks(wsmResponseBndl);
	netwPkt->setSrcMETD(getCurrentMETD());
	long otherControlBitLength = estimateInBitsCtrlSize(false, &wsmResponseBndl, NULL, NULL, NULL);
	netwPkt->addBitLength(otherControlBitLength);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}

void GeoSprayNetwLayer::handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt)
{
	std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();
	
	// step 1 : Build bundle list to send before reordering
	if (getCurrentMETD() > netwPkt->getSrcMETD()){
		std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
		unsortedWSMPair = bndlModule.getStoredBundlesWithReplica(serialResponseBndl);

		// step 2 : Reordering bundle list
		// step 3 : Filtering bundle to send

		// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
		std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(unsortedWSMPair, netwPkt->getSrcAddr(), netwPkt->getSrcType());

		// step 4 : Sending bundles
		sendingBundleMsg(netwPkt->getSrcAddr(), sentWSM);
	}
}

void GeoSprayNetwLayer::sendingBundleMsg(LAddress::L3Type destAddr, std::vector<WaveShortMessage* >  wsmToSend)
{
	for (std::vector<WaveShortMessage* >::iterator it = wsmToSend.begin(); it != wsmToSend.end(); it++){
		WaveShortMessage* wsm = *it;
		unsigned long serial = wsm->getSerial();

		// step 1 : Fixing amount of replica to send and decide whether or not perform custody transfer
		// Fixing the number of replica to send

		bool custodyTransfert = false;
		int nbrReplicaToSend  = bndlModule.computeNbrReplicaToSend(serial);

		// Decide whether we send bundles or not based on Nbr of Replica to Send
		if (nbrReplicaToSend == 0){
			continue;
		}else{
			if (nbrReplicaToSend == 1){
				custodyTransfert = true;
			}

			// step 2 : sending Bundles
			GeoDtnNetwPkt *netwPkt = new GeoDtnNetwPkt();
			prepareNetwPkt(netwPkt, Bundle, destAddr);
			netwPkt->setNbrReplica(nbrReplicaToSend);
			netwPkt->setCustodyTransfert(custodyTransfert);
			netwPkt->encapsulate(wsm->dup());
			sendDown(netwPkt, 0, 0, 1);
			emit(sentL3SignalId,1);
			if (!withExplicitH2HAck){
				if (custodyTransfert){
					bndlModule.deleteBundleUponCustody(serial);
				}
				bndlModule.updateSentReplica(serial, nbrReplicaToSend);

			}
		}
	}
}

void GeoSprayNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;
	unsortedWSMPair = bndlModule.getStoredBundlesWithReplica();

	// step 2 : Reordering bundle list
	// step 3 : Filtering bundle to send

	// These steps are now achieved by a unique function implemented in DtnNetwLayer.cc
	std::vector<WaveShortMessage* > sentWSM = scheduleFilterBundles(unsortedWSMPair, vpaAddr, VPA);

	// step 4 : Sending bundles
	std::set<unsigned long > serialsToDelete;
	for (std::vector<WaveShortMessage* >::iterator it = sentWSM.begin(); it != sentWSM.end(); it++){
		WaveShortMessage* wsm = *it;
		GeoDtnNetwPkt *bundleMsg = new GeoDtnNetwPkt();
		prepareNetwPkt(bundleMsg, Bundle, vpaAddr);
		bundleMsg->encapsulate(wsm->dup());
		sendDown(bundleMsg, 0, 0, 1);
		emit(sentL3SignalId,1);
		if(!withExplicitE2EAck){
			gen1AckSerial(wsm);
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

		if (withCtrlForSectorReAddr){
			wsm->setRecipientAddress(newSectorAddr);
		}

		std::set<unsigned long> finalReceivedWSM;

		std::set<unsigned long> receivedWSM;

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
			if (! (bndlModule.existBundle(wsm->getSerial()) || ackModule.existAck(wsm->getSerial())) ){
				bndlModule.storeBundle(wsm, netwPkt->getNbrReplica());
				receivedWSM.insert(wsm->getSerial());
				bundlesReceived++;
				emit(receiveL3SignalId,bundlesReceived);
			}
		}
		if (!finalReceivedWSM.empty() && withExplicitE2EAck){
			sendingBundleE2EAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM);
		}

		if (!receivedWSM.empty()  && withExplicitH2HAck){
			sendingBundleH2HAckMsg(netwPkt->getSrcAddr(), receivedWSM, netwPkt->getNbrReplica(), netwPkt->getCustodyTransfert());
		}
	}
}

void GeoSprayNetwLayer::sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmFinalDeliverd)
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

void GeoSprayNetwLayer::sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDeliverd, int nbrReplica, bool custodyTransfer)
{
	GeoDtnNetwPkt* netwPkt = new GeoDtnNetwPkt();
	prepareNetwPkt(netwPkt, Bundle_Ack, destAddr);

	std::set<unsigned long> serialOfH2HAck = std::set<unsigned long>(wsmDeliverd);
	netwPkt->setH2hAcks(serialOfH2HAck);
	netwPkt->setNbrReplica(nbrReplica);
	netwPkt->setCustodyTransfert(custodyTransfer);

	long otherControlBitLength = estimateInBitsCtrlSize(false, NULL, NULL, NULL, &serialOfH2HAck);
	netwPkt->addBitLength(otherControlBitLength);
	sendDown(netwPkt, 0, otherControlBitLength, 0);
}

void GeoSprayNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
	if (withAck && withExplicitE2EAck){
		std::map<unsigned long, double > finalDelivredToBndl = netwPkt->getAckSerialsWithTimestamp();
		updateStoredAcksForSession(netwPkt->getSrcAddr(),finalDelivredToBndl);
		storeNAckSerial(finalDelivredToBndl);
	}

	if (withExplicitH2HAck){
		std::set<unsigned long> delivredToBndl = netwPkt->getH2hAcks();
		if (delivredToBndl.size()>1){
			opp_warning("GeoSprayNetwLayer::handleBundleAckMsg - Unsupported handling of more then a single H2HAck");
		}
		for (std::set<unsigned long >::iterator it = delivredToBndl.begin(); it != delivredToBndl.end(); it++){
			if (bndlModule.existBundle(*it)){
				if (netwPkt->getCustodyTransfert() ){
					bndlModule.deleteBundleUponCustody(*it);
				}
				bndlModule.updateSentReplica(*it, netwPkt->getNbrReplica());
			}
		}
	}
}

////////////////////////////////////////// Others methods /////////////////////////

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

double GeoSprayNetwLayer::getCurrentMETD()
{
	double currentMETD;
	if (nodeType == Veh){
		currentMETD = getGeoTraci()->getCurrentMetd();
	}else if (nodeType == VPA){
		currentMETD = 0.0;
	}else {
		opp_error("Undefined NodeType");
	}
	return currentMETD;
}
