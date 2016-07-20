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

Define_Module(GeoSprayNetwLayer);

void GeoSprayNetwLayer::initialize(int stage)
{
    // TODO - Generated method body
	DtnNetwLayer::initialize(stage);
	if (stage == 0){
		DefineNodeType();
		netwRouteExpirency = par("netwRouteExpirency").doubleValue();
		netwRoutePending = par("netwRoutePending").doubleValue();
		heartBeatMsgPeriod = par("heartBeatMsgPeriod").doubleValue();
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
	double currentMETD, currentDist;
	if (msg == heartBeatMsg){
		if (nodeType == Veh){
			currentMETD = geoTraci->getCurrentMetd();
		}else if (nodeType == VPA){
			currentMETD = 0.0;
		}else {
			opp_error("Undefined NodeType");
		}
		if (nodeType == Veh){
			sectorId = geoTraci->getCurrentSector();
		}
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,currentMETD,0, simTime(), true, nodeType));
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
	if (nodeType == Veh){
		sectorId = geoTraci->getCurrentSector();
		netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
	}else if (nodeType == VPA){
		netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId ,LAddress::L3BROADCAST);
	}else {
		opp_error("Undefined NodeType");
	}
//	cout << "Sending Hello packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleHelloMsg(GeoDtnNetwPkt *netwPkt)
{
	// If not the same sector ignore message
	if (netwPkt->getVpaSectorId() != sectorId){
		return;
	}else{
		/*************************** Handling Hello Msg **********/
	    NetwRoute neighborEntry = NetwRoute(netwPkt->getSrcAddr(), 0, 0, simTime() , true, netwPkt->getSrcType());
	    updateNeighborhoodTable(netwPkt->getSrcAddr(), neighborEntry);

//	    cout << "Receiving Hello packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;

//	    if (netwPkt->getSrcType() == VPA){
//	    	sendingBundleMsgToVPA(netwPkt->getSrcAddr());
//	    }else{
		    GeoDtnNetwPkt* bundleAckMsg;
		    std::list<unsigned long> wsmFinalDeliverd;
		    sendingBundleAckMsg(bundleAckMsg, netwPkt->getSrcAddr(), wsmFinalDeliverd);

		    if (nodeType == Veh){
			    GeoDtnNetwPkt* bundleOfferMsg;
			    std::list<unsigned long> wsmStoredBndl;
			    sendingBundleOfferMsg(bundleOfferMsg, netwPkt->getSrcAddr(), wsmStoredBndl);
		    }
//	    }
	}
}

void GeoSprayNetwLayer::sendingBundleAckMsg(GeoDtnNetwPkt *netwPkt, LAddress::L3Type destAddr, std::list<unsigned long> wsmFinalDeliverd)
{
	netwPkt = prepareNetwPkt(Bundle_Ack,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);

	std::set<unsigned long> serialOfE2EAck;
//	for (std::list<unsigned long >::iterator it = wsmFinalDeliverd.begin(); it != wsmFinalDeliverd.end(); it++){
	for (std::set<unsigned long >::iterator it = ackSerial.begin(); it != ackSerial.end(); it++){
		serialOfE2EAck.insert(*it);
	}
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * serialOfE2EAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
//	cout << "Sending BundleAck packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleBundleAckMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving BundleAck packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;

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
	}

	if (netwPkt->getSrcType() == VPA){
		sendingBundleMsgToVPA(netwPkt->getSrcAddr());
	}
}

void GeoSprayNetwLayer::sendingBundleOfferMsg(GeoDtnNetwPkt *netwPkt, LAddress::L3Type destAddr, std::list<unsigned long> wsmStoredBndl)
{
	netwPkt = prepareNetwPkt(Bundle_Offer,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);

	std::set<unsigned long> serialOfH2hAck;
	for (std::list<WaveShortMessage* >::iterator it = bundles.begin(); it != bundles.end(); it++){
//		WaveShortMessage* wsm = it;
		serialOfH2hAck.insert((*it)->getSerial());
	}

	netwPkt->setH2hAcks(serialOfH2hAck);
	int length = sizeof(unsigned long) * serialOfH2hAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
//	cout << "Sending BundleOffer packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleBundleOfferMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving BundleOffer spacket from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;

	std::set<unsigned long> serialStoredBndl;
	std::set<unsigned long> serialResponseBndl;

	serialStoredBndl = netwPkt->getH2hAcks();
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

void GeoSprayNetwLayer::sendingBundleResponseMsg(GeoDtnNetwPkt *netwPkt, LAddress::L3Type destAddr, std::set<unsigned long> wsmResponseBndl)
{
	double currentMETD;
	if (nodeType == Veh){
		currentMETD = geoTraci->getCurrentMetd();
	}else if (nodeType == VPA){
		currentMETD = 0.0;
	}else {
		opp_error("Undefined NodeType");
	}
	netwPkt = prepareNetwPkt(Bundle_Response,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
	netwPkt->setH2hAcks(wsmResponseBndl);
	netwPkt->setSrcMETD(currentMETD);

//	cout << "Sending BundleResponse packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving BundleResponse packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;

	if (nodeType == Veh){
		double currentMETD = geoTraci->getCurrentMetd();
		if (currentMETD > netwPkt->getSrcMETD()){
			std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();

			// step 1 : Build bundle list to send before reordering
			std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair;
			for (std::set<unsigned long>::iterator it = serialResponseBndl.begin(); it != serialResponseBndl.end(); it++){
				unsigned long serial = *it;
				if (exist(serial)){
					if (withSentTrack){
						std::map<LAddress::L3Type, NetwSession>::iterator it4 = neighborhoodSession.find(netwPkt->getSrcAddr());
						if (it4 != neighborhoodSession.end()){
							NetwSession newSession = it4->second;
							if (newSession.getDelivredToBndl().count(serial)) {continue;}
						}
					}
					std::map<unsigned long, int>::iterator it2 = bundlesRmgReplica.find(serial);
					if (it2 == bundlesRmgReplica.end()){
						opp_error("Bundle Found but not in rmg replica index");
					}else{
						if (it2->second < 0) {opp_error("Remaining Bundle replica is negative");}
						if (it2->second == 0) {continue;}
						else{
							for (std::list<WaveShortMessage*>::iterator it3 = bundles.begin(); it3 != bundles.end(); it3++){
								if ((*it3)->getSerial() == serial){
									sortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((*it3), it2->second));
									break;
								}
							}
						}
					}
				}
			}

			// step 2 : Reordering bundle list
			std::sort(sortedWSMPair.begin(), sortedWSMPair.end(), comparatorRCAscObject);

			// step 3 : Sending bundles with NbrReplica to transfer
			for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = sortedWSMPair.begin(); it != sortedWSMPair.end(); it++){
				unsigned long serial = (it->first)->getSerial();
				std::map<unsigned long, int>::iterator it2 = bundlesRmgReplica.find(serial);
				if (it2 == bundlesRmgReplica.end()){
					opp_error("Remaining Replica not found");
				}
				bool custodyTransfert = false;

				int remainingCopy = it2->second;
				int nbrReplicaToSend = 0;
				if ((remainingCopy <= 0 ) || (remainingCopy > nbrReplica )){
					opp_error("Invalid remaining replica");
				}else if (remainingCopy == 1 ){
					nbrReplicaToSend = 1;
					custodyTransfert = true;
				}else{
					nbrReplicaToSend =  remainingCopy / 2;
				}
				GeoDtnNetwPkt* bundleMsg;
				sendingBundleMsg(bundleMsg, netwPkt->getSrcAddr(), (it->first)->dup(), nbrReplicaToSend);
				bundlesReplicaIndex[serial] = bundlesReplicaIndex[serial] + nbrReplicaToSend;
				bundlesRmgReplica[serial] = bundlesRmgReplica[serial] - nbrReplicaToSend;
				if (custodyTransfert){
					erase(serial);
				}

				if (withSentTrack){
					std::map<LAddress::L3Type, NetwSession>::iterator it3 = neighborhoodSession.find(netwPkt->getSrcAddr());
					if (it3 == neighborhoodSession.end()){
						NetwSession newSession = NetwSession(netwPkt->getSrcAddr(),0);
						newSession.insertInDelivredToBndl(serial);
						neighborhoodSession.insert(std::pair<LAddress::L3Type, NetwSession>(netwPkt->getSrcAddr(), newSession));
					}else{
						NetwSession newSession = it3->second;
						newSession.insertInDelivredToBndl(serial);
						neighborhoodSession[netwPkt->getSrcAddr()] = newSession;
					}
				}
			}
		}
	}
}

void GeoSprayNetwLayer::sendingBundleMsg(GeoDtnNetwPkt *netwPkt, LAddress::L3Type destAddr, WaveShortMessage* wsm,  int nbrReplica)
{
	netwPkt = prepareNetwPkt(Bundle,myNetwAddr, nodeType, destAddr, sectorId ,LAddress::L3BROADCAST);
	netwPkt->encapsulate(wsm);
	netwPkt->setNbrReplica(nbrReplica);
	cout << "Bundle Pkt " << wsm->getSerial() << " send from  " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << " nbr replica transfered " << nbrReplica << std::endl;

//	cout << "Sending Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void GeoSprayNetwLayer::handleBundleMsg(GeoDtnNetwPkt *netwPkt)
{
//	cout << "Receiving Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);
		totalBundlesReceived++;
		if (wsm->getRecipientAddress() == myNetwAddr){
			netwPkt->encapsulate(wsm);
			sendUp(netwPkt->dup());
			bundlesReceived++;
			storeAckSerial(wsm->getSerial());
			missedOpportunities.erase(wsm->getSerial());
		}else {
			/*
			 * Process to avoid storing twice the same msg
			 */
			if ((!exist(wsm->getSerial())) && (ackSerial.count(wsm->getSerial()) == 0)){
				storeBundle(wsm);
				std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(wsm->getSerial());
				if (it == bundlesReplicaIndex.end()){
					bundlesReplicaIndex.insert(std::pair<unsigned long, int>(wsm->getSerial(), 0));
					bundlesRmgReplica.insert(std::pair<unsigned long, int>(wsm->getSerial(), netwPkt->getNbrReplica()));
				}
				bundlesReceived++;
				missedOpportunities.erase(wsm->getSerial());
			}
		}
	}
}

void GeoSprayNetwLayer::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{
	// step 1 : check if we have any bundle that are addressed to @vpaAddr
	std::vector<WaveShortMessage* > wsmToSend;
	bundlesIndexIterator it = bundlesIndex.find(vpaAddr);
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		for (innerIndexIterator it2 = innerMap.begin(); it2 !=innerMap.end(); it2++){
			WaveShortMessage* wsm = it2->second;
			if (wsm != NULL){
				wsmToSend.push_back(wsm);
			}
		}
	}

	// step 2 : send bundles
	for (std::vector<WaveShortMessage*>::iterator it = wsmToSend.begin(); it!= wsmToSend.end(); it++){
		WaveShortMessage* wsm = *it;
		GeoDtnNetwPkt* bundleForVPA;
		bundleForVPA = prepareNetwPkt(Bundle,myNetwAddr, nodeType, vpaAddr, sectorId ,LAddress::L3BROADCAST);
		bundleForVPA->encapsulate(wsm->dup());
		cout << "Sending Bundle packet from " << bundleForVPA->getSrcAddr() << " addressed to VPA " << bundleForVPA->getDestAddr() << std::endl;
		sendDown(bundleForVPA);

		unsigned long serial = wsm->getSerial();
		storeAckSerial(serial);
		erase(serial);
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

void GeoSprayNetwLayer::DefineNodeType()
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
			opp_error("GeoSprayNetwLayer::DefineNodeType() - Unable to define NodeType please check existence of appl module in NED file");
		}
	}
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
