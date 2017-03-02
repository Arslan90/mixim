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

#include "ProphetNCV2.h"
#include "multiFunctions.h"
#include "ApplOppControlInfo.h"
#include "NetwOppControlInfo.h"
#include "FindModule.h"
#include "TraCIMobility.h"
#include <iomanip>
#include "Coord.h"
#include "simtime.h"

Define_Module(ProphetNCV2);

void ProphetNCV2::initialize(int stage)
{
	DtnNetwLayer::initialize(stage);
	if (stage==0){
		/*
		 * Initialization of ProphetV2 parameters
		 */
		PEncMax = par("PEncMax").doubleValue();
		PFirstContact = par("PFirstContact").doubleValue();
		PMinThreshold = par("PMinThreshold").doubleValue();
		I_TYP = par("I_TYP").doubleValue();
		Beta = par("Beta").doubleValue();
		GAMMA = par("GAMMA").doubleValue();
		secondsInTimeUnit = par("secondsInTimeUnit");
		/*
		 * Initialization of the forward strategy &
		 * the queuing strategy
		 */
		int fwd = par("fwdStrategy");
		fwdStrategy = static_cast<t_prophet_forward>(fwd);
		int q = par("qStrategy");
		qStrategy = static_cast<t_prophet_queuing>(q);
		/*
		 * Initialization of the used structures
		 */
		lastAgeUpdate = 0;

		I_Preds = par("I_Preds").doubleValue();

		/*
		 * Collecting data & metrics
		 */

	    nbrPredsVector.setName("Number of predictions");
	    predsMean.setName("Mean of predictions");
	    predsMax.setName("Maximum of predictions");
	    predsMin.setName("Minimum of predictions");
	    predsVariance.setName("Variance of predictions");
	    predForVPA.setName("Pred for vpa");

	    nbrSuccessfulContact = 0;
        nbrFailedContactBeforeRIB = 0;
        nbrFailedContactAtRIB= 0;
        nbrFailedContactAtBundle_Offer = 0;
        nbrFailedContactAtBundle_Response = 0;
        contactState = std::map<LAddress::L3Type, Prophetv2MessageKinds>();

        initAllClassifier();

        maxForRC = 5;

        nbrContactsForRCVect.setName("Evolution of nbrContact between RC");

        withPartialUpdate = par("withPartialUpdate").boolValue();

        withEMethod = par("withEMethod").boolValue();

	}
	else if (stage==1){
		preds.insert(std::pair<LAddress::L3Type,double>(myNetwAddr,1));
	}else if (stage==2){
		sectorId = getCurrentSector();
	}
}

/*******************************************************************
**
** 							Methods related to predictions
**
********************************************************************/

void ProphetNCV2::updateDeliveryPredsFor(const LAddress::L3Type BAdress)
{
	if (BAdress == myNetwAddr){
		opp_warning("Cannot Update Delivery Predictions for its own Address(ProphetNCV2::updateDeliveryPredsFor)");
	}else{
		double PEnc,lastEncTime, predsForB;
			/*
			 * before calculating predsForB, we must age preds.
			 */
			ageDeliveryPreds();
			double encTime = simTime().dbl();
			predsIterator it= preds.find(BAdress);
			predsIterator it2= lastEncouterTime.find(BAdress);

			if (!withEMethod){
				if (it2==lastEncouterTime.end()){
					/*
					 * if iterator is equal map.end(), it means that this node had not been meet yet
					 * so lastEncTime equal 0 and therefore P equals PFirstContact
					 */
					predsForB = PFirstContact;
				}else {
					/*
					 * if iterator is not equal map.end(), it means that this node had been meet before
					 */
					if (it==preds.end()){
						predsForB = 0;
					}else {
						predsForB = it->second;
					}
					lastEncTime = it2->second;

					if (simTime().dbl()-lastEncTime<I_TYP){
						/*
						 * if the node has been encountered recently then don't use PEncMax
						 */
						PEnc = PEncMax * (( simTime().dbl()-lastEncTime)/I_TYP);
					}else {
						/*
						 * if the node has been encountered long ago then use PEncMax
						 */
						PEnc = PEncMax;
					}
					predsForB = predsForB + (1-predsForB)*PEnc;
				}
			}else{
				if (it==preds.end()){
					predsForB = PFirstContact;
					if (it2==lastEncouterTime.end()){
						lastEncTime = 0;
					}else {
						lastEncTime = it2->second;
					}
				}else {
					predsForB = it->second;
					if (it2==lastEncouterTime.end()){
						lastEncTime = 0;
					}else {
						lastEncTime = it2->second;
					}

					if (simTime().dbl()-lastEncTime<I_TYP){
						PEnc = PEncMax * (( simTime().dbl()-lastEncTime)/I_TYP);
					}else {
						PEnc = PEncMax;
					}
					predsForB = predsForB + (1-predsForB)*PEnc;
				}
			}

			preds[BAdress] = predsForB;
			if (BAdress == 10){
				predForVPA.record(predsForB);
			}
			lastEncouterTime[BAdress] = encTime;
			cout << "@: " << myNetwAddr << " encountered@: " <<  BAdress << " T: " << encTime << " Pred: " << predsForB << endl;
	}
}

void ProphetNCV2::updateTransitivePreds(const LAddress::L3Type BAdress, std::map<LAddress::L3Type,double> Bpreds)
{

	double predsForB,predsForC,BpredsForC;
	LAddress::L3Type CAdress;
	predsIterator tmp_it;
	/*
	 * before calculating transitive predictions, we must age preds.
	 */
	ageDeliveryPreds();
	for (predsIterator it=Bpreds.begin(); it!=Bpreds.end();it++){

		if ((it->first==myNetwAddr)||(it->first==BAdress)){
			continue;
		}

		CAdress = it->first;
		tmp_it= preds.find(CAdress);
		if (tmp_it == preds.end()){
			predsForC = 0;
		} else {
			predsForC=tmp_it->second;
		}

		tmp_it= Bpreds.find(CAdress);
		BpredsForC=tmp_it->second;

		tmp_it= preds.find(BAdress);
		predsForB=tmp_it->second;

		if (predsForC<(predsForB*BpredsForC*Beta)){
			predsForC = predsForB*BpredsForC*Beta;
		}

		preds[CAdress] = predsForC;
		if (CAdress ==10){
			cout << "@: " << myNetwAddr << " encountered@: " <<  BAdress << " T: " << simTime().dbl() << " Pred after aging: " << predsForB << "Pred for VPA:" << predsForC <<endl;
			predForVPA.record(predsForC);
		}
	}
}

void ProphetNCV2::ageDeliveryPreds()
{
	predsIterator it2;
	double time = simTime().dbl();
	int  timeDiff = int (time-lastAgeUpdate)/secondsInTimeUnit;
	if (timeDiff==0){
		return;
	}else {
		double mult = std::pow(GAMMA, timeDiff);
		std::vector<LAddress::L3Type> predsToDelete;

		for (predsIterator it=preds.begin();it!=preds.end();it++){

			if (it->first==myNetwAddr){
				continue;
			}

			it->second = it->second * mult;


			if (it->second < PMinThreshold){
				predsToDelete.push_back(it->first);
			}

			if (it->first ==10){
				predForVPA.record(it->second);
			}
		}

		for (std::vector<LAddress::L3Type>::iterator it2 = predsToDelete.begin(); it2 != predsToDelete.end(); it2++){
			LAddress::L3Type tmp = *it2;
			preds.erase(tmp);
		}

		lastAgeUpdate = simTime().dbl();
	}
}

void ProphetNCV2::update(ProphetNCPkt *prophetPkt)
{
	updateDeliveryPredsFor(prophetPkt->getSrcAddr());
	updateTransitivePreds(prophetPkt->getSrcAddr(),prophetPkt->getPreds());
	recordPredsStats();
}

void ProphetNCV2::partialUpdate(ProphetNCPkt *prophetPkt)
{
	updateTransitivePreds(prophetPkt->getSrcAddr(),prophetPkt->getPreds());
	recordPredsStats();
}

/*******************************************************************
**
** 							Core methods
**
********************************************************************/

void ProphetNCV2::handleLowerMsg(cMessage* msg)
{
	std::pair<SimpleContactStats, std::set<SimpleContactStats>::iterator > pair;
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;

    ProphetNCPkt *prophetPkt = check_and_cast<ProphetNCPkt *>(m->decapsulate());

    if (isEquiped){

		switch (prophetPkt->getKind()) {
			case HELLO:
				if ((prophetPkt->getDestAddr()==LAddress::L3BROADCAST)||(prophetPkt->getDestAddr()==myNetwAddr)){
					handleHelloMsg(prophetPkt);
				}
				break;
			case Bundle_Offer:
				if (prophetPkt->getDestAddr()==myNetwAddr){
					handleBndlOfferMsg(prophetPkt);
				}
				break;
			case Bundle_Response:
				if (prophetPkt->getDestAddr()==myNetwAddr){
					handleBndlRespMsg(prophetPkt);
				}
				break;
			case Bundle:
				if (prophetPkt->getDestAddr()==myNetwAddr){
					handleBundleMsg(prophetPkt);
				}
				break;
			case Bundle_Ack:
				if (prophetPkt->getDestAddr()==myNetwAddr){
					handleBundleAckMsg(prophetPkt);
				}
				break;
			default:
				opp_error("Unknown or unsupported Prophetv2MessageKinds when calling HandleLowerMsg()");
				break;
		}
    }


    updatingL3Received();

    delete prophetPkt;
    delete msg;
}

void ProphetNCV2::handleLowerControl(cMessage* msg)
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
			}
			break;
	}

	}
	delete msg;
}

void ProphetNCV2::handleSelfMsg(cMessage* msg)
{
	if (msg == heartBeatMsg){
		updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,maxDbl,maxDbl, simTime(), true, nodeType, getCurrentPos()));
		ProphetNCPkt* netwPkt;
		sendingHelloMsg(netwPkt);
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
}

void ProphetNCV2::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
	std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(upper_msg->getSerial());
	if (it == bundlesReplicaIndex.end()){
		bundlesReplicaIndex.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), 0));
	}
}

void ProphetNCV2::sendingHelloMsg(ProphetNCPkt *netwPkt)
{
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(HELLO,myNetwAddr, nodeType ,LAddress::L3BROADCAST, sectorId , getCurrentPos());
	std::map<LAddress::L3Type, double> predToSend = std::map<LAddress::L3Type, double>();
	ageDeliveryPreds();
	predToSend.insert(preds.begin(),preds.end());
	netwPkt->setPreds(predToSend);


//	int predsLength = (sizeof(int ) + sizeof(double)) * predToSend.size();
//	int length = predsLength + netwPkt->getBitLength();
//	netwPkt->setBitLength(length);
	coreEV << "Sending GeoDtnNetwPkt packet from " << netwPkt->getSrcAddr() << " Destinated to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void ProphetNCV2::handleHelloMsg(ProphetNCPkt *netwPkt)
{
	// If not the same sector ignore message
	if (netwPkt->getVpaSectorId() != sectorId){
		return;
	}else{
		/*************************** Handling Hello Msg **********/
//	    cout << "Receiving Hello packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << " current address " << myNetwAddr << std::endl;
	    NetwRoute neighborEntry = NetwRoute(netwPkt->getSrcAddr(), maxDbl, maxDbl, simTime() , true, netwPkt->getSrcType(), netwPkt->getCurrentPos());
	    updateNeighborhoodTable(netwPkt->getSrcAddr(), neighborEntry);
	    update(netwPkt);

	    /*************************** Sending BundleOffer Msg **********/
	    sendingBndlOfferMsg(netwPkt->getSrcAddr(),netwPkt->getPreds());
//	    if (nodeType == Veh){
//	    	sendingBndlOfferMsg(netwPkt->getSrcAddr(),netwPkt->getPreds());
//		}
	}
}

void ProphetNCV2::sendingBndlOfferMsg(LAddress::L3Type nodeAddr, std::map<LAddress::L3Type, double> predsOfNode)
{
//	/*************************** H2H Acks (stored bundles) **********/
//	std::set<unsigned long> serialOfH2hAck;
//	for (std::list<WaveShortMessage* >::iterator it = bundles.begin(); it != bundles.end(); it++){
//		unsigned long wsmSerial = (*it)->getSerial();
//		std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(nodeAddr);
//		if (it2 != neighborhoodSession.end()){
//			NetwSession newSession = it2->second;
//			if (newSession.getStoredBndl().count(wsmSerial) > 0){
//				continue;
//			}else if (newSession.getDelivredToBndl().count(wsmSerial) > 0){
//				continue;
//			}else if (newSession.getDelivredToVpaBndl().count(wsmSerial) > 0){
//				continue;
//			}
//		}
//		predsIterator myPred = preds.find((*it)->getRecipientAddress());
//		predsIterator otherPred = predsOfNode.find((*it)->getRecipientAddress());
//		bool addToOffer = false;
//		if (otherPred != predsOfNode.end()){
//			if (myPred != preds.end()){
//				if (myPred->second <= otherPred->second){
//					addToOffer = true;
//				}
//			}else{
//				addToOffer = true;
//			}
//		}
//		if (addToOffer){
//			serialOfH2hAck.insert(wsmSerial);
//		}
//	}
//	netwPkt->setH2hAcks(serialOfH2hAck);
//
//	/*************************** E2E Acks **********/
	std::set<unsigned long> storedAck;
	for (std::set<unsigned long >::iterator it = ackSerial.begin(); it != ackSerial.end(); it++){
		unsigned long wsmSerial = (*it);
//		std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(nodeAddr);
//		if (it2 != neighborhoodSession.end()){
//			NetwSession newSession = it2->second;
//			if (newSession.getDelivredToVpaBndl().count(wsmSerial) > 0){
//				continue;
//			}
//		}
		storedAck.insert(wsmSerial);
	}

//	/*************************** H2H Acks (stored bundles) **********/
	std::set<unsigned long > storedBundle;
	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
		LAddress::L3Type destAddr = (*it)->getRecipientAddress();
		if (destAddr == myNetwAddr){
			opp_error("storing bundles addressed to current node");
		}
		predsIterator myPred = preds.find((*it)->getRecipientAddress());
		predsIterator otherPred = predsOfNode.find((*it)->getRecipientAddress());
		bool addToOffer = false;
		if (otherPred != predsOfNode.end()){
			if (myPred != preds.end()){
				if (myPred->second < otherPred->second){
					addToOffer = true;
				}
			}else{
				addToOffer = true;
			}
		}
		if (addToOffer){
			unsigned long wsmSerial = (*it)->getSerial();
//			std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(nodeAddr);
//			if (it2 != neighborhoodSession.end()){
//				NetwSession newSession = it2->second;
//				if (newSession.getStoredBndl().count(wsmSerial) > 0){
//					continue;
//				}else if (newSession.getDelivredToBndl().count(wsmSerial) > 0){
//					continue;
//				}
//			}
			storedBundle.insert(wsmSerial);
		}
	}

	if (!(storedBundle.empty() & storedAck.empty())){
		// if at least one of the two sets is not empty
		ProphetNCPkt *netwPkt;
		sectorId = getCurrentSector();
		netwPkt = prepareNetwPkt(Bundle_Offer,myNetwAddr, nodeType ,nodeAddr, sectorId , getCurrentPos());
		netwPkt->setE2eAcks(storedAck);
		netwPkt->setH2hAcks(storedBundle);
		int nbrEntries = storedAck.size()+ storedBundle.size();
		int length = sizeof(unsigned long) * (nbrEntries)+ netwPkt->getBitLength();
		netwPkt->setBitLength(length);
		cout << "Sending BundleOffer packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
		sendDown(netwPkt);
	}
}

void ProphetNCV2::handleBndlOfferMsg(ProphetNCPkt *netwPkt)
{

    std::set<unsigned long> receivedE2eAcks = netwPkt->getE2eAcks();
    if (!receivedE2eAcks.empty()){
//    	updateStoredAcksForSession(netwPkt->getSrcAddr(), receivedE2eAcks);
    	storeAckSerial(receivedE2eAcks);
    }
    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
    if (!storedBundle.empty()){
//    	updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
    }
    /*************************** E2E Acks **********/
//	std::map<LAddress::L3Type, NetwSession>::iterator it2;
//	std::set<unsigned long> finalDelivredToBndl = netwPkt->getE2eAcks();
//	it2 = neighborhoodSession.find(netwPkt->getSrcAddr());
//	if (it2 == neighborhoodSession.end()){
//		NetwSession newSession = NetwSession(netwPkt->getSrcAddr(),0);
//		for (std::set<unsigned long >::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
//			newSession.insertInDelivredToVpaBndl(*it);
//		}
//		neighborhoodSession.insert(std::pair<LAddress::L3Type, NetwSession>(netwPkt->getSrcAddr(), newSession));
//	}else{
//		NetwSession newSession = it2->second;
//		for (std::set<unsigned long >::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
//			newSession.insertInDelivredToVpaBndl(*it);
//		}
//		neighborhoodSession[netwPkt->getSrcAddr()] = newSession;
//	}
//
//	for (std::set<unsigned long >::iterator it = finalDelivredToBndl.begin(); it != finalDelivredToBndl.end(); it++){
//		storeAckSerial(*it);
//		erase(*it);
//	}
	/*************************** H2H Acks (stored bundles) **********/
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

//	if (!serialStoredBndl.empty()){
//		updateStoredBndlForSession(netwPkt->getSrcAddr(), serialStoredBndl);
//	}
	if (!serialResponseBndl.empty()){
		sendingBndlRespMsg(netwPkt->getSrcAddr(), serialResponseBndl);
	}

}

void ProphetNCV2::sendingBndlRespMsg( LAddress::L3Type nodeAddr, std::set<unsigned long> wsmResponseBndl)
{
	ProphetNCPkt *netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle_Response,myNetwAddr, nodeType ,nodeAddr, sectorId , getCurrentPos());
	netwPkt->setH2hAcks(wsmResponseBndl);
	int length = sizeof(unsigned long) * (wsmResponseBndl.size())+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	cout << "Sending BundleResponse packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void ProphetNCV2::handleBndlRespMsg(ProphetNCPkt *netwPkt)
{
	std::set<unsigned long> serialResponseBndl = netwPkt->getH2hAcks();

	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair;
	for (std::set<unsigned long>::iterator it = serialResponseBndl.begin(); it != serialResponseBndl.end(); it++){
		unsigned long serial = *it;
		if (exist(serial)){
			std::map<unsigned long, int>::iterator it2 = bundlesReplicaIndex.find(serial);
			if (it2 == bundlesReplicaIndex.end()){
				opp_error("Bundle Found but not in rmg replica index");
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

	// step 2 : Reordering bundle list
	std::sort(sortedWSMPair.begin(), sortedWSMPair.end(), comparatorRCAscObject);

	// step 3 : Sending bundles with NbrReplica to transfer
	sendingBundleMsg(netwPkt->getSrcAddr(),sortedWSMPair);
}

void ProphetNCV2::sendingBundleMsg(LAddress::L3Type destAddr,std::vector<std::pair<WaveShortMessage*, int> >wsmToSend)
{
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = wsmToSend.begin(); it != wsmToSend.end(); it++){
		WaveShortMessage* wsm = (*it).first;
		ProphetNCPkt* bundleMsg;
		sectorId = getCurrentSector();
		bundleMsg = prepareNetwPkt(Bundle,myNetwAddr, nodeType, destAddr, sectorId ,getCurrentPos());
		bundleMsg->encapsulate(wsm->dup());
//			cout << "Sending Bundle packet from " << bundleMsg->getSrcAddr() << " addressed to 2Fwds " << fwdDist.first << " & " << fwdMETD.first << " Bundle Serial " << wsm->getSerial() << std::endl;
		sendDown(bundleMsg);
		bundlesReplicaIndex[wsm->getSerial()]++;
	}
}

void ProphetNCV2::handleBundleMsg(ProphetNCPkt *netwPkt)
{
	//	cout << "Receiving Bundle packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	WaveShortMessage *wsm;
	wsm = check_and_cast<WaveShortMessage*>(netwPkt->decapsulate());

	if (wsm != NULL){
		wsm->setHopCount(wsm->getHopCount()+1);

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
			sendingBundleAckMsg(netwPkt->getSrcAddr(), finalReceivedWSM);
		}
	}
}

void ProphetNCV2::sendingBundleAckMsg(LAddress::L3Type destAddr, std::list<unsigned long > wsmFinalDeliverd)
{
	ProphetNCPkt *netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle_Ack,myNetwAddr, nodeType ,destAddr, sectorId , getCurrentPos());
	std::set<unsigned long> serialOfE2EAck;
	for (std::list<unsigned long >::iterator it = wsmFinalDeliverd.begin(); it != wsmFinalDeliverd.end(); it++){
		serialOfE2EAck.insert(*it);
	}
	netwPkt->setE2eAcks(serialOfE2EAck);
	int length = sizeof(unsigned long) * serialOfE2EAck.size()+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);
}

void ProphetNCV2::handleBundleAckMsg(ProphetNCPkt *netwPkt)
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
}

void ProphetNCV2::storeBundle(WaveShortMessage *msg)
{
		// step 1 : check if the bundle is already stored
	if (!exist(msg)){

		// step 2 : add the bundle to stored bundles
		switch (qStrategy) {
			case QUEUING_FIFO:
			{
				if (bundles.size()==bundlesStructureSize){
					bundles.pop_front();
				}else if (bundles.size()>bundlesStructureSize){
					opp_error("bundles storage structure exceed its maximum size");
				}
			}
				break;
			case QUEUING_MOFO:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_MOPR:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_LinearMOPR:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_SHLI:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_LEPR:
				opp_error("QUEUING Strategy not supported yet");
				break;
		}
		bundles.push_back(msg);

		// step 3 : adding this bundle to index
		bundlesIndexIterator it = bundlesIndex.find(msg->getRecipientAddress());
		innerIndexMap inner_map;
		if (it != bundlesIndex.end()){
			inner_map = it->second;
		}
		inner_map.insert(std::pair<unsigned long,WaveShortMessage*>(msg->getSerial(),msg));
		bundlesIndex[msg->getRecipientAddress()] = inner_map;

		haveToRestartIEP(simTime());
	}
}

void ProphetNCV2::updateStoredAcksForSession(LAddress::L3Type srcAddr, std::set<unsigned long > storedAcks)
{
	std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
	if (it2 == neighborhoodSession.end()){
		NetwSession newSession = NetwSession(srcAddr,0);
		for (std::set<unsigned long >::iterator it = storedAcks.begin(); it != storedAcks.end(); it++){
			newSession.insertInDelivredToVpaBndl(*it);
		}
		neighborhoodSession.insert(std::pair<LAddress::L3Type, NetwSession>(srcAddr, newSession));
	}else{
		NetwSession newSession = it2->second;
		for (std::set<unsigned long >::iterator it = storedAcks.begin(); it != storedAcks.end(); it++){
			newSession.insertInDelivredToVpaBndl(*it);
		}
		neighborhoodSession[srcAddr] = newSession;
	}
}
ProphetNCPkt *ProphetNCV2::prepareProphet(short  kind, LAddress::L3Type srcAddr,LAddress::L3Type destAddr, std::list<BundleMeta> *meta, std::map<LAddress::L3Type,double> *preds, WaveShortMessage *msg)
{

	int realPktLength = 0, msgSize = 0, metaLength = 0, predsLength = 0;
	ProphetNCPkt *prophetMsg = new ProphetNCPkt();
	prophetMsg->setKind(kind);
	prophetMsg->setSrcAddr(srcAddr);
	prophetMsg->setDestAddr(destAddr);

	realPktLength = sizeof(kind)+sizeof(srcAddr)+sizeof(destAddr)+sizeof(unsigned long) * 2 + sizeof(int);
	if (meta!=NULL){
		prophetMsg->setBndlmeta(*meta);
		metaLength = (sizeof(BundleMeta)) * meta->size();
		realPktLength += metaLength;
	}
	if (preds!=NULL){
		prophetMsg->setPreds(*preds);
		predsLength = (sizeof(int ) + sizeof(double)) * preds->size();
		realPktLength+= predsLength;
	}
	if (msg!=NULL){
		prophetMsg->encapsulate(msg);
		msgSize = msg->getBitLength();
	}

	realPktLength *= 8;
	realPktLength += msgSize;

	prophetMsg->setBitLength(realPktLength);

	return prophetMsg;
}

void ProphetNCV2::finish()
{
//	EV << "Sent:     " << nbrL3Sent << endl;
//	EV << "Received: " << nbrL3Received << endl;

	recordAllScalars();

	recordScalar("# failed contacts before RIB", nbrFailedContactBeforeRIB);
	recordScalar("# failed contacts at RIB", nbrFailedContactAtRIB);
	recordScalar("# failed contacts at Bundle_Offer", nbrFailedContactAtBundle_Offer);
	recordScalar("# failed contacts at Bundle_Response", nbrFailedContactAtBundle_Response);
	recordScalar("# successful contacts", nbrSuccessfulContact);


	classifyAll();
	recordAllClassifier();

	while (!bundles.empty()){
		delete bundles.front();
		bundles.pop_front();
	}
}

/*******************************************************************
**
** 							Methods for collecting  datas & stats
**
********************************************************************/

void ProphetNCV2::recordPredsStats()
{
	double min = DBL_MAX, max = DBL_MIN, mean = 0, sum = 0, variance = 0, varianceSum = 0;

	for (predsIterator it=preds.begin(); it!=preds.end();it++){
		if (it->first == myNetwAddr){
			continue;
		}

		// Minimum value
		if (it->second < min){
			min = it->second;
		}

		// Maximum value
		if (it->second > max){
			max = it->second;
		}

		// Summing values
		sum+=it->second;
	}

	predsIterator it2 = preds.find(myNetwAddr);
	if (it2 == preds.end()){
		nbrPredsVector.record(preds.size());
		if (preds.size() == 0){
			// Calculating mean
			mean = 0;

			// Calculating variance
			variance = 0;
		}else{
			// Calculating mean
			mean = sum / double (preds.size());

			// Calculating variance
			for (predsIterator it=preds.begin(); it!=preds.end();it++){
				if (it->first != myNetwAddr){
					varianceSum += pow((it->second - mean),2);
				}
			}
			variance = varianceSum / double (preds.size());
		}
	}else{
		if (preds.size() == 0 ){
			nbrPredsVector.record(preds.size());
			// Calculating mean
			mean = 0;

			// Calculating variance
			variance = 0;
		}else{
			nbrPredsVector.record(preds.size()-1);
			// Calculating mean
			mean = sum / double (preds.size()-1);

			// Calculating variance
			for (predsIterator it=preds.begin(); it!=preds.end();it++){
				if (it->first != myNetwAddr){
					varianceSum += pow((it->second - mean),2);
				}
			}
			variance = varianceSum / double (preds.size()-1);
		}
	}

	// recording values
	predsMin.record(min);
	predsMax.record(max);
	predsMean.record(mean);
	predsVariance.record(variance);

	std::map<LAddress::L3Type, int >::iterator it;
	std::map<LAddress::L3Type, cOutVector* >::iterator it3;

	for (it = nbrRepeatedContact.begin(); it != nbrRepeatedContact.end(); it++){
		if (it->second >= maxForRC){
			it3 = predsForRC.find(it->first);
			stringstream flux1;
			std::string tmpStr;
			cOutVector* tmp;
			if (it3 == predsForRC.end()){
				flux1 << it2->first;
				tmpStr = "Evolution of Preds for RC of @"+ flux1.str();
				tmp = new cOutVector(tmpStr.c_str());
			}else{
				tmp = it3->second;
			}
			it2 = preds.find(it->first);
			if (it2 == preds.end()){
				tmp->record(0);
			}else{
				tmp->record(it2->second);
			}
			predsForRC[it->first] = tmp;
		}
	}
}

void ProphetNCV2::recordEndContactStats(LAddress::L3Type addr, double time)
{
	double duration = time - contacts.find(addr)->second;;
	sumOfContactDur+=duration;
	contactDurVector.record(sumOfContactDur/ double (nbrContacts));

	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		switch (it->second) {
		case RIB:
			nbrFailedContactAtRIB++;
			break;
		case Bundle_Offer:
			nbrFailedContactAtBundle_Offer++;
			break;
		case Bundle_Response:
			nbrFailedContactAtBundle_Response++;
			break;
		case Bundle:
			nbrSuccessfulContact++;
			break;
		default:
			nbrFailedContactBeforeRIB++;
			break;
		}

		contacts.erase(addr);
		contactState.erase(addr);

		endContactTime[addr] = time;

		std::map<LAddress::L3Type, int>::iterator it2 = nbrRepeatedContact.find(addr);
		int nbrReContact;
		if (it2 != nbrRepeatedContact.end()){
			nbrReContact = it2->second;
			std::map<int, std::list<double> >::iterator it3 = contactDurForRC.find(nbrReContact);
			std::list<double> durationList;
			if (it3 != contactDurForRC.end()){
				durationList = it3->second;
			}
			durationList.push_back(duration);
			contactDurForRC[nbrReContact] = durationList;

			std::map<LAddress::L3Type, std::list<double> >::iterator it4 = contactDurByAddr.find(addr);
			std::list<double> durationListByAddr;
			if (it4 != contactDurByAddr.end()){
				durationListByAddr = it4->second;
			}
			durationListByAddr.push_back(duration);
			contactDurByAddr[addr] = durationListByAddr;
		}

		it2 = nbrContactsForRC.find(addr);
		int nbrContactSince = 0;
		if (it2 == nbrContactsForRC.end()){
			nbrContactSince = nbrContacts - 0;
		}else{
			nbrContactSince = nbrContacts - nbrContactsForRC[addr];
		}
		nbrContactsForRC[addr] = nbrContactSince;
		nbrContactsForRCVect.record(nbrContactSince);
	}
}

void ProphetNCV2::recordRecontactStats(LAddress::L3Type addr, double time)
{
	std::map<LAddress::L3Type, double>::iterator it = endContactTime.find(addr);
	double duration = 0;
	if (it != endContactTime.end()){
		// updating nbr repeated contacts nodes
		nbrRecontacts++;
		// updating vector stats for recontacted nodes
		duration = time - it->second;

		sumOfInterContactDur+=duration;
		intercontactDurVector.record(sumOfInterContactDur/ double (nbrRecontacts));

		std::map<LAddress::L3Type, int>::iterator it2 = nbrRepeatedContact.find(addr);

		int nbrReContact;
		if (it2 == nbrRepeatedContact.end()){
			nbrReContact = 1;
		}else{
			nbrReContact = it2->second;
			nbrReContact++;
		}
		nbrRepeatedContact[addr] = nbrReContact;

		std::map<int, std::list<double> >::iterator it3 = interContactDurForRC.find(nbrReContact);
		std::list<double> durationList;
		if (it3 != interContactDurForRC.end()){
			durationList = it3->second;
		}
		durationList.push_back(duration);
		interContactDurForRC[nbrReContact] = durationList;

		std::map<LAddress::L3Type, std::list<double> >::iterator it4 = interContactDurByAddr.find(addr);
		std::list<double> durationListByAddr;
		if (it4 != interContactDurByAddr.end()){
			durationListByAddr = it4->second;
		}
		durationListByAddr.push_back(duration);
		interContactDurByAddr[addr] = durationListByAddr;

		it2 = nbrContactsForRC.find(addr);
		int nbrContactSince = 0;
		if (it2 == nbrContactsForRC.end()){
			nbrContactSince = nbrContacts - 0;
		}else{
			nbrContactSince = nbrContacts - nbrContactsForRC[addr];
		}
		nbrContactsForRC[addr] = nbrContactSince;
		nbrContactsForRCVect.record(nbrContactSince);
	}
}

void ProphetNCV2::updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind)
{
	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		contactState.erase(addr);
	}
	contactState.insert(std::pair<LAddress::L3Type, Prophetv2MessageKinds>(addr,kind));
}

void ProphetNCV2::classify(SimpleContactStats* newContact)
{
	DtnNetwLayer::classify(newContact);
	if (!newContact->isSuccessfulContact()){
		switch (newContact->getState()) {
			case RIB:
				if (withFailRIB){
					FailRIB.update(newContact);
				}
				break;
			case Bundle_Offer:
				if (withFailBndlOffer){
					FailBndlOffer.update(newContact);
				}
				break;
			case Bundle_Response:
				if (withFailBndlResp){
					FailBndlResp.update(newContact);
				}
				break;
			case Bundle:
				if (withFailBndl){
					FailBndl.update(newContact);
				}
				break;
			case Bundle_Ack:
				if ((withAck)&&(withFailBndlAck)){
					FailBndlAck.update(newContact);
				}
				break;
			default:
				break;
		}
	}
}

void ProphetNCV2::recordAllClassifier()
{
	DtnNetwLayer::recordAllClassifier();

	if (withFailRIB){
		recordClassifier(FailRIB);
	}
	if (withFailBndlOffer){
		recordClassifier(FailBndlOffer);
	}
	if (withFailBndlResp){
		recordClassifier(FailBndlResp);
	}
	if (withFailBndl){
		recordClassifier(FailBndl);
	}
	if ((withFailBndlAck)&&(withAck)){
		recordClassifier(FailBndlAck);
	}
}

void ProphetNCV2::initAllClassifier()
{
	DtnNetwLayer::initAllClassifier();

	withFailRIB = par("withFailRIBClassifier");
	withCDFForFailRIB = par("CDFForFailRIBClassifier");
	if (withFailRIB){
	    FailRIB = ClassifiedContactStats("RIBFail",false,withCDFForFailRIB);
	}


	withFailBndlOffer = par("withFailBndlOfferClassifier");
	withCDFForFailBndlOffer = par("CDFForFailBndlOfferClassifier");
	if (withFailBndlOffer){
		FailBndlOffer = ClassifiedContactStats("BndlOfferFail",false,withCDFForFailBndlOffer);
	}


	withFailBndlResp = par("withFailBndlRespClassifier");
	withCDFForFailBndlResp = par("CDFForFailBndlRespClassifier");
	if (withFailBndlResp){
		FailBndlResp = ClassifiedContactStats("BndlRespFail",false,withCDFForFailBndlResp);
	}


	withFailBndl = par("withFailBndlClassifier");
	withCDFForFailBndl = par("CDFForFailBndlClassifier");
	if (withFailBndl){
		FailBndl = ClassifiedContactStats("BndlFail",false,withCDFForFailBndl);
	}


	withFailBndlAck = par("withFailBndlAckClassifier");
	withCDFForFailBndlAck = par("CDFForFailBndlAckClassifier");
	if ((withFailBndlAck)&&(withAck)){
		FailBndlAck = ClassifiedContactStats("BndlAckFail",false,withCDFForFailBndlAck);
	}
}

ProphetNCPkt *ProphetNCV2::prepareNetwPkt(short  kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, Coord currentPos)
{
	int realPktLength = 0;
	ProphetNCPkt *myNetwPkt = new ProphetNCPkt();
	myNetwPkt->setKind(kind);
	myNetwPkt->setSrcAddr(srcAddr);
	myNetwPkt->setSrcType(srcType);
	myNetwPkt->setDestAddr(destAddr);
	myNetwPkt->setVpaSectorId(vpaSectorId);
	myNetwPkt->setCurrentPos(currentPos);


	realPktLength = sizeof(kind)+sizeof(srcAddr)+sizeof(destAddr)+sizeof(unsigned long) * 2 + sizeof(int);
	realPktLength *= 8;

	myNetwPkt->setBitLength(realPktLength);

	return myNetwPkt;
}

void ProphetNCV2::storeAckSerial(unsigned long  serial)
{
	if (ackSerial.count(serial) == 0){
		ackSerial.insert(serial);
	}
}

void ProphetNCV2::storeAckSerial(std::set<unsigned long > setOfSerials)
{
    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
    	storeAckSerial(*it);
    	erase(*it);
    }
}

bool ProphetNCV2::erase(unsigned long serial)
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

/*******************************************************************
**
** 							Unused functions
**
********************************************************************/

ProphetNCV2::~ProphetNCV2()
{

}
