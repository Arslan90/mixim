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
			case RIB:
				if (prophetPkt->getDestAddr()==myNetwAddr){
					handleRIBMsg(prophetPkt);
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
	switch (msg->getKind()) {
		case RESTART:
			if (canITransmit){

			/*
			 * Extracting destAddress and Time from controlMsgName
			 */
			char* msgName = strdup(msg->getName());

			LAddress::L3Type addr = getAddressFromName((const char*)strtok(msgName,":"));

			ProphetNCPkt* emulatedPkt;
			emulatedPkt = prepareProphet(RIB,addr,myNetwAddr);

			if (recordContactStats){
				unsigned long contactID;
				iteratorContactID iterator1 = indexContactID.find(addr);
				if (iterator1 != indexContactID.end()){
					contactID = iterator1->second.back();
				}else{
					opp_error("contact does not exist");
				}
				emulatedPkt->setContactID(contactID);
			}

			/** Starting IEP Phase					*/

			/*
			 * We emulate the reception of a prophetPkt from the other node with contactID as a serial
			 * calculated by startRecoringContact
			 */
			NetwOppControlInfo* controlInfo = new NetwOppControlInfo(RESTART);
			emulatedPkt->setControlInfo(controlInfo);

			emulatedPkt->setRestartIEP(true);

			executeInitiatorRole(RIB,emulatedPkt);
			delete emulatedPkt;


			}
			break;
		case FORCED_RESTART:
			if (canITransmit){

			/*
			 * Extracting destAddress and Time from controlMsgName
			 */
			char* msgName = strdup(msg->getName());

			LAddress::L3Type addr = getAddressFromName((const char*)strtok(msgName,":"));

			ProphetNCPkt* emulatedPkt;
			emulatedPkt = prepareProphet(RIB,addr,myNetwAddr);

			if (recordContactStats){
				unsigned long contactID;
				iteratorContactID iterator1 = indexContactID.find(addr);
				if (iterator1 != indexContactID.end()){
					contactID = iterator1->second.back();
				}else{
					opp_error("contact does not exist");
				}
				emulatedPkt->setContactID(contactID);
			}

			/** Starting IEP Phase					*/

			/*
			 * We emulate the reception of a prophetPkt from the other node with contactID as a serial
			 * calculated by startRecoringContact
			 */
			NetwOppControlInfo* controlInfo = new NetwOppControlInfo(RESTART);
			emulatedPkt->setControlInfo(controlInfo);

			executeInitiatorRole(RIB,emulatedPkt);
			delete emulatedPkt;


			}
			break;
		default:
			if (msg == heartBeatMsg){
				updateNeighborhoodTable(myNetwAddr, NetwRoute(myNetwAddr,maxDbl,maxDbl, simTime(), true, nodeType, getCurrentPos()));
				ProphetNCPkt* netwPkt;
				sendingHelloMsg(netwPkt);
				scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
			}
			break;
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
//	netwPkt->setE2eAcks(ackSerial);
//	std::set<unsigned long > storedBundle;
//	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
//		storedBundle.insert((*it)->getSerial());
//	}
//	netwPkt->setH2hAcks(storedBundle);
//	int nbrEntries = storedBundle.size(); //ackSerial.size()+ storedBundle.size();

	std::map<LAddress::L3Type, double> predToSend = std::map<LAddress::L3Type, double>();
	ageDeliveryPreds();
	predToSend.insert(preds.begin(),preds.end());
	netwPkt->setPreds(predToSend);


	int predsLength = (sizeof(int ) + sizeof(double)) * predToSend.size();
	int length = predsLength + netwPkt->getBitLength();
//	int length = sizeof(unsigned long) * (nbrEntries)+ predsLength + netwPkt->getBitLength();
//	int length = sizeof(unsigned long) * (nbrEntries)+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
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
//	    std::set<unsigned long> receivedE2eAcks = netwPkt->getE2eAcks();
//	    if (!receivedE2eAcks.empty()){
//	    	storeAckSerial(receivedE2eAcks);
//	    }
//	    std::set<unsigned long> storedBundle = netwPkt->getH2hAcks();
//	    if (!storedBundle.empty()){
//	    	updateStoredBndlForSession(netwPkt->getSrcAddr(), storedBundle);
//	    }

	    /*************************** Sending RIB Msg **********/
//	    sendingRIBMsg(netwPkt->getSrcAddr());


	    update(netwPkt);
	    if (nodeType == Veh){
	    	sendingBndlOfferMsg(netwPkt->getSrcAddr(),netwPkt->getPreds());
		}
//	    if (nodeType == VPA){
//	    	return;
//	    }else{
//	    	/*************************** Sending Bundle Msg **********/
//
//		    if (netwPkt->getSrcType() == VPA){
////		    	sendingBundleMsgToVPA(netwPkt->getSrcAddr());
////		    	vpaContactDistance.push_back(getCurrentPos().distance(netwPkt->getCurrentPos()));
//		    }else if (netwPkt->getSrcType() == Veh){
////		    	sendingBundleMsg();
//		    }
//	    }
	}
}

void ProphetNCV2::sendingRIBMsg(LAddress::L3Type nodeAddr)
{
	ProphetNCPkt *netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(RIB,myNetwAddr, nodeType ,nodeAddr, sectorId , getCurrentPos());

	std::map<LAddress::L3Type, double> predToSend = std::map<LAddress::L3Type, double>();
	ageDeliveryPreds();
	predToSend.insert(preds.begin(),preds.end());
	netwPkt->setPreds(predToSend);

	int predsLength = (sizeof(int ) + sizeof(double)) * predToSend.size();
	int length = predsLength + netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	sendDown(netwPkt);


//	std::map<LAddress::L3Type, double> predToSend = std::map<LAddress::L3Type, double>();
//	ageDeliveryPreds();
//	predToSend.insert(preds.begin(),preds.end());
//
//	ProphetNCPkt *ribPkt;
//	ribPkt = prepareProphet(RIB, myNetwAddr, nodeAddr, NULL, &predToSend);
//	sendDown(ribPkt);
}

void ProphetNCV2::handleRIBMsg(ProphetNCPkt *netwPkt)
{
	update(netwPkt);
    if (nodeType == Veh){
    	sendingBndlOfferMsg(netwPkt->getSrcAddr(),netwPkt->getPreds());
	}
}

void ProphetNCV2::sendingBndlOfferMsg(LAddress::L3Type nodeAddr, std::map<LAddress::L3Type, double> predsOfNode)
{
	ProphetNCPkt *netwPkt;
	sectorId = getCurrentSector();
	netwPkt = prepareNetwPkt(Bundle_Offer,myNetwAddr, nodeType ,nodeAddr, sectorId , getCurrentPos());

	/*************************** H2H Acks (stored bundles) **********/
	std::set<unsigned long> serialOfH2hAck;
	for (std::list<WaveShortMessage* >::iterator it = bundles.begin(); it != bundles.end(); it++){
		unsigned long wsmSerial = (*it)->getSerial();
		std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(nodeAddr);
		if (it2 != neighborhoodSession.end()){
			NetwSession newSession = it2->second;
			if (newSession.getStoredBndl().count(wsmSerial) > 0){
				continue;
			}else if (newSession.getDelivredToBndl().count(wsmSerial) > 0){
				continue;
			}else if (newSession.getDelivredToVpaBndl().count(wsmSerial) > 0){
				continue;
			}
		}
		predsIterator myPred = preds.find((*it)->getRecipientAddress());
		predsIterator otherPred = predsOfNode.find((*it)->getRecipientAddress());
		bool addToOffer = false;
		if (otherPred != predsOfNode.end()){
			if (myPred != preds.end()){
				if (myPred->second <= otherPred->second){
					addToOffer = true;
				}
			}else{
				addToOffer = true;
			}
		}
		if (addToOffer){
			serialOfH2hAck.insert(wsmSerial);
		}
	}
	netwPkt->setH2hAcks(serialOfH2hAck);

	/*************************** E2E Acks **********/
	std::set<unsigned long> serialOfE2eAck;
	for (std::set<unsigned long >::iterator it = ackSerial.begin(); it != ackSerial.end(); it++){
		unsigned long wsmSerial = (*it);
		std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(nodeAddr);
		if (it2 != neighborhoodSession.end()){
			NetwSession newSession = it2->second;
			if (newSession.getStoredBndl().count(wsmSerial) > 0){
				continue;
			}else if (newSession.getDelivredToBndl().count(wsmSerial) > 0){
				continue;
			}else if (newSession.getDelivredToVpaBndl().count(wsmSerial) > 0){
				continue;
			}
		}
		serialOfE2eAck.insert(wsmSerial);
	}
	netwPkt->setE2eAcks(serialOfE2eAck);

	int nbrEntries = serialOfH2hAck.size()+serialOfE2eAck.size();
	int length = sizeof(unsigned long) * (nbrEntries)+ netwPkt->getBitLength();
	netwPkt->setBitLength(length);
	cout << "Sending BundleOffer packet from " << netwPkt->getSrcAddr() << " addressed to " << netwPkt->getDestAddr() << std::endl;
	sendDown(netwPkt);
}

void ProphetNCV2::handleBndlOfferMsg(ProphetNCPkt *netwPkt)
{
    /*************************** E2E Acks **********/
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

	if (!serialStoredBndl.empty()){
		updateStoredBndlForSession(netwPkt->getSrcAddr(), serialStoredBndl);
	}

	sendingBndlRespMsg(netwPkt->getSrcAddr(), serialResponseBndl);
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

void ProphetNCV2::sendingBundleMsgToVPA(LAddress::L3Type vpaAddr)
{

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

void ProphetNCV2::executeInitiatorRole(short  kind, ProphetNCPkt *prophetPkt)
{
	/*
	 * First we check if we actually record stats about contact
	 */
	unsigned long contactID;
	SimpleContactStats contact;
	if (recordContactStats){
		contactID = prophetPkt->getContactID();
		iteratorContactStats iterator1 = indexContactStats.find(contactID);
		if (iterator1 == indexContactStats.end()){
			/*
			 * No entry found for this address, it is impossible
			 */
			opp_error("looking for an non-existent contact(ProphetNCV2::executeInitiatorRole)");
		}else {
			contact = iterator1->second;
		}
	}

	switch (kind) {
		case HELLO:
			break;
		case ERROR:
			break;
		case RIBD:
			break;
		case RIB:
		{
			std::map<LAddress::L3Type, double> predToSend = std::map<LAddress::L3Type, double>();
			ageDeliveryPreds();
			predToSend.insert(preds.begin(),preds.end());

			// Decide if we have to fragment predictions in order to send them
			bool shouldFragment = false;
			int predSize = ((sizeof(int) + sizeof(double)) * predToSend.size()) * 8; // to express the size in bits unit

			if (predSize > dataLength) {
				shouldFragment = true;
			}

			if ((dontFragment) || (!shouldFragment)){
				ProphetNCPkt *ribPkt;
				ribPkt = prepareProphet(RIB, myNetwAddr, prophetPkt->getSrcAddr(), NULL, &predToSend);
				ribPkt->setContactID(prophetPkt->getContactID());
				ribPkt->setDf_Flag(false);

		    	cObject* cInfo = prophetPkt->removeControlInfo();
				if(cInfo != NULL){
					ribPkt->setControlInfo(cInfo);
				}

				ribPkt->setRestartIEP(prophetPkt->getRestartIEP());

				if (canITransmit){
					if (delayed == 0){
						sendDown(ribPkt);
					}else{
						sendDelayed(ribPkt,dblrand()*delayed,"lowerLayerOut");
					}

					/*
					 * Collecting data
					 */
					updatingL3Sent();
					updatingContactState(prophetPkt->getSrcAddr(),RIB);

					if (recordContactStats){
						contact.setL3Sent();
						contact.setPredictionsSent(contact.getPredictionsSent()+predToSend.size());
						updateContactWhenInit(prophetPkt, contactID, contact, kind);
					}
				}else {
					delete ribPkt;
				}


			}else{
				// we have to send fragment of predictions separately

				int entrySize = (sizeof(int) + sizeof(double)) * 8;
				int maxEntriesParFrag = dataLength / entrySize;
				int totalEntries = predSize / entrySize;

				short nbrFragment = totalEntries / maxEntriesParFrag;
				if (totalEntries % maxEntriesParFrag != 0){
					nbrFragment++;
				}

				short fragmentNum = 0;
				int index = 0;
				int remainingEntries = totalEntries;
				int nbrEntriesParFrag = 0;


				// in order to facilitate the process we copy preds into a vector to access it by index
				std::vector<std::pair<LAddress::L3Type, double> >tmp = std::vector<std::pair<LAddress::L3Type, double> >();

				for (predsIterator it = predToSend.begin(); it != predToSend.end(); it++){
					tmp.push_back(std::pair<LAddress::L3Type, double>(it->first, it->second));
				}

				while (remainingEntries > 0){
					if (remainingEntries >= maxEntriesParFrag){
						nbrEntriesParFrag = maxEntriesParFrag;
					}else{
						nbrEntriesParFrag = remainingEntries;
					}


					std::map<LAddress::L3Type, double> predFragment = std::map<LAddress::L3Type, double>();
					for (int i = index; i < index+nbrEntriesParFrag;i++){
						predFragment.insert(tmp[i]);
					}

					ProphetNCPkt *ribPkt;
					ribPkt = prepareProphet(RIB, myNetwAddr, prophetPkt->getSrcAddr(), NULL, &predFragment);
					ribPkt->setContactID(prophetPkt->getContactID());
					ribPkt->setDf_Flag(true);
					ribPkt->setFragNbr(fragmentNum);
					ribPkt->setFragTotal(nbrFragment);

					if (canITransmit){
						if (delayedFrag == 0){
							sendDown(ribPkt);
						}else{
							sendDelayed(ribPkt,dblrand()*delayedFrag,"lowerLayerOut");
						}

						/*
						 * Collecting data
						 */
						updatingL3Sent();
						updatingContactState(prophetPkt->getSrcAddr(),RIB);

						if (recordContactStats){
							contact.setL3Sent();
							contact.setPredictionsSent(contact.getPredictionsSent()+predToSend.size());
							updateContactWhenInit(prophetPkt, contactID, contact, kind);
						}
					}else {
						delete ribPkt;
					}

					remainingEntries-=nbrEntriesParFrag;
					fragmentNum++;
					index = index+nbrEntriesParFrag;
				}
			}
		}
			break;
		case Bundle_Offer:
		{
			/*
			 * Collecting data
			 */
			updatingL3Received();
			if (recordContactStats){
				contact.setL3Received();
				updateContactWhenInit(prophetPkt, contactID, contact, kind);
			}

			if (abortConnection(Bundle_Offer,prophetPkt)){
				/*
				 * Nothing to do, we have to stop the exchange
				 */
				updatingContactState(prophetPkt->getSrcAddr(),Bundle);

				if (recordContactStats){
					contact.setSuccessfulContact(true);
					updateContactWhenInit(prophetPkt, contactID, contact, kind);
				}
			}else {
				std::list<BundleMeta> bndlMetaToAccept;

				int bndlOffer = 0;
				int alreadyAcked = 0;

				for (std::list<BundleMeta>::iterator it = prophetPkt->getBndlmeta().begin(); it !=prophetPkt->getBndlmeta().end(); ++it) {
					if (it->getFlags() == Prophet_Enum::Bndl_Accepted){
						bndlOffer++;
						/*
						 * step 1 : check if offered bundles are already stored in this node,
						 * in that case delete them from the offered list
						 */
						if (exist(*it)){
							continue;
						}
						/*
						 * step 2 : check if offered bundles are already acked,
						 * in that case delete them from the offered list
						 */
						if (ackExist(*it)){
							demandedAckedBundle++;
							alreadyAcked++;
							continue;
						}
					}else if (it->getFlags() == Prophet_Enum::PRoPHET_ACK){
						/*
						 * step 3 : check if BundleMeta is ACK,
						 * in that case add it and delete the corresponding bundle from storage
						 */
						storeACK(*it);
						if (recordContactStats){
							contact.setAckReceived();
							updateContactWhenInit(prophetPkt, contactID, contact, kind);
						}
						continue;
					}else{
						opp_error("Reception of bundle Meta of unknown type(ProphetNCV2::executeInitiatorRole)");
					}
					bndlMetaToAccept.push_back(*it);
				}

				if (recordContactStats){
					contact.setOfferReceived(bndlOffer);
					contact.setNbrAlreadyAcked(alreadyAcked);
					updateContactWhenInit(prophetPkt, contactID, contact, kind);
				}

				/*
				 * highly important: without doing this we don't transmit the filtered list to the next step
				 * and so we will send bundle previously discarded
				 */
				prophetPkt->setBndlmeta(bndlMetaToAccept);

				executeInitiatorRole(Bundle_Response,prophetPkt);

				/*
				 * Collecting data
				 */
				updatingContactState(prophetPkt->getSrcAddr(),Bundle_Offer);
			}
		}
			break;
		case Bundle_Response:
		{
			std::list<BundleMeta> bundleToAcceptMeta;
			bundleToAcceptMeta = prophetPkt->getBndlmeta();

			/*
			 * step 1 : sending the response
			 */

			ProphetNCPkt *responsePkt;// = new Prophet();
			responsePkt = prepareProphet(Bundle_Response,myNetwAddr,prophetPkt->getSrcAddr(), &bundleToAcceptMeta);
			responsePkt->setContactID(prophetPkt->getContactID());
			if (canITransmit){
//				if (delayed == 0){
					sendDown(responsePkt);
//				}else{
//					sendDelayed(responsePkt,dblrand()*delayed,"lowerLayerOut");
//				}

				/*
				 * Collecting data
				 */
				updatingL3Sent();
				updatingContactState(prophetPkt->getSrcAddr(),Bundle_Response);

				if (recordContactStats){
					contact.setL3Sent();
					contact.setAcceptSent(bundleToAcceptMeta.size());
					updateContactWhenInit(prophetPkt, contactID, contact, kind);
				}
			}else{
				delete responsePkt;
			}
		}
			break;
		case Bundle:
		{
			/*
			 * Collecting data
			 */
			updatingL3Received();
			updatingContactState(prophetPkt->getSrcAddr(),Bundle);
			if (recordContactStats){
				contact.setL3Received();
				updateContactWhenInit(prophetPkt, contactID, contact, kind);
			}

			if (!(withAck)&&(recordContactStats)){
				contact.setSuccessfulContact(true);
			}

			if (!abortConnection(Bundle,prophetPkt)){
				bundlesReceived++;

				if (recordContactStats){
					contact.setBundleReceived();
					updateContactWhenInit(prophetPkt, contactID, contact, kind);
				}

				WaveShortMessage *wsm;
				wsm = check_and_cast<WaveShortMessage*>(prophetPkt->decapsulate());
				wsm->setHopCount(wsm->getHopCount()+1);

				if (wsm->getRecipientAddress() == myNetwAddr){
					prophetPkt->encapsulate(wsm);
					sendUp(prophetPkt->dup());
					if (withAck){
						executeInitiatorRole(Bundle_Ack,prophetPkt);
					}
				}else {
					/*
					 * Process to avoid storing twice the same msg
					 */
					if ( ! (exist(wsm)||ackExist(wsm))){
						storeBundle(wsm);
					}
				}
			}
		}
			break;
		case Bundle_Ack:{
			/*
			 * Note: No need to send all acks because it's already done in Bundle_Offer,
			 * send only ack of the received bundle
			 */
			ProphetNCPkt *ackPkt;// = new Prophet();
			WaveShortMessage *wsm = check_and_cast<WaveShortMessage*>(prophetPkt->getEncapsulatedPacket());

			if (!ackExist(wsm)){
				/*
				 * Step 1 : Creating the ACK
				 */
				BundleMeta meta = BundleMeta(wsm,Prophet_Enum::PRoPHET_ACK);
				storeACK(meta);

				/*
				 * Step 2 : Sending the ACK
				 */
				std::list<BundleMeta> acksMeta;
				BundleMeta copy;
				copy = meta;
				acksMeta.push_back(copy);

				ackPkt = prepareProphet(Bundle_Ack, myNetwAddr, prophetPkt->getSrcAddr(), &acksMeta);
				ackPkt->setContactID(prophetPkt->getContactID());
				if (canITransmit){
//					if (delayed == 0){
						sendDown(ackPkt);
//					}else{
//						sendDelayed(ackPkt,dblrand()*delayed,"lowerLayerOut");
//					}
					/*
					 * Collecting data
					 */
					updatingL3Sent();

					if (recordContactStats){
						contact.setL3Sent();
						contact.setAckSent();
						contact.setSuccessfulContact(true);
						updateContactWhenInit(prophetPkt, contactID, contact, kind);
					}
				}else{
					delete ackPkt;
				}
			}
		}
			break;
		default:
			opp_error("Unknown Prophetv2MessageKinds when calling executeInitiatorRole()");
			break;
	}
}

void ProphetNCV2::executeListenerRole(short  kind, ProphetNCPkt *prophetPkt)
{
	/*
	 * First we check if we actually record stats about contact
	 */
	unsigned long contactID;
	SimpleContactStats contact;
	if (recordContactStats){
		contactID = prophetPkt->getContactID();
		iteratorContactStats iterator1 = indexContactStats.find(contactID);
		if (iterator1 == indexContactStats.end()){
			if (kind == RIB){
				/*
				 * possibility that we received the RIB msg before receiving the control msg
				 * We must create an entry for this contact
				 */
				startRecordingContact(prophetPkt->getSrcAddr(),contactID);
				iteratorContactStats iterator2 = indexContactStats.find(contactID);
				if (iterator2 == indexContactStats.end()){
					/*
					 * No entry found for this address, it is impossible
					 */
					opp_error("looking for an non-existent contact that has been inserted(ProphetNCV2::executeInitiatorRole)");
				}else{
					contact = iterator2->second;
				}
			}else{
				/*
				 * No entry found for this address, it is impossible
				 */
				opp_error("looking for an non-existent contact(ProphetNCV2::executeInitiatorRole)");
			}
		}else {
			contact = iterator1->second;

		}
	}

	switch (kind) {
		case HELLO:
			break;
		case ERROR:
			break;
		case RIBD:
			break;
		case RIB:
		{

			if (!abortConnection(RIB,prophetPkt)){
				bool haveToPartiallyUpdate = false;
				if ((prophetPkt->getDf_Flag()==true)&&(contact.getPredictionsReceived()!=0)){
					haveToPartiallyUpdate = true;
				}

				if ((withPartialUpdate)&&(haveToPartiallyUpdate)){
					partialUpdate(prophetPkt);
				}else{
					update(prophetPkt);
				}
			}

			/*
			 * Collecting data
			 */
			updatingL3Received();
			if (recordContactStats){
				contact.setL3Received();
				contact.setPredictionsReceived(contact.getPredictionsReceived()+prophetPkt->getPreds().size());
				updateContactWhenList(prophetPkt, contactID, contact, kind);
			}

			executeListenerRole(Bundle_Offer,prophetPkt);
		}
			break;
		case Bundle_Offer:
		{
			/*
			 * Step 1 : Calculating Bundle to Offer
			 */

			if (withTTL){
				deleteOldBundle(ttl);
			}

			std::list<BundleMeta> bundleToOfferMeta;

			std::list<BundleMeta> directBundleToOffer;
			std::list<BundleMeta> otherBundleToOffer;
			std::list<BundleMeta> ackToOffer;

			if (!abortConnection(RIB,prophetPkt)){
				/*
				 * Extracting each type of bundleOffer, 1st list is related to bundle directly addressed to encounter node
				 * 2nd is related to other bundle to offer, 3rd is related to ack
				 */
				std::vector<std::list<BundleMeta> > allBundleMeta = defineBundleOffer(prophetPkt);
				for (unsigned int i  = 0; i  < allBundleMeta.size(); i++ ) {
					std::list<BundleMeta> tmp = allBundleMeta[i];
					switch (i) {
						case 0:
							directBundleToOffer = tmp;
							break;
						case 1:
							otherBundleToOffer = tmp;
							break;
						case 2:
							ackToOffer = tmp;
							break;
						default:
							opp_error("definition of Bundle Offer must return a vector of size equal to 3(ProphetNCV2::executeListenerRole)");
							break;
					}
					bundleToOfferMeta.insert(bundleToOfferMeta.end(),tmp.begin(),tmp.end());
				}
			}

			lastBundleProposal[prophetPkt->getSrcAddr()] = simTime().dbl();

			// Decide if we have to fragment predictions in order to send them
			bool shouldFragment = false;
			int bundleMetaSize = ((sizeof(BundleMeta)) * bundleToOfferMeta.size()) * 8; // to express the size in bits unit

			if (bundleMetaSize > dataLength) {
				shouldFragment = true;
			}

			if ((dontFragment) || (!shouldFragment)){

				/*
				 * Step 2 : Sending the ProphetPckt
				 */

				ProphetNCPkt *offerPkt;// = new Prophet();
				offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,prophetPkt->getSrcAddr(),&bundleToOfferMeta);
				offerPkt->setContactID(prophetPkt->getContactID());
				offerPkt->setDf_Flag(false);
				if (canITransmit){
					if (delayed == 0){
						sendDown(offerPkt);
					}else{
						sendDelayed(offerPkt,dblrand()*delayed,"lowerLayerOut");
					}

					/*
					 * Collecting data
					 */
					updatingL3Sent();

					if (recordContactStats){
						contact.setL3Sent();
						contact.setAckSent(contact.getAckSent()+ackToOffer.size());
						contact.setOfferSent(directBundleToOffer.size()+otherBundleToOffer.size());
						updateContactWhenList(prophetPkt, contactID, contact, kind);
					}
				}else{
					delete offerPkt;
				}

			}else{
				// we have to send fragment of predictions separately

				int entrySize = (sizeof(BundleMeta)) * 8;
				int maxEntriesParFrag = dataLength / entrySize;
				int totalEntries = bundleMetaSize / entrySize;

				short nbrFragment = totalEntries / maxEntriesParFrag;
				if (totalEntries % maxEntriesParFrag != 0){
					nbrFragment++;
				}

				short fragmentNum = 0;
				int index = 0;
				int remainingEntries = totalEntries;
				int nbrEntriesParFrag = 0;

				while (remainingEntries > 0){
					if (remainingEntries >= maxEntriesParFrag){
						nbrEntriesParFrag = maxEntriesParFrag;
					}else{
						nbrEntriesParFrag = remainingEntries;
					}

					std::list<BundleMeta> bndlMetaFragment = std::list<BundleMeta>();
					for (int i = index; i < index+nbrEntriesParFrag;i++){
						BundleMeta tmp = BundleMeta(bundleToOfferMeta.front());
						bndlMetaFragment.push_back(tmp);
						bundleToOfferMeta.pop_front();
					}

					/*
					 * Step 2 : Sending the ProphetPckt
					 */

					ProphetNCPkt *offerPkt;// = new Prophet();
					offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,prophetPkt->getSrcAddr(),&bndlMetaFragment);
					offerPkt->setContactID(prophetPkt->getContactID());
					offerPkt->setDf_Flag(true);
					offerPkt->setFragNbr(fragmentNum);
					offerPkt->setFragTotal(nbrFragment);
					if (canITransmit){
						if (delayedFrag == 0){
							sendDown(offerPkt);
						}else{
							sendDelayed(offerPkt,dblrand()*delayedFrag,"lowerLayerOut");
						}

						/*
						 * Collecting data
						 */
						updatingL3Sent();

						if (recordContactStats){
							contact.setL3Sent();
							int nbrAckSent = 0;
							for (std::list<BundleMeta>::iterator it = bndlMetaFragment.begin(); it != bndlMetaFragment.end(); it++){
								if (it->getFlags() == Prophet_Enum::PRoPHET_ACK){
									nbrAckSent++;
								}
							}
							contact.setAckSent(contact.getAckSent()+nbrAckSent);
							contact.setOfferSent(prophetPkt->getBndlmeta().size());
							updateContactWhenList(prophetPkt, contactID, contact, kind);
						}
					}else{
						delete offerPkt;
					}

					remainingEntries-= nbrEntriesParFrag;
					fragmentNum++;
					index = index+nbrEntriesParFrag;
				}
			}
		}
			break;
		case Bundle_Response:
		{
			std::list<BundleMeta> bndlMetaToAccept;
			bndlMetaToAccept.clear();
			if (withAck){
				/*
				 * 1 step : Check if demanded bundle in bundleResp is currently acked, delete it if it's the case
				 */

				for (std::list<BundleMeta>::iterator it = prophetPkt->getBndlmeta().begin(); it !=prophetPkt->getBndlmeta().end(); ++it) {
					if (!ackExist(*it)){
						bndlMetaToAccept.push_back(*it);
					}
				}
				/*
				 * highly important: without doing this we don't transmit the filtered list to the next step
				 * and so we will send bundle previously discarded
				 */
				prophetPkt->setBndlmeta(bndlMetaToAccept);
			}

			/*
			 * Collecting data
			 */
			updatingL3Received();
			if (recordContactStats){
				contact.setL3Received();
				contact.setAcceptReceived(prophetPkt->getBndlmeta().size());
				updateContactWhenList(prophetPkt, contactID, contact, kind);
			}
			executeListenerRole(Bundle,prophetPkt);
		}
			break;
		case Bundle:
		{
			if (withTTL){
				deleteOldBundle(ttl);
			}

			std::list<BundleMeta> bundleToSendMeta;
			bundleToSendMeta = prophetPkt->getBndlmeta();

			ProphetNCPkt *bundlePkt;

			if (abortConnection(Bundle_Response,prophetPkt)){
				/*
				 * No Bundle to transmit, send a prophet msg with NULL pointer instead of an encapsulated bundle
				 */
				bundlePkt = prepareProphet(Bundle,myNetwAddr,prophetPkt->getSrcAddr(),NULL,NULL,NULL);
				bundlePkt->setContactID(prophetPkt->getContactID());
				if (canITransmit){
//					if (delayed == 0){
						sendDown(bundlePkt);
//					}else{
//						sendDelayed(bundlePkt,dblrand()*delayed,"lowerLayerOut");
//					}
					/*
					 * Collecting data
					 */
					updatingL3Sent();
					if (recordContactStats){
						contact.setL3Sent();
						contact.setBundleSent();
						updateContactWhenList(prophetPkt, contactID, contact, kind);
					}
				}else{
					delete bundlePkt;
				}
			}else{
				/*
				 * 1 step : Send the corresponding bundle
				 */
				for (std::list<BundleMeta>::iterator it = bundleToSendMeta.begin(); it !=bundleToSendMeta.end(); ++it) {
					bundlesIndexIterator it2 = bundlesIndex.find(it->getRecipientAddress());
					if (it2!=bundlesIndex.end()){
						innerIndexIterator it3 = it2->second.find(it->getSerial());
						if (it3!=it2->second.end()){
							bundlePkt = prepareProphet(Bundle,myNetwAddr,prophetPkt->getSrcAddr(),NULL,NULL,(it3->second)->dup());
							bundlePkt->setContactID(prophetPkt->getContactID());
							if (canITransmit){
//								if (delayed == 0){
									sendDown(bundlePkt);
//								}else{
//									sendDelayed(bundlePkt,dblrand()*delayed,"lowerLayerOut");
//								}
								/*
								 * Collecting data
								 */
								updatingL3Sent();
								if (recordContactStats){
									contact.setL3Sent();
									contact.setBundleSent();
									updateContactWhenList(prophetPkt, contactID, contact, kind);
								}
							}else {
								delete bundlePkt;
							}
						}
					}
				}
			}
		}
			break;
		case Bundle_Ack:
		{
			/*
			 * Collecting data
			 */
			updatingL3Received();
			if (recordContactStats){
				contact.setL3Received();
				updateContactWhenList(prophetPkt, contactID, contact, kind);
			}

			std::list<BundleMeta> acksMeta;
			acksMeta = prophetPkt->getBndlmeta();

			if (!abortConnection(kind,prophetPkt)){
				for (std::list<BundleMeta>::iterator ackIt = acksMeta.begin(); ackIt !=acksMeta.end(); ++ackIt) {
					BundleMeta meta = *ackIt;
					storeACK(meta);
					if (recordContactStats){
						contact.setAckReceived();
						updateContactWhenList(prophetPkt, contactID, contact, kind);
					}
				}
			}
		}
			break;

		default:
			opp_error("Unknown Prophetv2MessageKinds when calling executeListenerRole()");
		break;
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

std::vector<std::list<BundleMeta> >ProphetNCV2::defineBundleOffer(ProphetNCPkt *prophetPkt)
{
	LAddress::L3Type encounterdNode = prophetPkt->getSrcAddr();
	std::map<LAddress::L3Type, double> concernedPreds = std::map<LAddress::L3Type, double>();
	std::vector<std::pair<LAddress::L3Type, double>	> sortedPreds;
	std::vector<std::list<BundleMeta> > allBundleMeta;

	std::list<BundleMeta> directBundleToOffer = std::list<BundleMeta>();
	std::list<BundleMeta> otherBundleToOffer = std::list<BundleMeta>();
	std::list<BundleMeta> ackToOffer = std::list<BundleMeta>();

	// step 1 : check if we have any bundle that are addressed to @encouterdNode

	bundlesIndexIterator it = bundlesIndex.find(encounterdNode);
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2;
		for (it2 = innerMap.begin(); it2 !=innerMap.end(); ++it2){
			BundleMeta meta (it2->second, Prophet_Enum::Bndl_Accepted);

			if (withAck){
				if (acksIndex.find(it2->second->getSerial())!=acksIndex.end()){
					existAndErase(meta);
					continue;
				}
			}
			directBundleToOffer.push_back(meta);
		}
	}

	// step 2 : check if we have any bundle that can be offered to @encouterdNode

	ageDeliveryPreds();
	predsIterator itCurrentNode;
	for (predsIterator itEncouterdNode =prophetPkt->getPreds().begin();itEncouterdNode !=prophetPkt->getPreds().end(); ++itEncouterdNode){
		if (itEncouterdNode->first == encounterdNode){
			continue;
		}

		itCurrentNode = preds.find(itEncouterdNode->first);
		if (itCurrentNode != preds.end()){
			// the current node have a prediction for @it->first 
			// the 2 nodes have a predictions for this destination
			if (itEncouterdNode->second > itCurrentNode->second){
				concernedPreds.insert(std::pair<LAddress::L3Type, double>(itEncouterdNode->first,itEncouterdNode->second));
			}
		}
	}

	switch (fwdStrategy) {
		case FWD_GRTR:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GTMX:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GTHR:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GRTRplus:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GTMXplus:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GRTRsort:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GRTRmax:
		{
			sortedPreds = std::vector<std::pair<LAddress::L3Type, double> >(concernedPreds.begin(), concernedPreds.end());
			std::sort(sortedPreds.begin(),sortedPreds.end(),fwdGRTRmax_CompObject);
		}
			break;
		default:
			break;
	}

	for (std::vector<std::pair<LAddress::L3Type, double> >::iterator it= sortedPreds.begin(); it != sortedPreds.end(); ++it){
		bundlesIndexIterator it2 = bundlesIndex.find(it->first);
			if (it2 != bundlesIndex.end()){
				innerIndexMap innerMap(it2->second);
				innerIndexIterator it3;
				for (it3 = innerMap.begin(); it3 !=innerMap.end(); ++it3){
					BundleMeta meta (it3->second, Prophet_Enum::Bndl_Accepted);

					if (withAck){
						if (acksIndex.find(it3->second->getSerial())!=acksIndex.end()){
							existAndErase(meta);
							continue;
						}
					}
					otherBundleToOffer.push_back(meta);
				}
			}
	}

	// step 3 : check if we have any ack that must be transmitted

	if (withAck){
		for (std::list<BundleMeta>::iterator it = acks.begin(); it !=acks.end(); ++it) {
			if (existAndErase(*it)){
				continue;
			}
			ackToOffer.push_back(*it);
		}
	}

	allBundleMeta.push_back(directBundleToOffer);
	allBundleMeta.push_back(otherBundleToOffer);
	allBundleMeta.push_back(ackToOffer);

	if (allBundleMeta.size()!=3){
		opp_error("definition of Bundle Offer must return a vector of size equal to 3(ProphetNCV2::defineBundleOffer)");
	}

	return allBundleMeta;
}

bool ProphetNCV2::abortConnection(short  kind, ProphetNCPkt *prophetPkt)
{
	bool abort = false;

	switch (kind) {
		case HELLO:
			break;
		case ERROR:
			break;
		case RIBD:
			break;
		case RIB:
		{
			if (prophetPkt->getPreds().size() == 0){
				abort = true;
			}
		}
			break;
		case Bundle_Offer:
		{
			if (prophetPkt->getBndlmeta().size() == 0){
				abort = true;
			}
		}
			break;
		case Bundle_Response:
		{
			if (prophetPkt->getBndlmeta().size() == 0){
				abort = true;
			}
		}
			break;
		case Bundle:
		{
			if (prophetPkt->getEncapsulatedPacket() == NULL){
				abort = true;
			}
		}
			break;
		case Bundle_Ack:
			break;
		default:
			opp_error("Unknown Prophetv2MessageKinds(ProphetNCV2::abortConnection)");
			break;
	}

	return abort;
}

void ProphetNCV2::updateContactWhenInit(ProphetNCPkt *prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind)
{
	/*
	 * Last we update stats about contact
	 */
	if (recordContactStats){
		iteratorContactID iterator3 = indexContactID.find(prophetPkt->getSrcAddr());
		if (iterator3 == indexContactID.end()){
			/*
			 * No entry found for this address, it is impossible
			 */
			opp_error("looking for an non-existent contact(ProphetNCV2::executeInitiatorRole)");
		}else {
			if (iterator3->second.size()>1){
				contact.setRepeatedContact(true);
			}
		}
		contact.setState(kind);
		indexContactStats[contactID] = contact;
	}
}

void ProphetNCV2::updateContactWhenList(ProphetNCPkt *prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind)
{
	/*
	 * Last we update stats about contact
	 */
	if (recordContactStats){
		iteratorContactID iterator3 = indexContactID.find(prophetPkt->getSrcAddr());
		if (iterator3 == indexContactID.end()){
			/*
			 * No entry found for this address, it is impossible
			 */
			opp_error("looking for an non-existent contact(ProphetNCV2::executeListenerRole)");
		}else {
			if (iterator3->second.size()>1){
				contact.setRepeatedContact(true);
			}
		}
		contact.setState(kind);
		indexContactStats[contactID] = contact;
	}
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

	if (!nbrRepeatedContact.empty()){
		histMaxRepeatedContact.setName("Histogram for max nbr repeated contact");
		std::map<LAddress::L3Type, int>::iterator it;
		std::vector<int> repeatedNTimes(maxForRC,0);
		for (it = nbrRepeatedContact.begin(); it != nbrRepeatedContact.end(); it++){
			histMaxRepeatedContact.collect(it->second);
			for (int i = 0; i < it->second; ++i) {
				int index = 0;
				if (i < maxForRC){
					index = i;
				}else {
					index = maxForRC - 1;
				}
				int tmp = repeatedNTimes[index];
				repeatedNTimes[index] = tmp +1;
			}
		}
		histMaxRepeatedContact.recordAs("Histogram for max nbr repeated contact");

		for (unsigned int i = 0; i < repeatedNTimes.size(); ++i) {
			stringstream flux1;
			flux1 << i+1;
			std::string tmpStr = "Bars for #repeated "+ flux1.str();
			recordScalar(tmpStr.c_str(),repeatedNTimes[i]);
		}

		std::map<int, std::list<double> >::iterator it2;
		std::vector<int> repartitionForMax(cutPoitsCDF.size()+1,0);
		bool incremented = false;
		for (it2 = contactDurForRC.begin(); it2 != contactDurForRC.end(); it2++){
			std::list<double> durationList = it2->second;
			std::vector<int> repartition(cutPoitsCDF.size()+1,0);

			for (std::list<double>::iterator it3 = durationList.begin(); it3 != durationList.end(); it3++){
				bool found = false;
				for (int i = 0; i < cutPoitsCDF.size(); ++i) {
					if (i == 0){
						if (*it3 <= cutPoitsCDF[i]){
							repartition[0]++;
							found = true;
						}
					}else if (i == cutPoitsCDF.size()-1){
						if (*it3 > cutPoitsCDF[i]){
							repartition[cutPoitsCDF.size()]++;
							found = true;
						}
					}else{
						if ((*it3 > cutPoitsCDF[i-1]) && (*it3 <= cutPoitsCDF[i])){
							repartition[i]++;
							found = true;
						}
					}
					if (found){
						break;
					}
				}
			}

			for (int i = 0; i < repartition.size(); ++i) {
				stringstream flux1;
				if (it2->first < maxForRC){
					flux1 << it2->first;
					std::string tmpStr = "#repeated "+ flux1.str();
					flux1.str(std::string());
					if (i == 0){
						flux1 << cutPoitsCDF[i];
						tmpStr = "LQ"+flux1.str()+" "+tmpStr;
						recordScalar(string("Contact CDF "+tmpStr).c_str(),repartition[i]);
					}else if (i == repartition.size()-1){
						flux1 << cutPoitsCDF[i-1];
						tmpStr = "G"+flux1.str()+" "+tmpStr;
						recordScalar(string("Contact CDF "+tmpStr).c_str(),repartition[i]);
					}else{
						flux1 << cutPoitsCDF[i-1];
						stringstream flux2;
						flux2 << cutPoitsCDF[i];
						tmpStr = "G"+flux1.str()+"LQ"+flux2.str()+" "+tmpStr;
						recordScalar(string("Contact CDF "+tmpStr).c_str(),repartition[i]);
					}
				}else{
					incremented = true;
					repartitionForMax[i]+=repartition[i];
				}
			}
		}

		if (incremented){
			for (int i = 0; i < repartitionForMax.size(); ++i) {
				stringstream flux1;
				flux1 << maxForRC;
				std::string tmpStr = "#repeated "+ flux1.str();
				flux1.str(std::string());
				if (i == 0){
					flux1 << cutPoitsCDF[i];
					tmpStr = "LQ"+flux1.str()+" "+tmpStr;
					recordScalar(string("Contact CDF "+tmpStr).c_str(),repartitionForMax[i]);
				}else if (i == repartitionForMax.size()-1){
					flux1 << cutPoitsCDF[i-1];
					tmpStr = "G"+flux1.str()+" "+tmpStr;
					recordScalar(string("Contact CDF "+tmpStr).c_str(),repartitionForMax[i]);
				}else{
					flux1 << cutPoitsCDF[i-1];
					stringstream flux2;
					flux2 << cutPoitsCDF[i];
					tmpStr = "G"+flux1.str()+"LQ"+flux2.str()+" "+tmpStr;
					recordScalar(string("Contact CDF "+tmpStr).c_str(),repartitionForMax[i]);
				}
			}
		}

		incremented = false;
		repartitionForMax = std::vector<int>(cutPoitsCDF.size()+1,0);
		for (it2 = interContactDurForRC.begin(); it2 != interContactDurForRC.end(); it2++){
			std::list<double> durationList = it2->second;
			std::vector<int> repartition(cutPoitsCDF.size()+1,0);

			for (std::list<double>::iterator it3 = durationList.begin(); it3 != durationList.end(); it3++){
				bool found = false;
				for (int i = 0; i < cutPoitsCDF.size(); ++i) {
					if (i == 0){
						if (*it3 <= cutPoitsCDF[i]){
							repartition[0]++;
							found = true;
						}
					}else if (i == cutPoitsCDF.size()-1){
						if (*it3 > cutPoitsCDF[i]){
							repartition[cutPoitsCDF.size()]++;
							found = true;
						}
					}else{
						if ((*it3 > cutPoitsCDF[i-1]) && (*it3 <= cutPoitsCDF[i])){
							repartition[i]++;
							found = true;
						}
					}
					if (found){
						break;
					}
				}
			}

			for (int i = 0; i < repartition.size(); ++i) {
				stringstream flux1;
				if (it2->first < maxForRC){
					flux1 << it2->first;
					std::string tmpStr = "#repeated "+ flux1.str();
					flux1.str(std::string());
					if (i == 0){
						flux1 << cutPoitsCDF[i];
						tmpStr = "LQ"+flux1.str()+" "+tmpStr;
						recordScalar(string("InterContact CDF "+tmpStr).c_str(),repartition[i]);
					}else if (i == repartition.size()-1){
						flux1 << cutPoitsCDF[i-1];
						tmpStr = "G"+flux1.str()+" "+tmpStr;
						recordScalar(string("InterContact CDF "+tmpStr).c_str(),repartition[i]);
					}else{
						flux1 << cutPoitsCDF[i-1];
						stringstream flux2;
						flux2 << cutPoitsCDF[i];
						tmpStr = "G"+flux1.str()+"LQ"+flux2.str()+" "+tmpStr;
						recordScalar(string("InterContact CDF "+tmpStr).c_str(),repartition[i]);
					}
				}else{
					incremented = true;
					repartitionForMax[i]+=repartition[i];
				}
			}
		}

		if (incremented){
			for (int i = 0; i < repartitionForMax.size(); ++i) {
				stringstream flux1;
				flux1 << maxForRC;
				std::string tmpStr = "#repeated "+ flux1.str();
				flux1.str(std::string());
				if (i == 0){
					flux1 << cutPoitsCDF[i];
					tmpStr = "LQ"+flux1.str()+" "+tmpStr;
					recordScalar(string("InterContact CDF "+tmpStr).c_str(),repartitionForMax[i]);
				}else if (i == repartitionForMax.size()-1){
					flux1 << cutPoitsCDF[i-1];
					tmpStr = "G"+flux1.str()+" "+tmpStr;
					recordScalar(string("InterContact CDF "+tmpStr).c_str(),repartitionForMax[i]);
				}else{
					flux1 << cutPoitsCDF[i-1];
					stringstream flux2;
					flux2 << cutPoitsCDF[i];
					tmpStr = "G"+flux1.str()+"LQ"+flux2.str()+" "+tmpStr;
					recordScalar(string("InterContact CDF "+tmpStr).c_str(),repartitionForMax[i]);
				}
			}
		}

		std::map<LAddress::L3Type, std::list<double> >::iterator it5;
		for (it5 = contactDurByAddr.begin(); it5 != contactDurByAddr.end(); it5++){
			std::list<double> durationList = it5->second;
			double totalDuration = 0;
			for (std::list<double>::iterator it3 = durationList.begin(); it3 != durationList.end(); it3++){
				totalDuration+=*it3;
			}
			contactDurHist.collect(totalDuration);
		}
		contactDurHist.recordAs("Histogram for total contact duration");

		for (it5 = interContactDurByAddr.begin(); it5 != interContactDurByAddr.end(); it5++){
			std::list<double> durationList = it5->second;
			double totalDuration = 0;
			for (std::list<double>::iterator it3 = durationList.begin(); it3 != durationList.end(); it3++){
				totalDuration+=*it3;
				interContactDuration.collect(*it3);
			}
			interContactDurHist.collect(totalDuration);
		}
		interContactDurHist.recordAs("Histogram for total interContact duration");
		interContactDuration.recordAs("Duration between RC");
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
