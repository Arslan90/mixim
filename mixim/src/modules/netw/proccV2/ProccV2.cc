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

#include "ProccV2.h"
#include "FindModule.h"
#include "VPApOpp.h"
#include "VEHICLEpOpp.h"
#include "algorithm"

Define_Module(ProccV2);

void ProccV2::initialize(int stage)
{
	DtnNetwLayer::initialize(stage);
	if (stage==0){
		DefineNodeClass();

		/*
		 * Initialization of ProphetV2 parameters
		 */
		PMinThreshold = par("PMinThreshold").doubleValue();
		Alpha = par("Alpha").doubleValue();
		GAMMA = par("GAMMA").doubleValue();
		Delta = par("Delta").doubleValue();
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

	    classEvolution.setName("Evolution of Node Class");
	    autorizeToAgeEvolution.setName("Evolution of Aging Authorization");


	    nbrSuccessfulContact = 0;
        nbrFailedContactBeforeRIB = 0;
        nbrFailedContactAtRIB= 0;
        nbrFailedContactAtBundle_Offer = 0;
        nbrFailedContactAtBundle_Response = 0;
        contactState = std::map<LAddress::L3Type, Prophetv2MessageKinds>();

        initAllClassifier();

        maxForRC = 5;

        nbrContactsForRCVect.setName("Evolution of nbrContact between RC");

        predsForRC.setName("Evolution of Preds for RC GQ Max");

        delayed = par("delayed");

	}
	else if (stage==1){
//		preds.insert(std::pair<LAddress::L3Type,double>(myNetwAddr,1));
	}
	else if (stage==2){
		switch (myClass) {
			case VPA:
				canIAge = false;
				convergeCastTo = myNetwAddr;
				preds.insert(std::pair<LAddress::L3Type, double>(convergeCastTo,1));
				classEvolution.record(5.0);
				autorizeToAgeEvolution.record(-1.0);
				break;
			case Vehicle_Type_I:
				canIAge = true;
				convergeCastTo = vpaDestAddr();
				preds.insert(std::pair<LAddress::L3Type, double>(convergeCastTo,0));
				classEvolution.record(1.0);
				autorizeToAgeEvolution.record(1.0);
				break;
			case Vehicle_Type_II:
				canIAge = false;
				convergeCastTo = vpaDestAddr();
				preds.insert(std::pair<LAddress::L3Type, double>(convergeCastTo,0));
				classEvolution.record(-1.0);
				autorizeToAgeEvolution.record(-1.0);
				break;
		}
	}
}

/*******************************************************************
**
** 							Methods related to predictions
**
********************************************************************/
void ProccV2::updateDataWhenCCN(const LAddress::L3Type convergeCastAddr){
	/*
	 * before calculating preds, we must age preds.
	 */
	predsIterator it= preds.find(convergeCastAddr);
	// some basic test
	if (it == preds.end()){
		opp_error("even if no history of Preds, preds for ConvergeCast addr must be previously setted");
	}

	// updating class
	myClass = Vehicle_Type_I;
	classEvolution.record(1.0);

	// refreshing all preds and other data
	preds[convergeCastTo] = 1;
	lastEncouterTime[convergeCastTo] = simTime().dbl();
}

void ProccV2::updateDataFor(const LAddress::L3Type convergeCastAddr, const nodeClass encounteredClass, std::map<LAddress::L3Type,double> Bpreds)
{
	double BpredsForCCN, myPredsForCCN;
	nodeClass BClass = encounteredClass;
	/*
	 * before calculating preds, we must age preds.
	 */
	ageDeliveryPreds();
	predsIterator it= preds.find(convergeCastAddr);
	// some basic test
	if (it != preds.end()){
		myPredsForCCN = it->second;
	}else {
		opp_error("even if no history of Preds, preds for ConvergeCast addr must be previously setted");
	}

	if (myClass == Vehicle_Type_II){
		std::vector<double> convergeCastToHistoricPreds;
		predsIterator it = Bpreds.find(convergeCastAddr);
		if (it != Bpreds.end()){
			if (BClass == Vehicle_Type_I){
				BpredsForCCN = it->second;
				myPredsForCCN = Alpha * myPredsForCCN + (1-Alpha) * BpredsForCCN;
				preds[convergeCastTo] = myPredsForCCN;
			}
		}else{
			opp_error("preds for ConvergeCast addr must be previously setted");
		}
	}
}

void ProccV2::DefineNodeClass()
{
	cModule* parentModule = this->getParentModule();
	if (parentModule->findSubmodule("appl")!=-1){
		VPApOpp* VPAModule = FindModule<VPApOpp*>::findSubModule(parentModule);
		VEHICLEpOpp* VehicleModule = FindModule<VEHICLEpOpp*>::findSubModule(parentModule);

		if (VPAModule != NULL){
			myClass = VPA;
		} else if (VehicleModule != NULL){
			myClass = Vehicle_Type_II;
		} else {
			opp_error("ProccV2::DefineNodeClass() - Unable to define NodeClass please check existence of appl module in NED file");
		}
	}
}

int ProccV2::vpaDestAddr()
{
	int vpaDestAddr = -2;
	cModule *systemModule = this->getParentModule();
	while (systemModule->getParentModule() !=NULL){
		systemModule = systemModule->getParentModule();
	}
	int numberVPA = systemModule->par("numeroNodes");
	cModule *vpa = systemModule->getSubmodule("VPA", numberVPA-1);
	if (vpa!=NULL){
		cModule *netw = vpa->getSubmodule("netw");
		if (netw!=NULL){
			BaseNetwLayer *baseNetw = check_and_cast<BaseNetwLayer*>(netw);
			vpaDestAddr = baseNetw->getMyNetwAddr();
		}
	}

	if (vpaDestAddr == -2){
		opp_error("Unable to get address of ConvergeCastNode");
	}

	return vpaDestAddr;
}

void ProccV2::ageDeliveryPreds()
{
	bool belowThreshold = false;
	predsIterator it2;
	double time = simTime().dbl(), myPreds;
	if ((canIAge) && (myClass == Vehicle_Type_I)){
		int timeDiff = 0;
		if (lastAgeUpdate == 0){
			std::map<LAddress::L3Type, double>::iterator it3 = endContactTime.find(convergeCastTo);
			if (it3 != endContactTime.end()){
				timeDiff = int (time-it3->second)/secondsInTimeUnit;
			}else{
				opp_error("After encountering the CCN node we must have set the ending time of the contact");
			}
		}else{
			timeDiff = int (time-lastAgeUpdate)/secondsInTimeUnit;
		}

		if (timeDiff==0){
			return;
		}else {
			double mult = std::pow(GAMMA, timeDiff);
			std::vector<LAddress::L3Type> predsToDelete;

			for (predsIterator it=preds.begin();it!=preds.end();it++){
				if (it->first!=convergeCastTo){
					continue;
				}

				myPreds = it->second * mult;
				if (it->second < PMinThreshold){
					myPreds = 0;
					belowThreshold = true;
				}
				preds[it->first] = myPreds;

				if (belowThreshold){
					myClass = Vehicle_Type_II;
					canIAge = false;
					classEvolution.record(-1.0);
					autorizeToAgeEvolution.record(-1.0);
				}
			}
			lastAgeUpdate = simTime().dbl();
		}
	}
}

void ProccV2::update(Procc *prophetPkt)
{
	nodeClass ndClass = static_cast<nodeClass>(prophetPkt->getNodeClass());
	updateDataFor(convergeCastTo, ndClass ,prophetPkt->getPreds());
	recordPredsStats();
}

void ProccV2::partialUpdate(Procc *prophetPkt)
{
	nodeClass ndClass = static_cast<nodeClass>(prophetPkt->getNodeClass());
	updateDataFor(convergeCastTo, ndClass ,prophetPkt->getPreds());
	recordPredsStats();
}

/*******************************************************************
**
** 							Core methods
**
********************************************************************/

void ProccV2::handleLowerMsg(cMessage* msg)
{
	std::pair<SimpleContactStats, std::set<SimpleContactStats>::iterator > pair;
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;

    Procc *prophetPkt = check_and_cast<Procc *>(m->decapsulate());

    if (isEquiped){

    if ((prophetPkt->getDestAddr()==LAddress::L3BROADCAST)||(prophetPkt->getDestAddr()==myNetwAddr)){

		switch (prophetPkt->getKind()) {
			case HELLO:
				break;
			case ERROR:
				break;
			case RIBD:
				break;
			case RIB:
				{
					// first step : updating preds
					executeListenerRole(RIB,prophetPkt);
					// second step : starting of Bundle_Offer phase
				}
				break;
			case Bundle_Offer:
				{
					executeInitiatorRole(Bundle_Offer,prophetPkt);
				}
				break;
			case Bundle_Response:
				{
					if (myNetwAddr != convergeCastTo){
						executeListenerRole(Bundle_Response,prophetPkt);
					}
				}
				break;
			case Bundle:
				{
					/*
					 * Note : this phase is responsible of triggering the executeInitiatorRole(Bundle_Ack,prophetPkt);
					 */
					executeInitiatorRole(Bundle,prophetPkt);
				}
				break;
			case Bundle_Ack:
				{
					if (myNetwAddr != convergeCastTo){
						executeListenerRole(Bundle_Ack,prophetPkt);
					}
				}
				break;
			default:
				opp_error("Unknown Prophetv2MessageKinds when calling HandleLowerMsg()");
				break;
		}
    }

    }

    delete prophetPkt;
    delete msg;
}

void ProccV2::handleLowerControl(cMessage* msg)
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

					if ((myClass != VPA) && (addr == convergeCastTo)){
						updateDataWhenCCN(addr);
					}

					Procc* emulatedPkt;
					emulatedPkt = prepareProphet(RIB,addr,myNetwAddr);

					if (recordContactStats){
						unsigned long contactID = startRecordingContact(addr,time);
						emulatedPkt->setContactID(contactID);
					}

					/** Starting IEP Phase					*/

					/*
					 * We emulate the reception of a prophetPkt from the other node with contactID as a serial
					 * calculated by startRecoringContact
					 */
					executeInitiatorRole(RIB,emulatedPkt);
					delete emulatedPkt;
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

				if ((myClass != VPA) && (addr == convergeCastTo)){
					if (myClass == Vehicle_Type_I){
						canIAge = true;
						autorizeToAgeEvolution.record(1.0);
					} else {
						opp_error("if the encountered node is the convergeCastNode, current node must be a Type_I");
					}
				}
			}
			break;
	}

	}
	delete msg;
}

void ProccV2::executeInitiatorRole(short  kind, Procc *prophetPkt)
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
			opp_error("looking for an non-existent contact(ProphetV2::executeInitiatorRole)");
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
			int predSize = ((sizeof(int) + sizeof(double) + 16 ) * predToSend.size()) * 8; // to express the size in bits unit

			if (predSize > dataLength) {
				shouldFragment = true;
			}

			if ((dontFragment) || (!shouldFragment)){
				Procc *ribPkt;
				ribPkt = prepareProphet(RIB, myNetwAddr, prophetPkt->getSrcAddr(), NULL, &predToSend);
				ribPkt->setContactID(prophetPkt->getContactID());
				ribPkt->setFragmentFlag(false);
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

				int entrySize = (sizeof(int) + sizeof(double) + 16 ) * 8;
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

					Procc *ribPkt;
					ribPkt = prepareProphet(RIB, myNetwAddr, prophetPkt->getSrcAddr(), NULL, &predFragment);
					ribPkt->setContactID(prophetPkt->getContactID());
					ribPkt->setFragmentFlag(true);
					ribPkt->setFragmentNum(fragmentNum);
					ribPkt->setTotalFragment(nbrFragment);

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
						opp_error("Reception of bundle Meta of unknown type(ProphetV2::executeInitiatorRole)");
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

			Procc *responsePkt;// = new Prophet();
			responsePkt = prepareProphet(Bundle_Response,myNetwAddr,prophetPkt->getSrcAddr(), &bundleToAcceptMeta);
			responsePkt->setContactID(prophetPkt->getContactID());
			if (canITransmit){
				if (delayed == 0){
					sendDown(responsePkt);
				}else{
					sendDelayed(responsePkt,dblrand()*delayed,"lowerLayerOut");
				}

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
			Procc *ackPkt;// = new Prophet();
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
					if (delayed == 0){
						sendDown(ackPkt);
					}else{
						sendDelayed(ackPkt,dblrand()*delayed,"lowerLayerOut");
					}
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

void ProccV2::executeListenerRole(short  kind, Procc *prophetPkt)
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
					opp_error("looking for an non-existent contact that has been inserted(ProphetV2::executeInitiatorRole)");
				}else{
					contact = iterator2->second;
				}
			}else{
				/*
				 * No entry found for this address, it is impossible
				 */
				opp_error("looking for an non-existent contact(ProphetV2::executeInitiatorRole)");
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
			/*
			 * Collecting data
			 */
			updatingL3Received();
			if (recordContactStats){
				contact.setL3Received();
				contact.setPredictionsReceived(contact.getPredictionsReceived()+prophetPkt->getPreds().size());
				updateContactWhenList(prophetPkt, contactID, contact, kind);
			}

			if (!abortConnection(RIB,prophetPkt)){
				update(prophetPkt);
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
							opp_error("definition of Bundle Offer must return a vector of size equal to 3(ProphetV2::executeListenerRole)");
							break;
					}
					bundleToOfferMeta.insert(bundleToOfferMeta.end(),tmp.begin(),tmp.end());
				}
			}

			// Decide if we have to fragment predictions in order to send them
			bool shouldFragment = false;
			int bundleMetaSize = ((sizeof(BundleMeta) + 8 ) * bundleToOfferMeta.size()) * 8; // to express the size in bits unit

			if (bundleMetaSize > dataLength) {
				shouldFragment = true;
			}

			if ((dontFragment) || (!shouldFragment)){

				/*
				 * Step 2 : Sending the ProphetPckt
				 */

				Procc *offerPkt;// = new Prophet();
				offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,prophetPkt->getSrcAddr(),&bundleToOfferMeta);
				offerPkt->setContactID(prophetPkt->getContactID());
				offerPkt->setFragmentFlag(false);
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

				int entrySize = (sizeof(BundleMeta) + 8 ) * 8;
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

					Procc *offerPkt;// = new Prophet();
					offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,prophetPkt->getSrcAddr(),&bndlMetaFragment);
					offerPkt->setContactID(prophetPkt->getContactID());
					offerPkt->setFragmentFlag(true);
					offerPkt->setFragmentNum(fragmentNum);
					offerPkt->setTotalFragment(nbrFragment);
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

			Procc *bundlePkt;

			if (abortConnection(Bundle_Response,prophetPkt)){
				/*
				 * No Bundle to transmit, send a prophet msg with NULL pointer instead of an encapsulated bundle
				 */
				bundlePkt = prepareProphet(Bundle,myNetwAddr,prophetPkt->getSrcAddr(),NULL,NULL,NULL);
				bundlePkt->setContactID(prophetPkt->getContactID());
				if (canITransmit){
					if (delayed == 0){
						sendDown(bundlePkt);
					}else{
						sendDelayed(bundlePkt,dblrand()*delayed,"lowerLayerOut");
					}
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
//							bundlePkt->setBitLength(headerLength);
							if (canITransmit){
								if (delayed == 0){
									sendDown(bundlePkt);
								}else{
									sendDelayed(bundlePkt,dblrand()*delayed,"lowerLayerOut");
								}
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

Procc *ProccV2::prepareProphet(short  kind, LAddress::L3Type srcAddr,LAddress::L3Type destAddr, std::list<BundleMeta> *meta, std::map<LAddress::L3Type,double> *preds, WaveShortMessage *msg)
{

	int realPktLength = 0, msgSize = 0, metaLength = 0, predsLength = 0;
	Procc *prophetMsg = new Procc();
	prophetMsg->setNodeClass(myClass);
	prophetMsg->setKind(kind);
	prophetMsg->setSrcAddr(srcAddr);
	prophetMsg->setDestAddr(destAddr);

	realPktLength = sizeof(kind)+sizeof(srcAddr)+sizeof(destAddr)+sizeof(unsigned long) * 2 + sizeof(int);
	if (meta!=NULL){
		prophetMsg->setBndlmeta(*meta);
		metaLength = (sizeof(BundleMeta) + 8 ) * meta->size();
		realPktLength += metaLength;
	}

	if (preds!=NULL){
		prophetMsg->setPreds(*preds);
		predsLength = (sizeof(int ) + sizeof(double) + 16 ) * preds->size();
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

std::vector<std::list<BundleMeta> >ProccV2::defineBundleOffer(Procc *prophetPkt)
{
	LAddress::L3Type encounterdNode = prophetPkt->getSrcAddr();
	nodeClass BClass = static_cast<nodeClass>(prophetPkt->getNodeClass());
	double BpredsForCCN, mypredsForCCN;

	predsIterator predsIt = prophetPkt->getPreds().find(convergeCastTo);
	if (predsIt != prophetPkt->getPreds().end()){
		BpredsForCCN = predsIt->second;
	}else{
		if (BClass == VPA){
			if (encounterdNode == convergeCastTo){
				opp_error("The ConvergeCastNode must have a prediction for itself (set to 1)");
			}else{
				opp_error("Encountered VPA must be the current ConvergeCastNode, which is not the case");
			}
		}else{
			opp_error("Encountered Vehicle must have a prediction to the ConvergeCastNode");
		}
	}


	std::map<LAddress::L3Type, double> concernedPreds = std::map<LAddress::L3Type, double>();
	std::vector<std::pair<LAddress::L3Type, double>	> sortedPreds;
	std::vector<std::list<BundleMeta> > allBundleMeta;

	std::list<BundleMeta> directBundleToOffer = std::list<BundleMeta>();
	std::list<BundleMeta> otherBundleToOffer = std::list<BundleMeta>();
	std::list<BundleMeta> ackToOffer = std::list<BundleMeta>();

	if (myClass != VPA){
		// step 1 : check if we have any bundle that are addressed to @encouterdNode

		bundlesIndexIterator bundlesIt = bundlesIndex.find(encounterdNode);
		if (bundlesIt != bundlesIndex.end()){
			innerIndexMap innerMap(bundlesIt->second);
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
		predsIt = preds.find(convergeCastTo);
		if (predsIt != preds.end()){
			mypredsForCCN = predsIt->second;
		}else{
			opp_error("Current Node must have a prediction to the ConvergeCastNode");
		}

		if (mypredsForCCN != 1 ){
			if ((BpredsForCCN == 1) || ((BpredsForCCN > mypredsForCCN + Delta))){
				bundlesIndexIterator it2 = bundlesIndex.find(convergeCastTo);
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
		opp_error("definition of Bundle Offer must return a vector of size equal to 3(ProphetV2::defineBundleOffer)");
	}

	return allBundleMeta;
}

bool ProccV2::abortConnection(short  kind, Procc *prophetPkt)
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
			opp_error("Unknown Prophetv2MessageKinds(ProphetV2::abortConnection)");
			break;
	}

	return abort;
}

void ProccV2::updateContactWhenInit(Procc *prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind)
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
			opp_error("looking for an non-existent contact(ProphetV2::executeInitiatorRole)");
		}else {
			if (iterator3->second.size()>1){
				contact.setRepeatedContact(true);
			}
		}
		contact.setState(kind);
		indexContactStats[contactID] = contact;
	}
}

void ProccV2::updateContactWhenList(Procc *prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind)
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
			opp_error("looking for an non-existent contact(ProphetV2::executeListenerRole)");
		}else {
			if (iterator3->second.size()>1){
				contact.setRepeatedContact(true);
			}
		}
		contact.setState(kind);
		indexContactStats[contactID] = contact;
	}
}

void ProccV2::finish()
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
			}
			interContactDurHist.collect(totalDuration);
		}
		interContactDurHist.recordAs("Histogram for total interContact duration");
	}
}

/*******************************************************************
**
** 							Methods for collecting  datas & stats
**
********************************************************************/

void ProccV2::recordPredsStats()
{
	double min = DBL_MAX, max = DBL_MIN, mean = 0, sum = 0, variance = 0, varianceSum = 0;

	for (predsIterator it=preds.begin(); it!=preds.end();it++){
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




	// recording values
	predsMin.record(min);
	predsMax.record(max);
	predsMean.record(mean);
	predsVariance.record(variance);

	std::map<LAddress::L3Type, int >::iterator it;
	std::map<LAddress::L3Type, cOutVector* >::iterator it3;

	double total = 0;
	int nbrContactForRC = 0;
	for (std::map<LAddress::L3Type, int >::iterator it = nbrRepeatedContact.begin(); it != nbrRepeatedContact.end(); it++){
		if (it->second >= maxForRC){
			nbrContactForRC++;
			predsIterator it2 = preds.find(it->first);
			if (it2 != preds.end()){
				total+=it2->second;
			}
		}
	}
	if (nbrContactForRC > 0){
		predsForRC.record(total/double(nbrContactForRC));
	}else{
		predsForRC.record(0);
	}

}

void ProccV2::recordEndContactStats(LAddress::L3Type addr, double time)
{
	double duration = time - contacts.find(addr)->second;
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

void ProccV2::recordRecontactStats(LAddress::L3Type addr, double time)
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

void ProccV2::updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind)
{
	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		contactState.erase(addr);
	}
	contactState.insert(std::pair<LAddress::L3Type, Prophetv2MessageKinds>(addr,kind));
}

void ProccV2::classify(SimpleContactStats* newContact)
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

void ProccV2::recordAllClassifier()
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


void ProccV2::initAllClassifier()
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

/*******************************************************************
**
** 							Unused functions
**
********************************************************************/

ProccV2::~ProccV2()
{

}
