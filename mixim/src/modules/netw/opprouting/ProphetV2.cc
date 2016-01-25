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

#include "ProphetV2.h"
#include "multiFunctions.h"
#include "ApplOppControlInfo.h"

Define_Module(ProphetV2);

void ProphetV2::initialize(int stage)
{
	BaseNetwLayer::initialize(stage);
	if (stage==0){
		/*
		 * L3Address will be initialized by BaseNetwLayer::initialize(1);
		 */
//		myNetwAddr = LAddress::L3Type( getId() );

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
		bundlesStructureSize = par("storageSize");
		if (bundlesStructureSize<=0){
			opp_error("Size of the structure that store bundles can not be negative");
		}
		bundles = std::list<WaveShortMessage*>();


		withAck = par("withAck");
		ackStructureSize = par("ackSize");
		if (ackStructureSize<=0){
			opp_error("Size of the structure that store acks can not be negative");
		}
		acks = std::list<BundleMeta>();
		acksIndex = std::map<unsigned long,BundleMeta>();

		withTTL = par("withTTL").boolValue();
		ttl = par("ttl");
		nbrDeletedWithTTL = 0;

		equipedVehPc = par("equipedVehPc").doubleValue();
		if ((equipedVehPc < 0) || (equipedVehPc > 1)){
			opp_error("Pourcentage of equiped vehicle is wrong, please correct it");
		}
		if (equipedVehPc == 1){
			isEquiped = true;
		}else {
			double tmp = uniform(0,1);
			if (tmp <= equipedVehPc){
				isEquiped = true;
			}else {
				isEquiped = false;
			}
		}

		maxPcktLength = par("MTU"); // MTU is expressed in bytes unit
		maxPcktLength*=8;

		dontFragment = par("DF").boolValue();

		const char* cutPoints = par("cutPoints").stringValue();
		char* cutPointsChar = strdup(cutPoints);

		char* tmp = strtok (cutPointsChar,";");
		double cutPointValue = 0;
		std::istringstream iss(tmp);
		iss >> cutPointValue;
		cutPoitsCDF.push_back(cutPointValue);

		while (tmp != NULL)	{
		    tmp = strtok (NULL, ";");
		    if (tmp != NULL){
		    	cutPointValue = 0;
		    	if (strcmp(tmp,"") != 0){
		    		std::istringstream iss(tmp);
					iss >> cutPointValue;
					cutPoitsCDF.push_back(cutPointValue);
		    	}
		    }
		}

		dataLength = maxPcktLength - headerLength;

		I_Preds = par("I_Preds").doubleValue();

		/*
		 * Collecting data & metrics
		 */

		recordContactStats = par("recordContactStats");

	    nbrL3Sent = 0;
	    nbrL3Received = 0;

	    nbrPredsVector.setName("Number of predictions");
	    predsMean.setName("Mean of predictions");
	    predsMax.setName("Maximum of predictions");
	    predsMin.setName("Minimum of predictions");
	    predsVariance.setName("Variance of predictions");

	    nbrContacts =0;
	    sumOfContactDur = 0.0;
	    contacts = std::map<LAddress::L3Type,double>();
	    contactDurVector.setName("Evolution of contact duration mean");

	    nbrRecontacts = 0;
	    sumOfInterContactDur = 0.0;
	    intercontactDurVector.setName("Evolution of intercontact duration mean");

	    nbrSuccessfulContact = 0;
        nbrFailedContactBeforeRIB = 0;
        nbrFailedContactAtRIB= 0;
        nbrFailedContactAtBundle_Offer = 0;
        nbrFailedContactAtBundle_Response = 0;
        contactState = std::map<LAddress::L3Type, Prophetv2MessageKinds>();

        deletedBundlesWithAck = 0;

        demandedAckedBundle = 0;

        bundlesReceived = 0;

        nbrSimpleContactStats = 0;

        initAllClassifier();

        maxForRC = 5;

        nbrContactsForRCVect.setName("Evolution of nbrContact between RC");

        delayed = par("delayed");

	}
	else if (stage==1){
		preds.insert(std::pair<LAddress::L3Type,double>(myNetwAddr,1));
		predsForRC.setName("Evolution of Preds for RC GQ Max");
	}
	else if (stage==2){
		cModule *systemModule = this->getParentModule();
		while (systemModule->getParentModule() !=NULL){
			systemModule = systemModule->getParentModule();
		}
		int numberVPA = systemModule->par("numeroNodes");
		predsMeanCommunities = new cOutVector[numberVPA];

		for (int i = 0; i < numberVPA; ++i) {
			cModule *vpa = systemModule->getSubmodule("VPA", i);
			if (vpa!=NULL){
				cModule *netw = vpa->getSubmodule("netw");
				if (netw!=NULL){
					BaseNetwLayer *baseNetw = check_and_cast<BaseNetwLayer*>(netw);
					int vpaAddr = baseNetw->getMyNetwAddr();
					addrCommunities.push_back(vpaAddr);
					stringstream flux1,flux2;
					flux1 << i;
					flux2 << myNetwAddr;
					std::string tmpStr = "Mean Predictions for Community "+ flux1.str();
					const char *tmp = tmpStr.c_str();
					predsMeanCommunities[i].setName(tmp);
				}
			}
		}
	}
}

/*******************************************************************
**
** 							Methods related to predictions
**
********************************************************************/

void ProphetV2::updateDeliveryPredsFor(const LAddress::L3Type BAdress)
{
	if (BAdress == myNetwAddr){
		opp_warning("Cannot Update Delivery Predictions for its own Address(ProphetV2::updateDeliveryPredsFor)");
	}else{
		double PEnc,lastEncTime, predsForB;
			/*
			 * before calculating predsForB, we must age preds.
			 */
			ageDeliveryPreds();
			double encTime = simTime().dbl();
			predsIterator it= preds.find(BAdress);
			predsIterator it2= lastEncouterTime.find(BAdress);
			if (it==preds.end()){

				/*
				 * if iterator is equal map.end(), it means that there is no entry for BAdress in preds
				 * so lastEncTime equal 0
				 */
				predsForB = PFirstContact;
				if (it2==lastEncouterTime.end()){
					lastEncTime = 0;
				}else {
					lastEncTime = it2->second;
				}

			}else {

				/*
				 * if iterator is not equal map.end(), it means that there is an entry for BAdress in preds
				 */
				predsForB = it->second;
				if (it2==lastEncouterTime.end()){
					lastEncTime = 0;
				}else {
					lastEncTime = it2->second;
				}

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
			preds[BAdress] = predsForB;
			lastEncouterTime[BAdress] = encTime;
	}
}

void ProphetV2::updateTransitivePreds(const LAddress::L3Type BAdress, std::map<LAddress::L3Type,double> Bpreds)
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
//		preds.insert(std::pair<LAddress::L3Type,double>(CAdress,predsForC));
	}
}

void ProphetV2::ageDeliveryPreds()
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
		}

		for (std::vector<LAddress::L3Type>::iterator it2 = predsToDelete.begin(); it2 != predsToDelete.end(); it2++){
			LAddress::L3Type tmp = *it2;
			preds.erase(tmp);
		}

		lastAgeUpdate = simTime().dbl();
	}
}

void ProphetV2::update(Prophet *prophetPkt)
{
	updateDeliveryPredsFor(prophetPkt->getSrcAddr());
	updateTransitivePreds(prophetPkt->getSrcAddr(),prophetPkt->getPreds());
	recordPredsStats();

	for (int i = 0; i < addrCommunities.size(); ++i) {
		double predForCommunityI = 0;
		predsIterator it = preds.find(addrCommunities[i]);
		if (it != preds.end()){
			predForCommunityI = it->second;
		}
		predsMeanCommunities[i].record(predForCommunityI);
	}
}

void ProphetV2::partialUpdate(Prophet *prophetPkt)
{
	updateTransitivePreds(prophetPkt->getSrcAddr(),prophetPkt->getPreds());
	recordPredsStats();

	for (int i = 0; i < addrCommunities.size(); ++i) {
		double predForCommunityI = 0;
		predsIterator it = preds.find(addrCommunities[i]);
		if (it != preds.end()){
			predForCommunityI = it->second;
		}
		predsMeanCommunities[i].record(predForCommunityI);
	}
}

/*******************************************************************
**
** 							Core methods
**
********************************************************************/

void ProphetV2::handleLowerMsg(cMessage* msg)
{
	std::pair<SimpleContactStats, std::set<SimpleContactStats>::iterator > pair;
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;

    Prophet *prophetPkt = check_and_cast<Prophet *>(m->decapsulate());

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
					executeListenerRole(Bundle_Response,prophetPkt);
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
					executeListenerRole(Bundle_Ack,prophetPkt);
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

void ProphetV2::handleUpperMsg(cMessage* msg)
{
	if (isEquiped){
		assert(dynamic_cast<WaveShortMessage*>(msg));
		WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
		storeBundle(upper_msg);
	}
}

void ProphetV2::handleLowerControl(cMessage* msg)
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

//					recordBeginSimplContactStats(addr,time);

					Prophet* emulatedPkt;
					emulatedPkt = prepareProphet(0,addr,myNetwAddr);
//					emulatedPkt->setBitLength(headerLength);

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

//				recordEndSimpleContactStats(addr,time);
			}
			break;
	}

	}
	delete msg;
}

void ProphetV2::handleUpperControl(cMessage* msg)
{
	ApplOppControlInfo* controlInfo = check_and_cast<ApplOppControlInfo* >(msg->getControlInfo());
	int newAddr = controlInfo->getNewSectorNetwAddr();

	bundlesIndexIterator it1;
	innerIndexIterator it2;
	innerIndexMap innerMap;

	for (it1 = bundlesIndex.begin(); it1 != bundlesIndex.end() ; it1++){
		for (it2 = it1->second.begin(); it2 != it1->second.end() ; it2++){
			it2->second->setRecipientAddress(newAddr);
			innerMap.insert(std::pair<unsigned long,WaveShortMessage*>(it2->first,it2->second));
		}
	}

	// we clear all the bundlesIndex container and reconstruct it
	// with one unique destination which correspond to newAddr


	if (!innerMap.empty()){
		bundlesIndex.clear();
		bundlesIndex[newAddr] = innerMap;
	}
}

void ProphetV2::storeBundle(WaveShortMessage *msg)
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
		if (bundlesIndex.count(msg->getRecipientAddress())==0){
			// no entry for msg->getRecipientAddress())
			std::map <unsigned long, WaveShortMessage*> inner_map;
			inner_map.insert(std::pair<unsigned long,WaveShortMessage*>(msg->getSerial(),msg));
			bundlesIndex.insert(std::pair<LAddress::L3Type, innerIndexMap >(msg->getRecipientAddress(),inner_map));
		}else {
			/*
			 * there is a current entry for this address,
			 * we have to add this message to the existing entry
			 */
			bundlesIndexIterator it = bundlesIndex.find(msg->getRecipientAddress());
			it->second.insert(std::pair<unsigned long,WaveShortMessage*>(msg->getSerial(),msg));
		}
	}
}

void ProphetV2::storeACK(BundleMeta meta)
{
	if (!ackExist(meta)){
		if (acks.size()==ackStructureSize){
			unsigned long serial = acks.front().getSerial();
			acksIndex.erase(serial);
			acks.pop_front();
		}

		acksIndex.insert(std::pair<unsigned long, BundleMeta>(meta.getSerial(),meta));
		acks.push_back(meta);

		if (existAndErase(meta)) {
			deletedBundlesWithAck++;
		}
	}
}

void ProphetV2::executeInitiatorRole(short  kind, Prophet *prophetPkt)
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

			if (!shouldFragment){
				Prophet *ribPkt;
				ribPkt = prepareProphet(RIB, myNetwAddr, prophetPkt->getSrcAddr(), NULL, &predToSend);
				ribPkt->setContactID(prophetPkt->getContactID());
				ribPkt->setFragmentFlag(false);
	//			ribPkt->setBitLength(headerLength);
				if (canITransmit){
//					sendDelayed(ribPkt,dblrand()*5,"lowerLayerOut");
//					sendDown(ribPkt);
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

					Prophet *ribPkt;
					ribPkt = prepareProphet(RIB, myNetwAddr, prophetPkt->getSrcAddr(), NULL, &predFragment);
					ribPkt->setContactID(prophetPkt->getContactID());
					ribPkt->setFragmentFlag(true);
					ribPkt->setFragmentNum(fragmentNum);
					ribPkt->setTotalFragment(nbrFragment);

					if (canITransmit){
//						sendDelayed(ribPkt,dblrand()*5,"lowerLayerOut");
//						sendDown(ribPkt);
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

				for (std::list<BundleMeta>::iterator it = prophetPkt->getBndlmeta().begin(); it !=prophetPkt->getBndlmeta().end(); ++it) {
					if (it->getFlags() == Prophet_Enum::Bndl_Accepted){
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

			Prophet *responsePkt;// = new Prophet();
			responsePkt = prepareProphet(Bundle_Response,myNetwAddr,prophetPkt->getSrcAddr(), &bundleToAcceptMeta);
			responsePkt->setContactID(prophetPkt->getContactID());
//			responsePkt->setBitLength(headerLength);
			if (canITransmit){
				sendDown(responsePkt);

				/*
				 * Collecting data
				 */
				updatingL3Sent();
				updatingContactState(prophetPkt->getSrcAddr(),Bundle_Response);

				if (recordContactStats){
					contact.setL3Sent();
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
			Prophet *ackPkt;// = new Prophet();
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
//				ackPkt->setBitLength(headerLength);
				if (canITransmit){
					sendDown(ackPkt);
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

void ProphetV2::executeListenerRole(short  kind, Prophet *prophetPkt)
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
	// if destAddr equals 0 then destAddr is  the sender of the previously received prophet packet
//
//	if (destAddr == 0) {
//		coreEV << "destAddr equal 0 (null) in Bundle_Ack of Initiator Role. destAddr will be recalculated";
//		destAddr = prophetPkt->getSrcAddr();
//	}
//	std::pair<SimpleContactStats, std::set<SimpleContactStats>::iterator > pair;
//	SimpleContactStats contact;
//	if (kind == RIB){
//		contact = getLastSimpleContactStats(destAddr);
//	}else {
//		pair = getSimpleContactStats(destAddr, prophetPkt->getCreationTime().dbl());
//		contact = pair.first;
//	}

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

			if (!shouldFragment){

				/*
				 * Step 2 : Sending the ProphetPckt
				 */

				Prophet *offerPkt;// = new Prophet();
				offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,prophetPkt->getSrcAddr(),&bundleToOfferMeta);
				offerPkt->setContactID(prophetPkt->getContactID());
				offerPkt->setFragmentFlag(false);
	//			offerPkt->setBitLength(headerLength);
				if (canITransmit){
					sendDown(offerPkt);

					/*
					 * Collecting data
					 */
					updatingL3Sent();

					if (recordContactStats){
						contact.setL3Sent();
						contact.setAckSent(contact.getAckSent()+ackToOffer.size());
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

					Prophet *offerPkt;// = new Prophet();
					offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,prophetPkt->getSrcAddr(),&bndlMetaFragment);
					offerPkt->setContactID(prophetPkt->getContactID());
					offerPkt->setFragmentFlag(true);
					offerPkt->setFragmentNum(fragmentNum);
					offerPkt->setTotalFragment(nbrFragment);
		//			offerPkt->setBitLength(headerLength);
					if (canITransmit){
						sendDown(offerPkt);

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
			/*
			 * Collecting data
			 */
			updatingL3Received();
			if (recordContactStats){
				contact.setL3Received();
				updateContactWhenList(prophetPkt, contactID, contact, kind);
			}

			if (withAck){
				/*
				 * 1 step : Check if demanded bundle in bundleResp is currently acked, delete it if it's the case
				 */
				std::list<BundleMeta> bndlMetaToAccept;
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

			Prophet *bundlePkt;

			if (abortConnection(Bundle_Response,prophetPkt)){
				/*
				 * No Bundle to transmit, send a prophet msg with NULL pointer instead of an encapsulated bundle
				 */
				bundlePkt = prepareProphet(Bundle,myNetwAddr,prophetPkt->getSrcAddr(),NULL,NULL,NULL);
				bundlePkt->setContactID(prophetPkt->getContactID());
//				bundlePkt->setBitLength(headerLength);
				if (canITransmit){
					sendDown(bundlePkt);
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
								sendDown(bundlePkt);
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

Prophet *ProphetV2::prepareProphet(short  kind, LAddress::L3Type srcAddr,LAddress::L3Type destAddr, std::list<BundleMeta> *meta, std::map<LAddress::L3Type,double> *preds, WaveShortMessage *msg)
{

	int realPktLength = 0, msgSize = 0, metaLength = 0, predsLength = 0;
	Prophet *prophetMsg = new Prophet();
	prophetMsg->setKind(kind);
	prophetMsg->setSrcAddr(srcAddr);
	prophetMsg->setDestAddr(destAddr);

	realPktLength = sizeof(kind)+sizeof(srcAddr)+sizeof(destAddr)+sizeof(unsigned long) * 2 + sizeof(int);
	if (meta!=NULL){
		prophetMsg->setBndlmeta(*meta);
		metaLength = (sizeof(BundleMeta) + 8 ) * meta->size();
//		realPktLength += sizeof(prophetMsg->getBndlmeta());
		realPktLength += metaLength;
	}
	if (preds!=NULL){
		prophetMsg->setPreds(*preds);
		predsLength = (sizeof(int ) + sizeof(double) + 16 ) * preds->size();
//		realPktLength += sizeof(prophetMsg->getPreds());
		realPktLength+= predsLength;
	}
	if (msg!=NULL){
		prophetMsg->encapsulate(msg);
//		realPktLength += sizeof(msg);
		msgSize = msg->getBitLength();
	}

	realPktLength *= 8;
	realPktLength += msgSize;

	if (realPktLength > 18000){
		//opp_warning("header length is big");
	}

	prophetMsg->setBitLength(realPktLength);

	return prophetMsg;
}

std::vector<std::list<BundleMeta> >ProphetV2::defineBundleOffer(Prophet *prophetPkt)
{
//	std::pair<SimpleContactStats, std::set<SimpleContactStats>::iterator > pair;
//	pair = getSimpleContactStats(prophetPkt->getSrcAddr(),prophetPkt->getCreationTime().dbl());
//	SimpleContactStats contact = pair.first;

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
//			contact.setAckSent();
		}
	}

//	contact.setL3Sent();
//	updateSimpleContactStats(prophetPkt->getSrcAddr(),contact,pair.second);

	allBundleMeta.push_back(directBundleToOffer);
	allBundleMeta.push_back(otherBundleToOffer);
	allBundleMeta.push_back(ackToOffer);

	if (allBundleMeta.size()!=3){
		opp_error("definition of Bundle Offer must return a vector of size equal to 3(ProphetV2::defineBundleOffer)");
	}

	return allBundleMeta;
}

bool ProphetV2::exist(WaveShortMessage *msg)
{
	bool found = false;
	bundlesIndexIterator it = bundlesIndex.find(msg->getRecipientAddress());
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(msg->getSerial());
		if (it2 !=innerMap.end()){
			found = true;
		}
	}
	return found;
}

bool ProphetV2::exist(BundleMeta bndlMeta)
{
	bool found = false;
	bundlesIndexIterator it = bundlesIndex.find(bndlMeta.getRecipientAddress());
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(bndlMeta.getSerial());
		if (it2 !=innerMap.end()){
			found = true;
		}
	}
	return found;
}

bool ProphetV2::existAndErase(BundleMeta bndlMeta)
{
	bool found = false;
	bundlesIndexIterator it = bundlesIndex.find(bndlMeta.getRecipientAddress());
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(bndlMeta.getSerial());
		if (it2 !=innerMap.end()){
			WaveShortMessage* wsm = it2->second;
			innerMap.erase(bndlMeta.getSerial());
			bundles.remove(wsm);
			if (innerMap.empty()){
				bundlesIndex.erase(bndlMeta.getRecipientAddress());
			}else {
				bundlesIndex[bndlMeta.getRecipientAddress()] = innerMap;
			}
			if (wsm->getOwner()==this){
				delete wsm;
			}
			found = true;
		}
	}
	return found;
}

bool ProphetV2::abortConnection(short  kind, Prophet *prophetPkt)
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

bool ProphetV2::ackExist(WaveShortMessage *msg)
{
	bool exist = false;
	if ((withAck)&&(acksIndex.find(msg->getSerial())!=acksIndex.end())){
		exist = true;
	}
	return exist;
}

bool ProphetV2::ackExist(BundleMeta bndlMeta)
{
	bool exist = false;
	if ((withAck)&&(acksIndex.find(bndlMeta.getSerial())!=acksIndex.end())){
		exist = true;
	}
	return exist;
}

void ProphetV2::updateContactWhenInit(Prophet *prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind)
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

void ProphetV2::updateContactWhenList(Prophet *prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind)
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

LAddress::L3Type ProphetV2::getAddressFromName(const char *name)
{
	int addr = 0;
	if (strcmp(name,"")!=0){
		std::istringstream iss(name);
		iss >> addr;
	}else {
		opp_error("getting address from name return NULL (ProphetV2::getAddressFromName)");
	}
	return addr;
}

double ProphetV2::getTimeFromName(const char *name)
{
	double time = 0;
	if (strcmp(name,"")!=0){
		std::istringstream iss(name);
		iss >> time;
	}else {
		opp_error("getting time from name return NULL (ProphetV2::getTimeFromName)");
	}
	return time;
}

void ProphetV2::finish()
{
//	EV << "Sent:     " << nbrL3Sent << endl;
//	EV << "Received: " << nbrL3Received << endl;

	recordScalar("# L3Sent", nbrL3Sent);
	recordScalar("# L3Received", nbrL3Received);

	recordScalar("# of Contacts", nbrContacts);
	recordScalar("# of InterContacts", nbrRecontacts);

//	recordScalar("# failed contacts before RIB", nbrFailedContactBeforeRIB);
//	recordScalar("# failed contacts at RIB", nbrFailedContactAtRIB);
//	recordScalar("# failed contacts at Bundle_Offer", nbrFailedContactAtBundle_Offer);
//	recordScalar("# failed contacts at Bundle_Response", nbrFailedContactAtBundle_Response);
//	recordScalar("# successful contacts", nbrSuccessfulContact);
//
	recordScalar("# Bundles at L3", bundlesReceived);
//
//	if (withAck){
//		recordScalar("# ACKs", acks.size());
//		recordScalar("# DeletedBundles", deletedBundlesWithAck);
//		recordScalar("# DemandedAckedBundles", demandedAckedBundle);
//		recordScalar("# Bundles", bundles.size());
//	}
//
//	if (withTTL){
//		recordScalar("# DeletedBundlesWithTTL", nbrDeletedWithTTL);
//	}
//
//	recordScalar("# nbrSimpleContactStats", nbrSimpleContactStats);
	classifyAll();
	recordAllClassifier();

	while (!bundles.empty()){
		delete bundles.front();
		bundles.pop_front();
	}

	if (!nbrRepeatedContact.empty()){
		histMaxRepeatedContact.setName("Histogram for max nbr repeated contact");
//		int maxRepeated = 0;
		std::map<LAddress::L3Type, int>::iterator it;
//		for (it = nbrRepeatedContact.begin(); it != nbrRepeatedContact.end(); it++){
//			if (it->second > maxRepeated){
//				maxRepeated = it->second;
//			}
//		}

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

		for (int i = 0; i < repeatedNTimes.size(); ++i) {
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

void ProphetV2::recordPredsStats()
{
	// nbPreds = preds.size()-1, because P(X,X) is counted as a prediction for every node X


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

	double total = 0;
	int nbrEntries = 0;
	for (it = nbrRepeatedContact.begin(); it != nbrRepeatedContact.end(); it++){
		if (it->second >= maxForRC){
			int tmp = 0;
			it2 = preds.find(it->first);
			if (it2 != preds.end()){
				tmp = it2->second;
			}
			total+=tmp;
			nbrEntries++;
		}
	}
	double meanPredsForRC = 0;
	if (nbrEntries > 0 ){
		meanPredsForRC = total / (double) nbrEntries;
	}
	predsForRC.record(meanPredsForRC);
}

void ProphetV2::recordBeginContactStats(LAddress::L3Type addr, double time)
{
	// updating nbr contacts
	nbrContacts++;
	// saving the starting time of the contact
	contacts.insert(std::pair<LAddress::L3Type, double>(addr, time));
}

void ProphetV2::recordEndContactStats(LAddress::L3Type addr, double time)
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

void ProphetV2::recordRecontactStats(LAddress::L3Type addr, double time)
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

void ProphetV2::updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind)
{
	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		contactState.erase(addr);
	}
	contactState.insert(std::pair<LAddress::L3Type, Prophetv2MessageKinds>(addr,kind));
}

unsigned long ProphetV2::generateContactSerial(int myAddr, int seqNumber, int otherAddr)
{
	unsigned long serial = 0;
	unsigned long cantorPairForAddresses;

	if (myAddr>otherAddr){
		cantorPairForAddresses = multiFunctions::cantorPairingFunc(otherAddr,seqNumber);
		serial = multiFunctions::cantorPairingFunc(cantorPairForAddresses,myAddr);
	}else{
		cantorPairForAddresses = multiFunctions::cantorPairingFunc(myAddr,seqNumber);
		serial = multiFunctions::cantorPairingFunc(cantorPairForAddresses,otherAddr);
	}

	return serial;
}

unsigned long ProphetV2::startRecordingContact(int addr, double time)
{
	unsigned long contactID = 0, lastContactID = 0;
	std::list<unsigned long> contactIDList;
	SimpleContactStats contact, lastContact;
	iteratorContactID iterator1 = indexContactID.find(addr);
	if (iterator1 == indexContactID.end()){
		/*
		 * No entry found for this address
		 */
		contactID = generateContactSerial(myNetwAddr,1,addr);
		contactIDList.push_back(contactID);
		indexContactID.insert(std::pair<int,std::list<unsigned long> >(addr,contactIDList));

		contact = SimpleContactStats(time);
		contact.setSerial(contactID);
		indexContactStats.insert(std::pair<unsigned long,SimpleContactStats>(contactID,contact));
	}else{
		/*
		 * there is an entry for this address
		 */
		contactIDList = indexContactID[addr];
		if (contactIDList.empty()){
			opp_error("contactIDList cannot be empty if there is an entry for the other addr(ProphetV2::startRecordingContact)");
		}else{
			lastContactID = contactIDList.back();
			iteratorContactStats iterator2 = indexContactStats.find(lastContactID);
			if (iterator2 == indexContactStats.end()){
				opp_error("contactStats cannot be NULL if there is an entry for the contactID (ProphetV2::startRecordingContact)");
			}else{
				lastContact = iterator2->second;
				if (lastContact.hasFinished()){
					/*
					 * precedent contact has finished, create a new contact
					 */
					contactID = generateContactSerial(myNetwAddr,contactIDList.size()+1,addr);
					contactIDList.push_back(contactID);
					indexContactID[addr] = contactIDList;

					contact = SimpleContactStats(time);
					contact.setSerial(contactID);
					indexContactStats[contactID] = contact;
				}else{
					/*
					 * the last contact is potentially the good one, check if it is the same serial or not
					 * by generating a contactID based on contactIDList.size() (not size()+1)
					 */
					contactID = generateContactSerial(myNetwAddr,contactIDList.size(),addr);
					if (lastContactID == contactID){
						/*
						 * the last contact is the good one, no need to add it to the list, only update starting time
						 */
						contact.setStartTime(time);
						indexContactStats[contactID] = contact;
					}else {
						opp_error("Error: generated serial differs from the serial of last contact");
					}
				}
			}
		}
	}

	return contactID;
}

unsigned long ProphetV2::startRecordingContact(int addr, unsigned long contactID)
{
	/*
	 * possibility that we received the RIB msg before receiving the control msg
	 * We must create an entry for this contact
	 */

	std::list<unsigned long> contactIDList = indexContactID[addr];
	contactIDList.push_back(contactID);
	indexContactID[addr] = contactIDList;

	SimpleContactStats contact = SimpleContactStats();
	contact.setSerial(contactID);
	indexContactStats.insert(std::pair<unsigned long,SimpleContactStats>(contactID,contact));

	return contactID;
}

unsigned long ProphetV2::endRecordingContact(int addr, double time)
{
	unsigned long contactID = 0;
	std::list<unsigned long> contactIDList;
	SimpleContactStats contact;
	iteratorContactID iterator1 = indexContactID.find(addr);
	if (iterator1 == indexContactID.end()){
		/*
		 * No entry found for this address, it is impossible
		 */
		opp_error("cannot receive end of contact for a not started contact(ProphetV2::endRecordingContact)");
	}else {
		/*
		 * We return the last contact on the contactIDList
		 */
		contactIDList = iterator1->second;
		contactID = contactIDList.back();
		iteratorContactStats iterator2 = indexContactStats.find(contactID);
		if (iterator2 == indexContactStats.end()){
			opp_error("contactStats cannot be NULL if there is an entry for the contactID (ProphetV2::endRecordingContact)");
		}else{
			contact = iterator2->second;
			/*
			 * we update the end time of the contact
			 */
			contact.setEndTime(time);
			indexContactStats[contactID] = contact;
		}
	}

	return contactID;
}

unsigned long ProphetV2::endRecordingContact(unsigned long contactID, bool hasForecedEnding)
{
	iteratorContactStats iterator = indexContactStats.find(contactID);
	if (iterator == indexContactStats.end()){
		opp_error("contactStats cannot be NULL if there is an entry for the contactID (ProphetV2::endRecordingContact)");
	}else{
		SimpleContactStats contact = iterator->second;
		/*
		 * we update the end time of the contact
		 */
		contact.setEndTime(simTime().dbl());
		contact.setHasForcedEnding(hasForecedEnding);
		indexContactStats[contactID] = contact;
	}

	return contactID;
}

void ProphetV2::classify(SimpleContactStats* newContact)
{
	/*
	 * check if the contact has started, otherwise error
	 */
	if (!newContact->hasStarted()){
		opp_error("impossible to classify a not-started contact");
	}
	/*
	 * check if the contact has ended
	 */
	if (!newContact->hasFinished()){
		endRecordingContact(newContact->getSerial(),true);
	}

	/*
	 * then classify by classifier
	 */

	if (withGlobal){
		Global.update(newContact);
	}
	if ((withSucc)&&(newContact->isSuccessfulContact())){
		Succ.update(newContact);
	}else{
		if (withFail){
			Fail.update(newContact);
		}
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

void ProphetV2::classifyAll()
{
	for(iteratorContactStats it = indexContactStats.begin(); it != indexContactStats.end(); it++){
		SimpleContactStats* contactToClassify = &(it->second);
		classify(contactToClassify);
	}
}

void ProphetV2::recordClassifier(ClassifiedContactStats classifier)
{

	recordScalar(string(classifier.getName()+": # nbrContact").c_str(),classifier.getNbrContacts());
	recordScalar(string(classifier.getName()+": # nbrToDiscard").c_str(),classifier.getNbrToDiscard());

	if (classifier.getNbrContacts()>0){
		double repeatedPourcentage = (double (classifier.getNbrRepeated()) / double (classifier.getNbrContacts())) * 100;
		recordScalar(string(classifier.getName()+": % repeated").c_str(), repeatedPourcentage);
	}

	recordScalar(string(classifier.getName()+": # L3Sent").c_str(),classifier.getL3Sent());
	recordScalar(string(classifier.getName()+": # L3Received").c_str(),classifier.getL3Received());

	recordScalar(string(classifier.getName()+": # BundleSent").c_str(),classifier.getBundleSent());
	recordScalar(string(classifier.getName()+": # BundleReceived").c_str(),classifier.getBundleReceived());

	recordScalar(string(classifier.getName()+": # AckSent").c_str(),classifier.getAckSent());
	recordScalar(string(classifier.getName()+": # AckReceived").c_str(),classifier.getAckReceived());

	recordScalar(string(classifier.getName()+": # PredictionsSent").c_str(),classifier.getPredictionsSent());
	recordScalar(string(classifier.getName()+": # PredictionsReceived").c_str(),classifier.getPredictionsReceived());

	if (classifier.isWithCdf()){
		recordScalar(string(classifier.getName()+": # LQ5").c_str(),classifier.getNbrLq5());
		recordScalar(string(classifier.getName()+": # G5LQ20").c_str(),classifier.getNbrG5Lq20());
		recordScalar(string(classifier.getName()+": # G20LQ50").c_str(),classifier.getNbrG20Lq50());
		recordScalar(string(classifier.getName()+": # G50LQ100").c_str(),classifier.getNbrG50Lq100());
		recordScalar(string(classifier.getName()+": # G100LQ500").c_str(),classifier.getNbrG100Lq500());
		recordScalar(string(classifier.getName()+": # G500LQ1800").c_str(),classifier.getNbrG500Lq1800());
		recordScalar(string(classifier.getName()+": # G1800").c_str(),classifier.getNbrG1800());
	}

	classifier.getDurationStats().recordAs(string(classifier.getName()+": DurationStats").c_str());
}

void ProphetV2::recordAllClassifier()
{
	if (withGlobal){
		recordClassifier(Global);
	}
	if (withSucc){
		recordClassifier(Succ);
	}
	if (withFail){
		recordClassifier(Fail);
	}
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


void ProphetV2::initAllClassifier()
{

	withGlobal = par("withGlobalClassifier");
	withCDFForGlobal = par("CDFForGlobalClassifier");
	if (withGlobal){
		Global = ClassifiedContactStats("Global",false,withCDFForGlobal);
	}


	withSucc = par("withSuccClassifier");
	withCDFForSucc = par("CDFForSuccClassifier");
	if (withSucc){
	    Succ = ClassifiedContactStats("Successful",false,withCDFForSucc);
	}


	withFail = par("withFailClassifier");
	withCDFForFail = par("CDFForFailClassifier");
	if (withFail){
	    Fail = ClassifiedContactStats("Failed",false,withCDFForFail);
	}


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
		FailBndlResp = ClassifiedContactStats("withCDFForFailBndlResp",false,withCDFForFailBndlResp);
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

void ProphetV2::handleSelfMsg(cMessage* msg)
{

}

cObject *const ProphetV2::setDownControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr)
{
	return BaseNetwLayer::setDownControlInfo(pMsg, pDestAddr);
}

cObject *const ProphetV2::setUpControlInfo(cMessage *const pMsg, const LAddress::L3Type& pSrcAddr)
{
	return BaseNetwLayer::setUpControlInfo(pMsg, pSrcAddr);
}

void ProphetV2::deleteOldBundle(int ttl)
{
	std::list<BundleMeta> bundleToDelete = std::list<BundleMeta>();

	std::list<WaveShortMessage*>::iterator it;
	for (it = bundles.begin(); it != bundles.end(); it++){
		WaveShortMessage* tmp = *it;
		if ((tmp->getTimestamp() + ttl) > simTime().dbl()){
			BundleMeta meta = BundleMeta(tmp, Prophet_Enum::Bndl_Accepted);
		}
	}

	std::list<BundleMeta>::iterator it2;
	for (it2 = bundleToDelete.begin(); it2 != bundleToDelete.end(); it2++){
		nbrDeletedWithTTL++;
		existAndErase(*it2);
	}


//	= bundlesIndex.find(bndlMeta.getRecipientAddress());
//	if (it != bundlesIndex.end()){
//		innerIndexMap innerMap(it->second);
//		innerIndexIterator it2 = innerMap.find(bndlMeta.getSerial());
//		if (it2 !=innerMap.end()){
//			WaveShortMessage* wsm = it2->second;
//			innerMap.erase(bndlMeta.getSerial());
//			bundles.remove(wsm);
//			if (innerMap.empty()){
//				bundlesIndex.erase(bndlMeta.getRecipientAddress());
//			}else {
//				bundlesIndex[bndlMeta.getRecipientAddress()] = innerMap;
//			}
//			if (wsm->getOwner()==this){
//				delete wsm;
//			}
//			found = true;
//		}
//	}
//	return found;
}

ProphetV2::~ProphetV2()
{

}
