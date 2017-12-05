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

#include "DtnNetwLayer.h"
#include "multiFunctions.h"
#include "ApplOppControlInfo.h"
#include "FindModule.h"
#include "algorithm"
#include "VEHICLEpOpp.h"
#include "VPApOpp.h"


Define_Module(DtnNetwLayer);

void DtnNetwLayer::initialize(int stage)
{
	BaseNetwLayer::initialize(stage);
	if (stage==0){

		DefineNodeType();
		netwRouteExpirency = par("netwRouteExpirency").doubleValue();
		netwRoutePending = par("netwRoutePending").doubleValue();
		heartBeatMsgPeriod = par("heartBeatMsgPeriod").doubleValue();

		NBHTableNbrInsert    = 0;
		NBHTableNbrDelete    = 0;
		NBHAddressNbrInsert  = 0;
		NBHAddressNbrDelete  = 0;
		/*
		 * L3Address will be initialized by BaseNetwLayer::initialize(1);
		 */
		/*
		 * Initialization of the used structures
		 */
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
			opp_error("Percentage of equipped vehicle is wrong, please correct it");
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

		/*
		 * Collecting data & metrics
		 */

		recordContactStats = par("recordContactStats");

		nbrL3Sent = 0;
		nbrL3Received = 0;

		nbrContacts =0;
		sumOfContactDur = 0.0;
		contacts = std::map<LAddress::L3Type,double>();
		contactDurVector.setName("Evolution of contact duration mean");

		nbrRecontacts = 0;
		sumOfInterContactDur = 0.0;
		intercontactDurVector.setName("Evolution of intercontact duration mean");

		deletedBundlesWithAck = 0;

		delayed = par("delayed").doubleValue();

		delayedFrag = par("delayedFrag").doubleValue();

        demandedAckedBundle = 0;

        bundlesReceived = 0;

        lastBundleUpdate = 0.0;

        nbrRestartedIEP = 0;

        nbrCancelRestartedIEP = 0;

        withRestart = par("withRestart").boolValue();

        withConnectionRestart = par("withConnectionRestart").boolValue();

		receiveL3SignalId = registerSignal("receivedL3Bndl");
		sentBitsLengthSignalId = registerSignal("sentBitsLength");

		nbrNeighors = 0;
		nbrCountForMeanNeighbors = 0;

		hadBundles = false;

		receivedHWICVPA = 0;
		receivedBWICVPA = 0;
		receivedAWICVPA = 0;

		int tmpScheduleStrategy = par("scheduleStrategy");

		switch (tmpScheduleStrategy) {
			case 0:
				scheduleStrategy = RCAscRLDesc;
				break;
			case 1:
				scheduleStrategy = RCAscRLAsc;
				break;
			case 2:
				scheduleStrategy = RCDescRLDesc;
				break;
			case 3:
				scheduleStrategy = RCDescRLAsc;
				break;
			case 4:
				scheduleStrategy = RLAsc;
				break;
			case 5:
				scheduleStrategy = RLDesc;
				break;
			default:
				opp_error("Unrecognized scheduling strategy");
				break;
		}

		totalBundlesReceived = 0;
		bndlSentToVPA = 0;
		totalBndlSentToVPA = 0;

		firstSentToVPA = false;

		meetVPA = false;
	}

	if (stage == 2){
		sectorId = getCurrentSector();
		heartBeatMsg = new cMessage("heartBeatMsg");
		scheduleAt(simTime(), heartBeatMsg);
	}
}


bool DtnNetwLayer::ackExist(WaveShortMessage *msg)
{
	bool exist = false;
	if ((withAck)&&(acksIndex.find(msg->getSerial())!=acksIndex.end())){
		exist = true;
	}
	return exist;
}



bool DtnNetwLayer::ackExist(BundleMeta bndlMeta)
{
	bool exist = false;
	if ((withAck)&&(acksIndex.find(bndlMeta.getSerial())!=acksIndex.end())){
		exist = true;
	}
	return exist;
}



bool DtnNetwLayer::existAndErase(BundleMeta bndlMeta)
{
	bool found = false;
	unsigned long serial = bndlMeta.getSerial();
	LAddress::L3Type addr = bndlMeta.getRecipientAddress();

	if (exist(bndlMeta)){
		bundlesIndexIterator it = bundlesIndex.find(addr);
		if (it != bundlesIndex.end()){
			innerIndexMap inner_map = it->second;
			innerIndexIterator it2 = inner_map.find(serial);
			if (it2 !=inner_map.end()){
				WaveShortMessage* wsm = it2->second;
				bundles.remove(wsm);
				inner_map.erase(serial);
				if (inner_map.empty()){
					bundlesIndex.erase(addr);
				}else {
					bundlesIndex[addr] = inner_map;
				}
				if (wsm->getOwner()==this){
					delete wsm;
				}
				found = true;
			}
		}else{
			opp_error("bundle exist but not found in the index");
		}
	}
	return found;
}

void DtnNetwLayer::storeBundle(WaveShortMessage *msg)
{
	// step 1 : check if the bundle is already stored
	if (!exist(msg)){

		// step 2 : add the bundle to stored bundles
		bundlesIndexIterator it;
		if (bundles.size()==bundlesStructureSize){
			erase(bundles.front());
		}else if (bundles.size()>bundlesStructureSize){
			opp_error("bundles storage structure exceed its maximum size");
		}
		bundles.push_back(msg);

		// step 3 : adding this bundle to index
		it = bundlesIndex.find(msg->getRecipientAddress());
		innerIndexMap inner_map;
		if (it != bundlesIndex.end()){
			inner_map = it->second;
		}
		inner_map.insert(std::pair<unsigned long,WaveShortMessage*>(msg->getSerial(),msg));
		bundlesIndex[msg->getRecipientAddress()] = inner_map;

		haveToRestartIEP(simTime());

	  	std::list<WaveShortMessage*> bundlesCopy = std::list<WaveShortMessage*>(bundles.begin(),bundles.end());
	  	bundlesCopy.erase(std::unique( bundlesCopy.begin(), bundlesCopy.end() ), bundlesCopy.end());

	  	if (bundles.size() != bundlesCopy.size()){
	  		opp_error("Double insertion in bundles Index");
	  	}
	}

	if (bundles.size() > 0){
		hadBundles = true;
	}
}



void DtnNetwLayer::storeACK(BundleMeta meta)
{
	// step 1 : check if the ack is already stored
	if (!ackExist(meta)){
		if (acks.size()==ackStructureSize){
			unsigned long serial = acks.front().getSerial();
			acksIndex.erase(serial);
			acks.pop_front();
		}else if (acks.size() > ackStructureSize){
			opp_error("acks storage structure exceed its maximum size");
		}
		// step 2 : add the ack to stored ack
		acks.push_back(meta);

		// step 3 : adding this ack to index
		acksIndex[meta.getSerial()] = meta;

		// step 4 : increment counter related to bundles deleted with the use of ACK
		if (existAndErase(meta)) {
			deletedBundlesWithAck++;
		}
	}
}



LAddress::L3Type DtnNetwLayer::getAddressFromName(const char *name)
{
	int addr = 0;
	if (strcmp(name,"")!=0){
		std::istringstream iss(name);
		iss >> addr;
	}else {
		opp_error("getting address from name return NULL (DtnNetwLayer::getAddressFromName)");
	}
	return addr;
}



double DtnNetwLayer::getTimeFromName(const char *name)
{
	double time = 0;
	if (strcmp(name,"")!=0){
		std::istringstream iss(name);
		iss >> time;
	}else {
		opp_error("getting time from name return NULL (DtnNetwLayer::getTimeFromName)");
	}
	return time;
}



void DtnNetwLayer::deleteOldBundle(int ttl)
{
	std::list<WaveShortMessage*> bundlesToDelete;

	if (withTTL){
		std::list<WaveShortMessage*>::iterator it;
		for (it = bundles.begin(); it != bundles.end(); it++){
			WaveShortMessage* wsm = *it;
			if ((wsm->getTimestamp() + ttl) > simTime().dbl()){
				bundlesToDelete.push_front(wsm);
			}
		}

		for (it = bundlesToDelete.begin(); it != bundlesToDelete.end(); it++){
			WaveShortMessage* wsm = *it;
			if(erase(wsm)){
				nbrDeletedWithTTL++;
			}
		}
	}
}



void DtnNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
	std::map<unsigned long, int>::iterator it = bundlesReplicaIndex.find(upper_msg->getSerial());
	if (it == bundlesReplicaIndex.end()){
		bundlesReplicaIndex.insert(std::pair<unsigned long, int>(upper_msg->getSerial(), 0));
	}
}



void DtnNetwLayer::handleLowerMsg(cMessage *msg)
{
	Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);

	NetwPkt *netwPkt = check_and_cast<NetwPkt *>(m->decapsulate());

	if (isEquiped){
		if ((netwPkt->getDestAddr()==LAddress::L3BROADCAST)||(netwPkt->getDestAddr()==myNetwAddr)){
			updatingL3Received();
		}
	}
	delete netwPkt;
	delete msg;
}

void DtnNetwLayer::handleSelfMsg(cMessage *msg)
{
	switch (msg->getKind()) {
		case RESTART:
			break;
		default:
			break;
	}
}

void DtnNetwLayer::handleLowerControl(cMessage *msg)
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
							startRecordingContact(addr,time);
						}

						lastBundleProposal[addr] = time;

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

void DtnNetwLayer::handleUpperControl(cMessage *msg)
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

	unsigned int bundlesIndexSize = 0;
	for (it1 = bundlesIndex.begin(); it1 != bundlesIndex.end() ; it1++){
		bundlesIndexSize+= it1->second.size();
	}
	if (bundlesIndexSize != bundles.size()){
		opp_warning("Size of Bundles and BundlesIndex data structurs are not the same");
	}
}

bool DtnNetwLayer::haveToRestartIEP(simtime_t t)
{
	bool haveToRestartIEP = false;

	lastBundleUpdate = t.dbl();
	if (withRestart){
		for (std::map<LAddress::L3Type, double>::iterator it = lastBundleProposal.begin(); it != lastBundleProposal.end(); it++){
			double lastProposal = it->second;
			if (lastProposal < lastBundleUpdate){
				stringstream ss;
				ss << it->first;
//				if (restartIEP->isScheduled()){
//					cancelEvent(restartIEP);
//					nbrCancelRestartedIEP++;
//				}
				restartIEP = new cMessage(ss.str().c_str(), RESTART);
				scheduleAt(simTime(),restartIEP);

				haveToRestartIEP = true;
				nbrRestartedIEP++;
			}
		}
		if (withConnectionRestart){
			for (std::set<LAddress::L3Type>::iterator it = neighborsAddress.begin(); it != neighborsAddress.end(); it++){
				std::map<LAddress::L3Type, double>::iterator it2 = lastBundleProposal.find(*it);
				if (it2 == lastBundleProposal.end()){
					stringstream ss;
					ss << *it;
					restartIEP = new cMessage(ss.str().c_str(), RESTART);
					scheduleAt(simTime(),restartIEP);

					haveToRestartIEP = true;
					nbrRestartedIEP++;
				}
			}
		}
	}

	return haveToRestartIEP;
}

bool DtnNetwLayer::forceRestartIEP(LAddress::L3Type addr)
{
	bool haveToRestartIEP = false;
	stringstream ss;
	ss << addr;
	//				if (restartIEP->isScheduled()){
	//					cancelEvent(restartIEP);
	//					nbrCancelRestartedIEP++;
	//				}
	restartIEP = new cMessage(ss.str().c_str(), FORCED_RESTART);
	scheduleAt(simTime(),restartIEP);

	haveToRestartIEP = true;
	nbrRestartedIEP++;

	return haveToRestartIEP;
}

unsigned long DtnNetwLayer::generateContactSerial(int myAddr, int seqNumber, int otherAddr)
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

unsigned long DtnNetwLayer::startRecordingContact(int addr, double time)
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

unsigned long DtnNetwLayer::startRecordingContact(int addr, unsigned long  contactID)
{
	/*
	 * possibility that we received a msg before receiving the control msg
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



unsigned long DtnNetwLayer::endRecordingContact(int addr, double time)
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



unsigned long DtnNetwLayer::endRecordingContact(unsigned long  contactID, bool hasForcedEnding)
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
		contact.setHasForcedEnding(hasForcedEnding);
		indexContactStats[contactID] = contact;
	}

	return contactID;
}

void DtnNetwLayer::recordBeginContactStats(LAddress::L3Type addr, double time)
{
	// updating nbr contacts
	nbrContacts++;
	// saving the starting time of the contact
	contacts.insert(std::pair<LAddress::L3Type, double>(addr, time));
}



void DtnNetwLayer::recordEndContactStats(LAddress::L3Type addr, double time)
{
	double duration = time - contacts.find(addr)->second;
	sumOfContactDur+=duration;
	contactDurVector.record(sumOfContactDur/ double (nbrContacts));
	contacts.erase(addr);
	endContactTime[addr] = time;
}



void DtnNetwLayer::recordRecontactStats(LAddress::L3Type addr, double time)
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
	}
}



void DtnNetwLayer::classify(SimpleContactStats *newContact)
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
	}else if ((withFail)&&(!newContact->isSuccessfulContact())){
		Fail.update(newContact);
	}
}



void DtnNetwLayer::classifyAll()
{
	for(iteratorContactStats it = indexContactStats.begin(); it != indexContactStats.end(); it++){
		SimpleContactStats* contactToClassify = &(it->second);
		classify(contactToClassify);
	}
}



void DtnNetwLayer::initAllClassifier()
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
}



void DtnNetwLayer::recordClassifier(ClassifiedContactStats classifier)
{
	recordScalar(string(classifier.getName()+": # nbrContact").c_str(),classifier.getNbrContacts());
	recordScalar(string(classifier.getName()+": # nbrToDiscard").c_str(),classifier.getNbrToDiscard());
	recordScalar(string(classifier.getName()+": # nbrAlreadyAcked").c_str(),classifier.getNbrAlreadyAcked());

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

	recordScalar(string(classifier.getName()+": # OffersSent").c_str(),classifier.getOfferSent());
	recordScalar(string(classifier.getName()+": # OffersReceived").c_str(),classifier.getOfferReceived());

	recordScalar(string(classifier.getName()+": # AcceptsSent").c_str(),classifier.getAcceptSent());
	recordScalar(string(classifier.getName()+": # AcceptsReceived").c_str(),classifier.getAcceptReceived());

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



bool DtnNetwLayer::erase(WaveShortMessage *msg)
{
	bool found = false;
	unsigned long serial = msg->getSerial();
	LAddress::L3Type addr = msg->getRecipientAddress();

	bundlesIndexIterator it = bundlesIndex.find(addr);
	if (it != bundlesIndex.end()){
		innerIndexMap inner_map = it->second;
		innerIndexIterator it2 = inner_map.find(serial);
		if (it2 !=inner_map.end()){
			WaveShortMessage* wsm = it2->second;
			bundles.remove(wsm);
			inner_map.erase(serial);
			if (inner_map.empty()){
				bundlesIndex.erase(addr);
			}else {
				bundlesIndex[addr] = inner_map;
			}
			if (wsm->getOwner()==this){
				delete wsm;
			}
			found = true;
		}else {
			opp_warning("Unable to locate the bundle, must do a global research to found it");
			bundlesIndexIterator it1;
			innerIndexIterator it2;

			for (it1 = bundlesIndex.begin(); it1 != bundlesIndex.end() ; it1++){
				innerIndexMap inner_map = it1->second;
				innerIndexIterator it2 = inner_map.find(serial);
				if (it2 !=inner_map.end()){
					WaveShortMessage* wsm = it2->second;
					bundles.remove(wsm);
					inner_map.erase(serial);
					if (inner_map.empty()){
						bundlesIndex.erase(addr);
					}else {
						bundlesIndex[addr] = inner_map;
					}
					if (wsm->getOwner()==this){
						delete wsm;
					}
					found = true;
				}
			}

			if (!found){
				opp_error("bundle doesn't exist in the index");
			}
		}
	}else{
		opp_error("bundle exist but not found in the index");
	}
	return found;
}

bool DtnNetwLayer::erase(unsigned long serial)
{
	bool found = false;

	WaveShortMessage* wsm = getStoredWSMFromSerial(serial);

	if (wsm !=NULL){
		found = erase(wsm);
		if (found){
			bundlesReplicaIndex.erase(serial);
		}
	}

	return found;
}

void DtnNetwLayer::finish()
{
	recordAllScalars();
	classifyAll();
	recordAllClassifier();

	while (!bundles.empty()){
		delete bundles.front();
		bundles.pop_front();
	}
}

void DtnNetwLayer::recordAllClassifier()
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
}



void DtnNetwLayer::recordAllScalars()
{
	recordScalar("# L3Sent", nbrL3Sent);
	recordScalar("# L3Received", nbrL3Received);

	recordScalar("# of Contacts", nbrContacts);
	recordScalar("# of InterContacts", nbrRecontacts);

	recordScalar("# Bundles at L3", bundlesReceived);

	if (withAck){
		recordScalar("# ACKs", acks.size());
		recordScalar("# DeletedBundles", deletedBundlesWithAck);
		recordScalar("# DemandedAckedBundles", demandedAckedBundle);
		recordScalar("# Bundles", bundles.size());
	}

	if (withTTL){
		recordScalar("# DeletedBundlesWithTTL", nbrDeletedWithTTL);
	}

	if (withRestart){
		recordScalar("# Restarted IEP", nbrRestartedIEP);
	}

	recordScalar("# insertOper Oracle", NBHAddressNbrInsert);
	recordScalar("# delOper Oracle", NBHAddressNbrDelete);
	recordScalar("# insertOper NBHTable", NBHTableNbrInsert);
	recordScalar("# delOper NBHTable", NBHTableNbrDelete);

	recordScalar("# Redundant Bundle at L3", (totalBundlesReceived- bundlesReceived));

	recordScalar("# Bndl Sent to VPA (total)", totalBndlSentToVPA);
	recordScalar("# Bndl Sent to VPA (first)", bndlSentToVPA);
}

void DtnNetwLayer::resetStatPerVPA()
{
	bundleSentPerVPA.clear();
	ackReceivedPerVPA.clear();
	meetVPA = false;
	nbrNeighors = 0;
	nbrCountForMeanNeighbors = 0;
	hadBundles = false;
	if (bundles.size() > 0){
		hadBundles = true;
	}
	vpaContactDuration.clear();
	vpaContactDistance.clear();

	receivedHWICVPA = 0;
	receivedBWICVPA = 0;
	receivedAWICVPA = 0;
}

bool DtnNetwLayer::isMeetVpa() const
{
    return meetVPA;
}

std::string DtnNetwLayer::BundleSentPerVpaSerialToString() const
{
	std::string serialsAsStr ="";
	for(std::set<unsigned long>::const_iterator it = bundleSentPerVPA.begin(); it != bundleSentPerVPA.end(); it++){
		std::stringstream ss;
		ss << *it;
		serialsAsStr = serialsAsStr+ss.str()+",";
	}
//	if (serialsAsStr.size() > 1){
//		serialsAsStr.erase(serialsAsStr.size()-1);
//	}
	if (serialsAsStr.size() == 0){
		serialsAsStr = serialsAsStr+",";
	}
	return serialsAsStr;
}

long DtnNetwLayer::getNbrCountForMeanNeighbors() const
{
    return nbrCountForMeanNeighbors;
}

long DtnNetwLayer::getNbrNeighors() const
{
    return nbrNeighors;
}

bool DtnNetwLayer::isHadBundles() const
{
    return hadBundles;
}

std::pair<double,double> DtnNetwLayer::VPAContactDuration()
{
	double total = 0.0;
	double count = 0.0;

	for (std::list<double>::iterator it = vpaContactDuration.begin(); it != vpaContactDuration.end(); it++){
		double tmp = *it;
		if (tmp >= 0){
			total +=tmp;
			count++;
		}
	}

	return std::pair<double,double>(total,count);
}

std::pair<double,double> DtnNetwLayer::VPAContactDistance()
{
	double total = 0.0;
	double count = 0.0;

	for (std::list<double>::iterator it = vpaContactDistance.begin(); it != vpaContactDistance.end(); it++){
		double tmp = *it;
		if (tmp >= 0){
			total +=tmp;
			count++;
		}
	}

	return std::pair<double,double>(total,count);
}

void DtnNetwLayer::DefineNodeType()
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
			opp_error("DtnNetwLayer::DefineNodeType() - Unable to define NodeType please check existence of appl module in NED file");
		}
	}
}

int DtnNetwLayer::getCurrentSector()
{
	int oldSector = sectorId;
	switch (nodeType) {
		case Veh:
			traci = TraCIMobilityAccess().get(getParentModule());
			sectorId = traci->getCurrentSector();
			break;
		case VPA:
			traci = NULL;
			sectorId = this->getParentModule()->getIndex();
			break;
		default:
			opp_error("DtnNetwLayer::getCurrentSector() - Unable to define CurrentSector due to unknown node type");
			break;
	}
	if (oldSector != sectorId){
		firstSentToVPA = false;
	}
	return sectorId;
}

Coord DtnNetwLayer::getCurrentPos()
{
		BaseMobility* mobilityMod = FindModule<BaseMobility*>::findSubModule(getParentModule());
		if (mobilityMod == NULL){
			opp_error("No mobility module found");
		}
		return mobilityMod->getCurrentPosition();
}

void DtnNetwLayer::updateNeighborhoodTable(LAddress::L3Type neighbor, NetwRoute neighborEntry)
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
		if (neighborEntry.getNodeType() == VPA){
			meetVPA = true;
		}
	}else{
		// neighbor exists, update the old entry
		neighborhoodTable[neighbor] = neighborEntry;
	}
}

void DtnNetwLayer::updateStoredBndlForSession(LAddress::L3Type srcAddr, std::set<unsigned long > storedBundle)
{
	std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
	if (it2 == neighborhoodSession.end()){
		NetwSession newSession = NetwSession(srcAddr,0);
		for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
			newSession.insertInDelivredToBndl(*it);
		}
		neighborhoodSession.insert(std::pair<LAddress::L3Type, NetwSession>(srcAddr, newSession));
	}else{
		NetwSession newSession = it2->second;
		for (std::set<unsigned long >::iterator it = storedBundle.begin(); it != storedBundle.end(); it++){
			newSession.insertInDelivredToBndl(*it);
		}
		neighborhoodSession[srcAddr] = newSession;
	}
}

void DtnNetwLayer::updateStoredAcksForSession(LAddress::L3Type srcAddr, std::set<unsigned long > storedAcks)
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

bool DtnNetwLayer::exist(WaveShortMessage *msg)
{
	return exist(msg->getSerial());
}

bool DtnNetwLayer::exist(BundleMeta bndlMeta)
{
	return exist(bndlMeta.getSerial());
}

bool DtnNetwLayer::exist(unsigned long  serial)
{
	bool found = false;

	for (bundlesIndexIterator it = bundlesIndex.begin(); it != bundlesIndex.end(); it++){
		innerIndexMap innerMap = it->second;
		innerIndexIterator it2 = innerMap.find(serial);
		if (it2 !=innerMap.end()){
				found = true;
				break;
		}
	}
	return found;
}

std::vector<std::pair<WaveShortMessage*,int> > DtnNetwLayer::compAsFn_schedulingStrategy(std::vector<std::pair<WaveShortMessage*,int> > vectorToSort)
{
//	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = vectorToSort.begin(); it != vectorToSort.end(); it++){
//		cout << it->first->getSerial() << std::endl;
//	}
	switch (scheduleStrategy) {
		case RCAscRLDesc:
			std::sort(vectorToSort.begin(), vectorToSort.end(), func_RCAscRLDesc);
			break;
		case RCAscRLAsc:
			std::sort(vectorToSort.begin(), vectorToSort.end(), func_RCAscRLAsc);
			break;
		case RCDescRLDesc:
			std::sort(vectorToSort.begin(), vectorToSort.end(), func_RCDescRLDesc);
			break;
		case RCDescRLAsc:
			std::sort(vectorToSort.begin(), vectorToSort.end(), func_RCDescRLAsc);
			break;
		case RLAsc:
			std::sort(vectorToSort.begin(), vectorToSort.end(), func_RLAsc);
			break;
		case RLDesc:
			std::sort(vectorToSort.begin(), vectorToSort.end(), func_RLDesc);
			break;
		default:
			opp_error("Unrecognized scheduling strategy");
			break;
	}
//	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = vectorToSort.begin(); it != vectorToSort.end(); it++){
//		cout << it->first->getSerial() << std::endl;
//	}
	return vectorToSort;
}

void DtnNetwLayer::sendDown(cMessage *msg)
{
	BaseLayer::sendDown(msg);
	updatingL3Sent();
}

void DtnNetwLayer::sendDown(cMessage *msg, long HelloCtrlLength, long OtherCtrlLength, short nbrEncapData)
{
	string signalStr = lg2Str(HelloCtrlLength)+":"+lg2Str(OtherCtrlLength)+":"+lg2Str(nbrEncapData);
	emit(sentBitsLengthSignalId,signalStr.c_str());
	sendDown(msg);
}

void DtnNetwLayer::storeAckSerial(unsigned long  serial)
{
	if (ackSerial.count(serial) == 0){
		ackSerial.insert(serial);
	}
}

void DtnNetwLayer::storeAckSerials(std::set<unsigned long > setOfSerials)
{
    for (std::set<unsigned long>::iterator it = setOfSerials.begin(); it != setOfSerials.end(); it++){
    	storeAckSerial(*it);
    	if (erase(*it)){
    		deletedBundlesWithAck++;
    	}
    }
}

WaveShortMessage* DtnNetwLayer::getStoredWSMFromSerial(unsigned long serial){
	WaveShortMessage* wsm = NULL;
	for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); it++){
		if (serial == (*it)->getSerial()){
			wsm = (*it);
			break;
		}
	}
	return wsm;
}

void DtnNetwLayer::prepareNetwPkt(DtnNetwPkt* myNetwPkt, short  kind, LAddress::L3Type destAddr)
{
	int realPktLength = 0;
	myNetwPkt->setKind(kind);
	myNetwPkt->setSrcAddr(myNetwAddr);
	myNetwPkt->setSrcType(nodeType);
	myNetwPkt->setDestAddr(destAddr);
	sectorId = getCurrentSector();
	myNetwPkt->setVpaSectorId(sectorId);
	myNetwPkt->setCurrentPos(getCurrentPos());

	realPktLength = sizeof(kind)+sizeof(myNetwAddr)+sizeof(destAddr)+sizeof(unsigned long) * 2 + sizeof(int);
	realPktLength *= 8;

	myNetwPkt->setBitLength(realPktLength);
}

std::vector<WaveShortMessage* > DtnNetwLayer::scheduleFilterBundles(std::vector<std::pair<WaveShortMessage*,int> > unsortedWSMPair, LAddress::L3Type destAddr, int destType){

	// step 1 : Reordering Bundles list
	std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair = compAsFn_schedulingStrategy(unsortedWSMPair);


	// step 2 : Filtering Bundles to send
	std::vector<WaveShortMessage* > sentWSM;
	std::vector<unsigned long > oldWSM;
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = sortedWSMPair.begin(); it != sortedWSMPair.end(); it++){
		WaveShortMessage* wsm = it->first;
		// step 2.1 : Check if the current bundle is not registered in neighborhoodSession
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

		// step 2.2 : Check if the current bundle is up to date and has not expired
		if (withTTL){
			double duration = (simTime()-wsm->getTimestamp()).dbl();
			if (duration > ttl){
				oldWSM.push_back(wsm->getSerial());
				continue;
			}
		}

		// step 2.3 : If bundle is fine, then add it to list of Bundles to sends
		sentWSM.push_back(wsm);
	}

	// step 3 : Delete Expired Bundles
	for (std::vector<unsigned long >::iterator it = oldWSM.begin(); it != oldWSM.end(); it++){
		if (erase(*it)){
			nbrDeletedWithTTL++;
		}
	}

	// step 4 : Update stats related Bundles sent to VPA if the encountered node is a VPA
	if (destType == VPA){
		if (!firstSentToVPA){
			bndlSentToVPA+=sentWSM.size();
			firstSentToVPA = true;
		}
		totalBndlSentToVPA+=sentWSM.size();
	}

	return sentWSM;
}
