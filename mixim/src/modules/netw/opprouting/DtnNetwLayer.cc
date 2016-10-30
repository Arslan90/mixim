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
//#include "istream"
#include "algorithm"
#include "fstream"
#include "TraCIMobility.h"
#include "ConstSpeedMobility.h"
#include "iostream"
#include <unistd.h>
#include <sys/types.h>
#include "simtime.h"


Define_Module(DtnNetwLayer);

void DtnNetwLayer::initialize(int stage)
{
	BaseNetwLayer::initialize(stage);
	if (stage==0){
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

    	withContactTrFl = par("withContactTraceFile").boolValue();
    	contactTrFlName = par("contactTraceFileName").stringValue();
	}

	if (stage == 2){
		if (withContactTrFl){
	        mobility = FindModule<BaseMobility*>::findSubModule(this->getParentModule());
	    	if (mobility == NULL){
	    		opp_error("Unable to find mobility subModule of currentParentModule");
	    	}else{
				Coord currentPos = mobility->getCurrentPosition();
				std::string id = "";
				ConstSpeedMobility* constMob = dynamic_cast<ConstSpeedMobility*>(mobility);
				if (constMob != NULL) {
			        if (constMob->getParentModule()->getIndex() == 0){
			        	contactTrFl.open(contactTrFlName.c_str(), ios::out);
			        	contactTrFl.close();
			        }
					std::ostringstream flux;
					flux << constMob->getParentModule()->getIndex();
					id = "r_"+flux.str();
				}else{
					TraCIMobility* traciMob = dynamic_cast<TraCIMobility*>(mobility);
					if (traciMob != NULL) {
						id = "v_"+traciMob->getExternalId();
					}
					if ((constMob == NULL) && (traciMob == NULL)){
						opp_error("Unable to correctly identify whether the node is a VPA or Veh");
					}
				}
				contactTrFl.open(contactTrFlName.c_str(), ios::out | ios::app);
				if (contactTrFl.is_open()){
					contactTrFl << myNetwAddr << ":" << id <<":[(" << currentPos.x << "," << currentPos.y << ")]:(s)" << simTime().dbl() << "\n"; // << std::endl;
					contactTrFl.close();
				}
	    	}

    		contactTrFlUpdatePeriod = par("contactTrFlUpdatePeriod").doubleValue();
    		contactTrFlMsg = new cMessage("TraceFile");
    		int currentTime = floor(simTime().dbl());
    		double nextSchedule = (currentTime / int(contactTrFlUpdatePeriod) + 1) * contactTrFlUpdatePeriod;
    		scheduleAt(nextSchedule, contactTrFlMsg);
		}
	}
}


std::vector<std::list<BundleMeta> > DtnNetwLayer::defineBundleOffer(NetwPkt *netwPkt)
{
	LAddress::L3Type encounterdNode = netwPkt->getSrcAddr();
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
	for (it = bundlesIndex.begin(); it != bundlesIndex.end(); it++){
		if (it->first == encounterdNode){
			continue;
		}else{
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



bool DtnNetwLayer::exist(WaveShortMessage *msg)
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



bool DtnNetwLayer::exist(BundleMeta bndlMeta)
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
	if (isEquiped){
		assert(dynamic_cast<WaveShortMessage*>(msg));
		WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
		storeBundle(upper_msg);
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
							unsigned long contactID = startRecordingContact(addr,time);
						}

						lastBundleProposal[addr] = time;
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
				int addr = *it;
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
	if (withContactTrFl){
		updateTraceFile(addr, time, "b");
	}

}



void DtnNetwLayer::recordEndContactStats(LAddress::L3Type addr, double time)
{
	double duration = time - contacts.find(addr)->second;
	sumOfContactDur+=duration;
	contactDurVector.record(sumOfContactDur/ double (nbrContacts));
	contacts.erase(addr);
	endContactTime[addr] = time;
	if (withContactTrFl){
		updateTraceFile(addr, time, "e");
	}
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

bool DtnNetwLayer::erase(BundleMeta bndlMeta)
{
	bool found = false;
	unsigned long serial = bndlMeta.getSerial();
	LAddress::L3Type addr = bndlMeta.getRecipientAddress();

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
}

void DtnNetwLayer::updateTraceFile(LAddress::L3Type addr, double time, char* type)
{
	Coord currentPos = mobility->getCurrentPosition();

	int length = 0;
	std::string lastLine, newLine;
	char c = '\0';
	int posNLChar = -1;

	stringstream ss1, ss2, ss3;
	ss1 << addr << ":" <<  myNetwAddr;
	ss2 << "(" << type << ")" << time;

	contactTrFl.open(contactTrFlName.c_str(), ios::app | ios::out | ios::in);
	if (contactTrFl.is_open()){
		contactTrFl.seekg(0, contactTrFl.end); // go to the end of file
        length = contactTrFl.tellg();//Get file size
//        // loop backward over the file
        if (length > 0 ){
        	int i = 0;
            for(i = length-2; i >= 0; i-- )
            {
            	contactTrFl.seekg(i, contactTrFl.beg);
                c = contactTrFl.get();
                if(c == '\n' ){
                	//new line?
                	posNLChar = i;
                	break;
                }
            }
            if (i == -1){
            	contactTrFl.seekg(0, contactTrFl.beg);
            }
            std::getline(contactTrFl, lastLine);//read last line
        }
    	int pos = lastLine.find(ss2.str());
    	if ((lastLine.find(ss1.str()) != std::string::npos) && (lastLine.find(ss2.str()) != std::string::npos)){
    		// we can merge both lines
    		newLine = lastLine;
    		ss3 << "[" << currentPos.x << "," << currentPos.y << "]:";
    		newLine.insert(pos,ss3.str());
    		if (posNLChar == -1 ){
    			truncate(contactTrFlName.c_str(), 0);
    		} else if (posNLChar > 0){
    			truncate(contactTrFlName.c_str(), posNLChar+1);
    		}

    	}else {
    		ss3 << myNetwAddr << ":" <<  addr << ":["  << currentPos.x << "," << currentPos.y << "]:(" << type << ")" << time; // << std::endl;
    		newLine = ss3.str();
    	}

    	contactTrFl << newLine << "\n"; // << std::endl;
		contactTrFl.close();
	}
}

void DtnNetwLayer::updateTraceFile(std::list<LAddress::L3Type> listAddr, double time, char* type)
{
	Coord currentPos = mobility->getCurrentPosition();

	stringstream ss1, ss2;
	for (std::list<LAddress::L3Type>::iterator it = listAddr.begin(); it != listAddr.end();it++){
		int i = *it;
		ss1 << i << ":";
	}
	ss2 << myNetwAddr << ":" <<  ss1.str() << "["  << currentPos.x << "," << currentPos.y << "]:(" << type << ")" << time; // << std::endl;

	contactTrFl.open(contactTrFlName.c_str(), ios::app | ios::out | ios::in);
	if (contactTrFl.is_open()){
    	std::string newLine = ss2.str();
    	contactTrFl << newLine << "\n"; // << std::endl;
		contactTrFl.close();
	}
}

void DtnNetwLayer::periodicUpdateTraceFile()
{
	if (withContactTrFl){
		std::map<LAddress::L3Type, double>::iterator it;
		std::list<LAddress::L3Type> addrList;
		if (!contacts.empty()){
			for (it = contacts.begin(); it != contacts.end(); it++){
				addrList.push_back(it->first);
			}
			double time = simTime().dbl();
			updateTraceFile(addrList,time,"u");
		}
	}
}






