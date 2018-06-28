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

		maxSimulationTime = atoi(ev.getConfig()->getConfigValue("sim-time-limit"));
		recomputeMyNetwRoute = true;

		netwRouteExpirency = par("netwRouteExpirency").doubleValue();
		netwRoutePending = par("netwRoutePending").doubleValue();
		heartBeatMsgPeriod = par("heartBeatMsgPeriod").doubleValue();
		if ((netwRouteExpirency <= 0) || (netwRoutePending <= 0) || (heartBeatMsgPeriod <= 0)){
			opp_error("PyGraphServerManager::initialize - Neighborhood parameters should be a strictly positive doubles");
		}

		updateInterval = par("updateInterval").doubleValue();
		if (updateInterval <= 0){
			opp_error("PyGraphServerManager::initialize - Update Interval should be a strict positive double");
		}

		initBndlManagementOptions();

		initAckManagementOptions();

		initEquipedVehicle();

		withCtrlForSectorReAddr = false;
		oldSectorAddr = LAddress::L3NULL;
		newSectorAddr = LAddress::L3NULL;

		/*
		 * Initializing statistics & various metrics
		 */
		initContactStats();

		NBHTableNbrInsert    = 0;
		NBHTableNbrDelete    = 0;
		NBHAddressNbrInsert  = 0;
		NBHAddressNbrDelete  = 0;

		nbrL3Sent = 0;
		nbrL3Received = 0;

        bundlesReceived = 0;
		totalBundlesReceived = 0;
		bndlSentToVPA = 0;
		totalBndlSentToVPA = 0;

		firstSentToVPA = false;

		/*
		 * Initializing signals
		 */
		receiveL3SignalId = registerSignal("receivedL3Bndl");
		sentL3SignalId = registerSignal("sentL3Bndl");

		sentBitsLengthSignalId = registerSignal("sentBitsLength");

		helloCtrlBitsLengthId = registerSignal("helloCtrlBitsLength");
		otherCtrlBitsLengthId = registerSignal("otherCtrlBitsLength");
	}

	if (stage == 2){
		sectorId = getCurrentSector();
		heartBeatMsg = new cMessage("heartBeatMsg");
		scheduleAt(simTime(), heartBeatMsg);

		updateMsg = new cMessage("updateMsg");
		double currentTime = simTime().dbl();
		double scheduleTime = ceil(currentTime/updateInterval)*updateInterval;
		if (currentTime == scheduleTime){
			scheduleTime+=updateInterval;
		}
		scheduleAt(scheduleTime, updateMsg);
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

void DtnNetwLayer::handleUpperMsg(cMessage *msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	bndlModule.storeBundle(upper_msg);
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
	if (msg == heartBeatMsg){
		/***************** Cleaning Old Entries *****/
		if (withTTL){
			bndlModule.deleteExpiredBundles();
		}
		if (withAck && withTTLForAck){
			ackModule.deleteExpiredAcks();
		}
		/***************** Cleaning Old Entries *****/
		if(recomputeMyNetwRoute){
			myNetwRoute = NetwRoute(myNetwAddr,maxDbl,maxDbl, simTime(), true, nodeType, getCurrentPos());
		}
		updateNeighborhoodTable(myNetwAddr, myNetwRoute);
		sendingHelloMsg();
		scheduleAt(simTime()+heartBeatMsgPeriod, heartBeatMsg);
	}
	if (msg == updateMsg){
		unsigned long nbrStoredBundles = bndlModule.getNbrStoredBundles();
		if (nbrStoredBundles != 0){
			nbrStoredBundleVector.record(nbrStoredBundles);
		}
		unsigned long nbrStoredAcks = ackModule.getNbrStoredAcks();
		if (nbrStoredAcks != 0){
			nbrStoredAcksVector.record(nbrStoredAcks);
		}

		double currentTime = simTime().dbl();
		double scheduleTime = ceil(currentTime/updateInterval)*updateInterval;
		if (currentTime == scheduleTime){
			scheduleTime+=updateInterval;
		}
		scheduleAt(scheduleTime, updateMsg);
	}
}

void DtnNetwLayer::sendingHelloMsg()
{
	// Nothing for the moment
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

						NBHAddressNbrInsert++;

						free((void*) msgName);
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

					NBHAddressNbrDelete++;

					free((void*) msgName);
				}
				break;
		}

	}
	delete msg;
}

void DtnNetwLayer::handleUpperControl(cMessage *msg)
{
	ApplOppControlInfo* controlInfo = check_and_cast<ApplOppControlInfo* >(msg->getControlInfo());

	withCtrlForSectorReAddr = true;
	oldSectorAddr = controlInfo->getOldSectorNetwAddr();
	newSectorAddr = controlInfo->getNewSectorNetwAddr();

	if (withCtrlForSectorReAddr){
		bndlModule.updateRcvAddrForBundles(newSectorAddr);
	}
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

void DtnNetwLayer::finish()
{
	recordAllScalars();
	classifyAll();
	recordAllClassifier();
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
	if (withTTL){
		recordScalar("# DeletedBundlesWithTTL", bndlModule.getNbrDeletedBundlesByTtl());
	}
	recordScalar("# DeletedBundlesWithFIFO", bndlModule.getNbrDeletedBundlesByFifo());

	if (withAck){
		recordScalar("# DeletedBundlesWithAck", bndlModule.getNbrDeletedBundlesByAck());
		if (withTTLForAck){
			recordScalar("# DeletedAcksWithTTL", ackModule.getNbrDeletedAcksByTtl());
			recordScalar("# NbrMAJOfAcksExpTime", ackModule.getNbrUpdatesForAckExpireTime());
		}
		recordScalar("# DeletedAcksWithFIFO", ackModule.getNbrDeletedAcksByFifo());
	}

	recordScalar("# of Contacts", nbrContacts);
	recordScalar("# of InterContacts", nbrRecontacts);

	recordScalar("# insertOper Oracle", NBHAddressNbrInsert);
	recordScalar("# delOper Oracle", NBHAddressNbrDelete);
	recordScalar("# insertOper NBHTable", NBHTableNbrInsert);
	recordScalar("# delOper NBHTable", NBHTableNbrDelete);

	recordScalar("# L3Sent", nbrL3Sent);
	recordScalar("# L3Received", nbrL3Received);

	recordScalar("# Redundant Bundle at L3", (totalBundlesReceived- bundlesReceived));
	recordScalar("# Bundles at L3", bundlesReceived);
	recordScalar("# Bndl Sent to VPA (total)", totalBndlSentToVPA);
	recordScalar("# Bndl Sent to VPA (first)", bndlSentToVPA);
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
	}else{
		// neighbor exists, update the old entry
		neighborhoodTable[neighbor] = neighborEntry;
	}
}

void DtnNetwLayer::updateStoredBndlForSession(LAddress::L3Type srcAddr, std::set<unsigned long > bundlesToStore)
{
	NetwSession currentSession;
	std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
	if (it2 == neighborhoodSession.end()){
		currentSession = NetwSession(srcAddr,0);
	}else{
		currentSession = it2->second;
	}
	currentSession.updateStoredBundle(bundlesToStore);
	neighborhoodSession[srcAddr] = currentSession;
}

void DtnNetwLayer::updateStoredAcksForSession(LAddress::L3Type srcAddr, std::map<unsigned long, double > acksToStore)
{
	NetwSession currentSession;
	std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
	if (it2 == neighborhoodSession.end()){
		currentSession = NetwSession(srcAddr,0);
	}else{
		currentSession = it2->second;
	}
	currentSession.updateStoredAck(acksToStore);
	neighborhoodSession[srcAddr] = currentSession;
}

std::set<unsigned long > DtnNetwLayer::getUnStoredBndlForSession(LAddress::L3Type srcAddr, std::set<unsigned long > bundlesToFilter)
{
	std::set<unsigned long > filteredBundles;
	for (std::set<unsigned long >::iterator it = bundlesToFilter.begin(); it != bundlesToFilter.end(); it++){
		unsigned long serial = (*it);
		std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
		if (it2 == neighborhoodSession.end()){
			filteredBundles.insert(serial);
		}else{
			NetwSession srcSession = it2->second;
			if(!srcSession.existInStoredBundle(serial)){
				filteredBundles.insert(serial);
			}
		}
	}
	return filteredBundles;
}

std::map<unsigned long ,double> DtnNetwLayer::getUnStoredAcksForSession(LAddress::L3Type srcAddr, std::map<unsigned long ,double> acksToFilter)
{
	std::map<unsigned long ,double> filteredAcks;
	for (std::map<unsigned long ,double>::iterator it = acksToFilter.begin(); it != acksToFilter.end(); it++){
		unsigned long serial = it->first;
		double expTime = it->second;
		std::map<LAddress::L3Type, NetwSession>::iterator it2 = neighborhoodSession.find(srcAddr);
		if (it2 == neighborhoodSession.end()){
			filteredAcks.insert(std::pair<unsigned long ,double>(serial,expTime));
		}else{
			NetwSession srcSession = it2->second;
			if(!srcSession.existInStoredAck(serial)){
				filteredAcks.insert(std::pair<unsigned long ,double>(serial,expTime));
			}
		}
	}
	return filteredAcks;
}

std::vector<std::pair<WaveShortMessage*,int> > DtnNetwLayer::compAsFn_schedulingStrategy(std::vector<std::pair<WaveShortMessage*,int> > vectorToSort)
{
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
	// step 0: Deleting expired bundles
	if (withTTL){
		bndlModule.deleteExpiredBundles();
	}

	// step 1 : Reordering Bundles list
	std::vector<std::pair<WaveShortMessage*, int> >addressedToDestAddrWSMPair;
	std::vector<std::pair<WaveShortMessage*, int> >notAddressedToDestAddrWSMPair;
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = unsortedWSMPair.begin(); it != unsortedWSMPair.end(); it++){
		WaveShortMessage* wsm = it->first;
		if (wsm->getRecipientAddress() == destAddr){
			addressedToDestAddrWSMPair.push_back(std::pair<WaveShortMessage*, int>(it->first, it->second));
		}else{
			notAddressedToDestAddrWSMPair.push_back(std::pair<WaveShortMessage*, int>(it->first, it->second));
		}
	}

	std::vector<std::pair<WaveShortMessage*, int> >addressed_sortedWSMPair = compAsFn_schedulingStrategy(addressedToDestAddrWSMPair);
	std::vector<std::pair<WaveShortMessage*, int> >sortedWSMPair = addressed_sortedWSMPair;

	std::vector<std::pair<WaveShortMessage*, int> >notAddressed_sortedWSMPair = compAsFn_schedulingStrategy(notAddressedToDestAddrWSMPair);
	if (destType != VPA){
		sortedWSMPair.insert(sortedWSMPair.end(), notAddressed_sortedWSMPair.begin(), notAddressed_sortedWSMPair.end());
	}

	// step 2 : Filtering Bundles to send
	std::vector<WaveShortMessage* > sentWSM;
	std::vector<unsigned long > oldWSM;
	for (std::vector<std::pair<WaveShortMessage*, int> >::iterator it = sortedWSMPair.begin(); it != sortedWSMPair.end(); it++){
		WaveShortMessage* wsm = it->first;
		// step 2.1 : Check if the current bundle is not registered in neighborhoodSession
		if (ackModule.existAck(wsm->getSerial())) {continue;}
		std::map<LAddress::L3Type, NetwSession>::iterator itNode = neighborhoodSession.find(destAddr);
		if ((itNode != neighborhoodSession.end())){
			NetwSession sessionNode = itNode->second;
			if(sessionNode.existInStoredBundleOrAck(wsm->getSerial())){
				continue;
			}
		}
		sentWSM.push_back(wsm);
	}

	// step 3 : Update stats related Bundles sent to VPA if the encountered node is a VPA
	if (destType == VPA){
		if (!firstSentToVPA){
			bndlSentToVPA+=sentWSM.size();
			firstSentToVPA = true;
		}
		totalBndlSentToVPA+=sentWSM.size();
	}

	return sentWSM;
}

void DtnNetwLayer::emitSignalForHelloCtrlMsg(long  sizeHC_SB_Octets, long  sizeHC_SA_Octets, long  sizeHC_CL_Octets, long  sizeHC_RCC_Octets)
{
	string signalStr = lg2Str(sizeHC_SB_Octets * 8)+":"+lg2Str(sizeHC_SA_Octets * 8)+":"
						+lg2Str(sizeHC_CL_Octets * 8)+":"+lg2Str(sizeHC_RCC_Octets * 8);
	emit(helloCtrlBitsLengthId,signalStr.c_str());
}

void DtnNetwLayer::emitSignalForOtherCtrlMsg(long  sizeOC_SB_Octets, long  sizeOC_SA_Octets, long  sizeOC_CL_Octets, long  sizeOC_RCC_Octets)
{
	string signalStr = lg2Str(sizeOC_SB_Octets * 8)+":"+lg2Str(sizeOC_SA_Octets * 8)+":"
						+lg2Str(sizeOC_CL_Octets * 8)+":"+lg2Str(sizeOC_RCC_Octets * 8);
	emit(otherCtrlBitsLengthId,signalStr.c_str());
}

void DtnNetwLayer::gen1AckSerial(WaveShortMessage *msg)
{
	unsigned long serial = msg->getSerial();
	double currentTime = simTime().dbl();
	double expireTime = maxDbl;
	if (withTTLForAck){
		if (typeTTLForAck == Fixed_TTL){
			expireTime = currentTime + ttlForAck;
		}else if (typeTTLForAck == Adaptative_TTL){
			expireTime = msg->getTimestamp().dbl() + ttl;
		}
	}

	if (store1AckSerial(serial,expireTime)){
		if (withTTLForAck){
			emitSignalForAckLifeTime(serial, currentTime, expireTime);
		}
	}
}

bool DtnNetwLayer::store1AckSerial(unsigned long  serial, double expireTime)
{
	bool stored = ackModule.storeAck(serial,expireTime);

	if (stored){
		bndlModule.deleteBundleUponACK(serial);
	}

	return stored;
}

void DtnNetwLayer::storeNAckSerial(std::map<unsigned long ,double> ackSerialsWithTimestamp)
{
	for (std::map<unsigned long, double >::iterator it = ackSerialsWithTimestamp.begin(); it != ackSerialsWithTimestamp.end(); it++){
		unsigned long serial = it->first;
		double expireTime = it->second;
		if (store1AckSerial(serial,expireTime)){
			if (withTTLForAck){
				emitSignalForAckLifeTime(serial, -1, expireTime);
			}
		}
	}
}

void DtnNetwLayer::emitSignalForAckLifeTime(unsigned long serial, double startTime, double endTime)
{
	string signalStr = lg2Str((long)(serial))+":"+dbl2Str(startTime)+":"+dbl2Str(endTime);
	emit(t_ackLifeTime,signalStr.c_str());
}

void DtnNetwLayer::initBndlManagementOptions()
{
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

	bundlesStructureSize = par("storageSize");
	if (bundlesStructureSize<=0){
		opp_error("Size of the structure that store bundles can not be negative");
	}

	withTTL = par("withTTL").boolValue();
	if (withTTL){
		ttl = par("ttl").doubleValue();
	}else{
		ttl = double(maxSimulationTime);
	}

	bndlModule = BndlStorageHelper(bundlesStructureSize, withTTL, ttl);

	nbrStoredBundleVector.setName("StoredBundles");
	nbrStoredBundleVector.record(0);
}

void DtnNetwLayer::initAckManagementOptions()
{
	ackStructureSize = par("ackSize");
	if (ackStructureSize<=0){
		opp_error("Size of the structure that store acks can not be negative");
	}

	withAck = par("withAck");

	withTTLForAck = par("withTTLForAck").boolValue();
	if (withTTLForAck){
		ttlForAck = par("ttlForAck").doubleValue();
	}else{
		ttlForAck = double(maxSimulationTime);
	}

	ackModule = AckStorageHelper(ackStructureSize, withAck, withTTLForAck);

	int ttlType = par("typeTTLForAck");
	typeTTLForAck = (TTLForCtrlType) ttlType;

	nbrStoredAcksVector.setName("StoredAcks");
	nbrStoredAcksVector.record(0);

	t_ackLifeTime = registerSignal("ackLifeTime");
}

void DtnNetwLayer::initEquipedVehicle()
{
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
}

void DtnNetwLayer::initContactStats()
{
	recordContactStats = par("recordContactStats");

	nbrContacts =0;
	sumOfContactDur = 0.0;
	contactDurVector.setName("Evolution of contact duration mean");

	nbrRecontacts = 0;
	sumOfInterContactDur = 0.0;
	intercontactDurVector.setName("Evolution of intercontact duration mean");
}

long DtnNetwLayer::estimateInBitsCtrlSize(bool isHelloCtrl, std::set<unsigned long > *SB_Ctrl, std::map<unsigned long ,double> *SA_Ctrl, std::map<unsigned long ,double> *CL_Ctrl, std::set<unsigned long > *RCC_Ctrl)
{
	long sizeSB_Octets = 0, sizeSA_Octets = 0, sizeCL_Octets = 0, sizeRCC_Octets = 0;

	long totalSizeInBits = 0;

	if (SB_Ctrl != NULL){
		sizeSB_Octets = sizeof(unsigned long) * SB_Ctrl->size();
	}

	if (SA_Ctrl != NULL){
		if (withTTLForAck){
			sizeSA_Octets = (sizeof(unsigned long) + sizeof(double)) * SA_Ctrl->size();
		}else{
			sizeSA_Octets = sizeof(unsigned long) * SA_Ctrl->size();
		}
	}

	if (CL_Ctrl != NULL){
		sizeCL_Octets = sizeof(unsigned long) * CL_Ctrl->size();
	}

	if (RCC_Ctrl != NULL){
		sizeRCC_Octets = sizeof(unsigned long) * RCC_Ctrl->size();
	}

	if (isHelloCtrl){
		emitSignalForHelloCtrlMsg(sizeSB_Octets, sizeSA_Octets, sizeCL_Octets, sizeRCC_Octets);
	}else{
		emitSignalForOtherCtrlMsg(sizeSB_Octets, sizeSA_Octets, sizeCL_Octets, sizeRCC_Octets);
	}

	totalSizeInBits = (sizeSB_Octets + sizeSA_Octets + sizeCL_Octets + sizeRCC_Octets) * 8;

	return totalSizeInBits;
}
