/*
 * BndlStorageHelper.cc
 *
 *  Created on: Feb 7, 2018
 *      Author: arslan
 */

#include "BndlStorageHelper.h"

BndlStorageHelper::BndlStorageHelper()
{
	this->bundleSerials = std::list<unsigned long>();
	this->bundlesIndexedBySerial = std::map<unsigned long,WaveShortMessage*>();
	this->serialsIndexedByRcvAddr = std::multimap<LAddress::L3Type, unsigned long>();
	this->indexForReplica = std::map<unsigned long, std::pair<int,int> >();

	this->withLimitedReplica = false;
	this->maxNbrReplica = NS_BndlStorageHelper::maxInt;

	nbrDeletedBundlesByTTL = 0;
	nbrDeletedBundlesByAck = 0;
	nbrDeletedBundlesByFIFO = 0;
	nbrDeletedBundlesByCustody = 0;
	nbrDeletedBundlesByNoRmgReplica = 0;
}

BndlStorageHelper::BndlStorageHelper(unsigned int sizeOfBundleStorage, bool withTTL, double ttl)
{
	BndlStorageHelper();
	this->storageSize = sizeOfBundleStorage;

	this->withTTL = withTTL;
	this->ttl = ttl;
}

BndlStorageHelper::~BndlStorageHelper() {
	// TODO Auto-generated destructor stub
}

void BndlStorageHelper::reInitWithLimitedReplica(int maxNbrReplica)
{
	this->withLimitedReplica = true;
	this->maxNbrReplica = maxNbrReplica;
}

bool BndlStorageHelper::storeBundle(WaveShortMessage *msg, int rmgReplica)
{
	bool stored = false;
	unsigned long serial = msg->getSerial();
	LAddress::L3Type addr = msg->getRecipientAddress();
	double expireTime = msg->getTimestamp().dbl() + ttl;
	// step 1 : check if the bundle is already stored
	if ((!existBundle(serial)) && (!hasExpired(expireTime))){
		// step 2 : check if there is enough place to store bundle
		deleteFirstBundle();

		// step 3 : add the bundle to stored bundles and to all relative indexes
		bundleSerials.push_back(serial);
		bundlesIndexedBySerial.insert(std::pair<unsigned long, WaveShortMessage*>(serial,msg));
		serialsIndexedByRcvAddr.insert(std::pair<LAddress::L3Type, unsigned long>(addr,serial));
		indexForReplica.insert(std::make_pair(serial,std::make_pair(0,rmgReplica)));

	  	stored = true;
		integrityChecker();
	}

	return stored;
}



bool BndlStorageHelper::deleteBundle(unsigned long  serial)
{
	bool deleted = false;
	LAddress::L3Type addr = LAddress::L3NULL;
	// step 1 : check if the bundle is already stored
	if (existBundle(serial)){
		// step 2 : retrieving recipient address for bundle
		WaveShortMessage *bundle = getBundleBySerial(serial);
		if (bundle == NULL){
			opp_error("BundleSHelper::deleteBundle --- Bundle exists in index but returning null pointer");
		}else{
			addr = bundle->getRecipientAddress();
		}

		// step 2 : delete it from bundleSerials
		bundleSerials.remove(serial);

		// step 3 : delete it from serialsIndexByRecipientAddr
		std::pair<multiIter, multiIter> iterpair = serialsIndexedByRcvAddr.equal_range(addr);
		bool found = false;
		for (multiIter it2 = iterpair.first; it2 != iterpair.second; ++it2) {
		    if (it2->second == serial) {
		    	found = true;
		    	serialsIndexedByRcvAddr.erase(it2);
		        break;
		    }
		}
		if (!found){
			opp_error("BundleSHelper::deleteBundle --- Bundle exists but not found in serialsIndexByRecipientAddr");
		}

		// step 3 : delete it from indexForReplica
		std::map<unsigned long, std::pair<int, int> >::iterator it3 = indexForReplica.find(serial);
		if (it3 == indexForReplica.end()){
			opp_error("BundleSHelper::deleteBundle --- Bundle exists but not found in indexForReplica");
		}else{
			indexForReplica.erase(serial);
		}

		// step 4 : delete it from bundlesIndexBySerial
		bundlesIndexedBySerial.erase(serial);

		// step 5 : check if it is the owner of this WSM and delete it in case
		if (bundle->getOwner()==this){
			delete bundle;
		}

		deleted = true;
		integrityChecker();
	}

	return deleted;
}

bool BndlStorageHelper::deleteBundleUponACK(unsigned long  serial)
{
	bool deleted = deleteBundle(serial);
	if (deleted){
		nbrDeletedBundlesByAck++;
	}
	return deleted;
}

bool BndlStorageHelper::deleteBundleUponTTL(unsigned long  serial)
{
	bool deleted = deleteBundle(serial);
	if (deleted){
		nbrDeletedBundlesByTTL++;
	}
	return deleted;
}


bool BndlStorageHelper::deleteBundleUponCustody(unsigned long  serial)
{
	bool deleted = deleteBundle(serial);
	if (deleted){
		nbrDeletedBundlesByCustody++;
	}
	return deleted;
}

bool BndlStorageHelper::deleteBundleUponNoRmgReplica(unsigned long  serial)
{
	bool deleted = deleteBundle(serial);
	if (deleted){
		nbrDeletedBundlesByNoRmgReplica++;
	}
	return deleted;
}

bool BndlStorageHelper::existBundle(unsigned long  serial)
{
	bool found = false;
	std::map<unsigned long,WaveShortMessage*>::iterator it = bundlesIndexedBySerial.find(serial);
	if (it != bundlesIndexedBySerial.end()){
		found = true;
	}
	return found;
}

WaveShortMessage *BndlStorageHelper::getBundleBySerial(unsigned long  serial)
{
	WaveShortMessage* wsm = NULL;
	std::map<unsigned long,WaveShortMessage*>::iterator it = bundlesIndexedBySerial.find(serial);
	if (it != bundlesIndexedBySerial.end()){
		wsm = it->second;
	}
	return wsm;
}

void BndlStorageHelper::deleteFirstBundle()
{
	if (bundleSerials.size() == storageSize){
		unsigned long serialToDelete = bundleSerials.front();
		if (deleteBundle(serialToDelete)){
			nbrDeletedBundlesByFIFO++;
		}
	}else if (bundleSerials.size() > storageSize){
		opp_error("BundleSHelper::deleteFirstBundle --- Bundles storage structure exceed its maximum size");
	}
}

void BndlStorageHelper::deleteExpiredBundles()
{
	if(withTTL){
		std::vector<unsigned long > oldWSM;
		for (std::map<unsigned long,WaveShortMessage*>::iterator it = bundlesIndexedBySerial.begin(); it != bundlesIndexedBySerial.end(); it++){
			// step 1 : Check if the current bundle is up to date and has not expired
			WaveShortMessage* wsm = it->second;
			double expireTime = wsm->getTimestamp().dbl() + ttl;
			if (hasExpired(expireTime)){
				// step 2 : If the current bundle has expired, add it to bundles to delete
				oldWSM.push_back(wsm->getSerial());
			}
		}

		// step 3 : Delete Expired Bundles
		for (std::vector<unsigned long >::iterator it = oldWSM.begin(); it != oldWSM.end(); it++){
			deleteBundleUponTTL(*it);
		}
	}
}

unsigned int BndlStorageHelper::getNbrStoredBundles()
{
	integrityChecker();
	if (!bundleSerials.empty()){
		return bundleSerials.size();
	}else{
		return 0;
	}
}

void BndlStorageHelper::updateRcvAddrForBundles(LAddress::L3Type newAddr)
{
	// step 1 : Empyting this index, in order to update it
	serialsIndexedByRcvAddr.clear();

	for (std::map<unsigned long,WaveShortMessage*>::iterator it = bundlesIndexedBySerial.begin(); it != bundlesIndexedBySerial.end(); it++){
		// step 2 : Change RecipientAddress for each entry
		WaveShortMessage* wsm = it->second;
		wsm->setRecipientAddress(newAddr);

		// step 3 : Adding each entry to serialsIndexedByRcvAddr index
		unsigned long serial = it->first;
		serialsIndexedByRcvAddr.insert(std::pair<LAddress::L3Type, unsigned long>(newAddr,serial));
	}

	integrityChecker();
}

void BndlStorageHelper::updateSentReplica(unsigned long  serial, int sentReplica)
{
	if (existBundle(serial)){
		std::map<unsigned long, std::pair<int, int> >::iterator it3 = indexForReplica.find(serial);
		if (it3 == indexForReplica.end()){
			opp_error("BundleSHelper::updateSentReplica --- Bundle exists but not found in indexForReplica");
		}else{
			int currentSentReplica = it3->second.first;
			int currentRmgReplica = it3->second.second;
			bool updateEntry = true;

			currentSentReplica += sentReplica;
			if (withLimitedReplica){
				currentRmgReplica-= sentReplica;
				if (currentRmgReplica <= 0){
					deleteBundleUponNoRmgReplica(serial);
					updateEntry = false;
				}
			}

			if(updateEntry){
				indexForReplica[serial] = std::pair<int, int>(currentSentReplica,currentRmgReplica);
			}
		}
	}

	integrityChecker();
}

std::vector<std::pair<WaveShortMessage*,int> > BndlStorageHelper::getStoredBundlesWithReplica(std::set<unsigned long > serialsOfBundles)
{
	// step 1 : Build bundle list to send before reordering
	std::vector<std::pair<WaveShortMessage*, int> >unsortedWSMPair;

	for (std::set<unsigned long>::iterator it = serialsOfBundles.begin(); it != serialsOfBundles.end(); it++){
		unsigned long serial = *it;
		if (existBundle(serial)){
			WaveShortMessage *bundle = getBundleBySerial(serial);
			if (bundle == NULL){
				opp_error("BundleSHelper::getStoredBundlesWithReplica --- Bundle exists in index but returning null pointer");
			}

			std::map<unsigned long, std::pair<int, int> >::iterator it2 = indexForReplica.find(serial);
			if (it2 == indexForReplica.end()){
				opp_error("BundleSHelper::getStoredBundlesWithReplica --- Bundle exists but not found in indexForReplica");
			}
			int sentReplica = it2->second.first;

			unsortedWSMPair.push_back(std::pair<WaveShortMessage*, int>((bundle), sentReplica));
		}
	}

	return unsortedWSMPair;
}

std::vector<std::pair<WaveShortMessage*,int> > BndlStorageHelper::getStoredBundlesWithReplica()
{
	std::set<unsigned long > serialsOfBundles = std::set<unsigned long >(bundleSerials.begin(), bundleSerials.end());
	return getStoredBundlesWithReplica(serialsOfBundles);
}


int BndlStorageHelper::computeNbrReplicaToSend(unsigned long  serial)
{
	int nbrReplicaToSend = 0;
	if (existBundle(serial)){
		if (withLimitedReplica){
			std::map<unsigned long, std::pair<int, int> >::iterator it3 = indexForReplica.find(serial);
			if (it3 == indexForReplica.end()){
				opp_error("BundleSHelper::computeNbrReplicaToSend --- Bundle exists but not found in indexForReplica");
			}else{
				int currentRmgReplica = it3->second.second;

				if ((currentRmgReplica <= 0 ) || (currentRmgReplica > maxNbrReplica )){
					opp_error("BundleSHelper::computeNbrReplicaToSend --- Invalid remaining replica");
				}else {
					if (currentRmgReplica == 1){
						nbrReplicaToSend = 1;
					}else{
						nbrReplicaToSend =  currentRmgReplica / 2;
					}
				}
			}
		}else{
			opp_error("BundleSHelper::computeNbrReplicaToSend --- Requesting computation of NbrReplicaToSend but not in limitedReplica mode");
		}
	}else{
		opp_error("BundleSHelper::computeNbrReplicaToSend --- Requesting computation of NbrReplicaToSend but Bundle does not exist");
	}
	return nbrReplicaToSend;
}

int BndlStorageHelper::getNbrRmgReplica(unsigned long  serial)
{
	int nbrRmgReplica = 0;
	if (existBundle(serial)){
		if (withLimitedReplica){
			std::map<unsigned long, std::pair<int, int> >::iterator it3 = indexForReplica.find(serial);
			if (it3 == indexForReplica.end()){
				opp_error("BundleSHelper::getNbrRmgReplica --- Bundle exists but not found in indexForReplica");
			}else{
				int currentRmgReplica = it3->second.second;

				nbrRmgReplica = currentRmgReplica;
			}
		}else{
			opp_error("BundleSHelper::getNbrRmgReplica --- Getting NbrRmgReplica but not in limitedReplica mode");
		}
	}else{
		opp_error("BundleSHelper::getNbrRmgReplica --- Getting NbrRmgReplica but Bundle does not exist");
	}
	return nbrRmgReplica;
}

bool BndlStorageHelper::hasExpired(double expireTime)
{
	bool hasExpired = false;
	double currentTime = simTime().dbl();

	if(withTTL & ( currentTime > expireTime)){
		hasExpired = true;
	}

	return hasExpired;
}

void BndlStorageHelper::integrityChecker()
{
	// If not all Data Structures are equal in size, there is an error and we must throw an error message
	if (! ((bundleSerials.size() == bundlesIndexedBySerial.size()) && (bundlesIndexedBySerial.size() == serialsIndexedByRcvAddr.size()) && (serialsIndexedByRcvAddr.size() == indexForReplica.size()))){
		opp_error("BundleSHelper::integrityChecker --- Data Structure dedicated to storage are not equal");
	}
}


