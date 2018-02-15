/*
 * BndlStorageHelper.h
 *
 *  Created on: Feb 7, 2018
 *      Author: arslan
 */

#ifndef BNDLSTORAGEHELPER_H_
#define BNDLSTORAGEHELPER_H_

#include "map"
#include "WaveShortMessage_m.h"
#include "SimpleAddress.h"
#include "limits"

typedef std::map<unsigned long,WaveShortMessage*> innerIndexMap;
typedef std::map<unsigned long,WaveShortMessage*>::iterator innerIndexIterator;

typedef std::map<LAddress::L3Type, innerIndexMap>::iterator bundlesIndexIterator;

typedef std::multimap<LAddress::L3Type,unsigned long>::iterator multiIter;

namespace NS_BndlStorageHelper {
	const int maxInt = std::numeric_limits<int>::max();
};

class BndlStorageHelper : public cObject {

protected:
    /** Size of the WMS Storage structure */
    unsigned int storageSize;

    /** Fifo structure for WMS Storage*/
    std::list<unsigned long> bundleSerials;

    /** Specific map with K as recipient address of WSMessage &
  	 * V as a WSM Serial.
  	 * This structure simplify the search of WSMessage destinated
  	 * to a specific node
  	 * */
    std::multimap<LAddress::L3Type,unsigned long> serialsIndexedByRcvAddr;

    /** Specific map with K as a WSM Serial &
  	 * V as a pointer to the WaveShortMessage.
  	 * */
    std::map<unsigned long,WaveShortMessage*> bundlesIndexedBySerial;

    /** Data Structure for saving Nbr of Replica by Serial
     * first int is the current nbr of replica sent
     * second int is the remaining nbr of replica if using limited nbr of replica
     * */
	std::map<unsigned long, std::pair <int, int> > indexForReplica;

	/** Boolean for using Bundle Time To Live */
    bool withTTL;

    /** Value of Time To Live */
    double ttl;

	/** if Nbr of Replica is fixed */
	bool withLimitedReplica;

	/** Max nbr of replica*/
	int maxNbrReplica;

	/** Stats related to Bundles deleted due to TTL */
	int nbrDeletedBundlesByTTL;

	/** Stats related to Bundles deleted due to ACK */
	int nbrDeletedBundlesByAck;

	/** Stats related to Bundles deleted due to StorageLimit */
	int nbrDeletedBundlesByFIFO;

	/** Stats related to Bundles deleted due to Custody Transfer */
	int nbrDeletedBundlesByCustody;

	/** Stats related to Bundles deleted due to No Rmg Replica */
	int nbrDeletedBundlesByNoRmgReplica;

public:
	BndlStorageHelper();
	BndlStorageHelper(unsigned int sizeOfBundleStorage, bool withTTL, double ttl);
	virtual ~BndlStorageHelper();

	void reInitWithLimitedReplica(int maxNbrReplica);

	bool storeBundle(WaveShortMessage *msg, int rmgReplica = NS_BndlStorageHelper::maxInt);

	bool deleteBundle(unsigned long serial);
	bool deleteBundleUponACK(unsigned long serial);
	bool deleteBundleUponTTL(unsigned long serial);
	bool deleteBundleUponCustody(unsigned long serial);
	bool deleteBundleUponNoRmgReplica(unsigned long serial);

	bool existBundle(unsigned long  serial);

	WaveShortMessage* getBundleBySerial(unsigned long serial);

	void deleteFirstBundle();

	void deleteExpiredBundles();

	unsigned int getNbrStoredBundles();

	std::vector<std::pair<WaveShortMessage*, int> > getStoredBundlesWithReplica(std::set<unsigned long> serialsOfBundles);

	std::vector<std::pair<WaveShortMessage*, int> > getStoredBundlesWithReplica();

	int computeNbrReplicaToSend(unsigned long serial);

	int getNbrRmgReplica(unsigned long serial);

	void updateRcvAddrForBundles(LAddress::L3Type newAddr);

	void updateSentReplica(unsigned long serial, int SentReplica = 1);

	bool hasExpired(double expireTime);

	std::list<unsigned long > getBundleSerials() const
	{
		return std::list<unsigned long >(bundleSerials);
	}

	std::set<unsigned long > getBundleSerialsAsSet() const
	{
		return std::set<unsigned long >(bundleSerials.begin(),bundleSerials.end());
	}

    int getNbrDeletedBundlesByAck() const
    {
        return nbrDeletedBundlesByAck;
    }

    int getNbrDeletedBundlesByFifo() const
    {
        return nbrDeletedBundlesByFIFO;
    }

    int getNbrDeletedBundlesByTtl() const
    {
        return nbrDeletedBundlesByTTL;
    }

    int getNbrDeletedBundlesByCustody() const
    {
        return nbrDeletedBundlesByCustody;
    }

    int getNbrDeletedBundlesByNoRmgReplica() const
    {
        return nbrDeletedBundlesByNoRmgReplica;
    }

protected:
	void integrityChecker();
};

#endif /* BNDLSTORAGEHELPER_H_ */
