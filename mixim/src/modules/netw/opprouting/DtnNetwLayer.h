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

#ifndef __MIXIM_DTNNETWLAYER_H_
#define __MIXIM_DTNNETWLAYER_H_

#include <omnetpp.h>
#include <cassert>

#include <map>
#include <set>
#include <iterator>

#include "BaseNetwLayer.h"
#include "WaveShortMessage_m.h"
#include "NetwPkt_m.h"
#include "cmessage.h"
#include "NetwControlInfo.h"
#include "ArpInterface.h"
#include "SimpleAddress.h"
#include "cmodule.h"
#include "Mac80211Pkt_m.h"
#include "Prophet_m.h"
#include "BundleMeta.h"
#include "SimpleContactStats.h"
#include "ClassifiedContactStats.h"
#include "iterator"
#include "BaseMobility.h"
#include "fstream"



/**
 * TODO - Generated class
 */

typedef std::map<unsigned long,WaveShortMessage*> innerIndexMap;
typedef std::map<unsigned long,WaveShortMessage*>::iterator innerIndexIterator;

typedef std::map<LAddress::L3Type, innerIndexMap>::iterator bundlesIndexIterator;

typedef std::map<LAddress::L3Type, std::list<unsigned long> >::iterator iteratorContactID;
typedef std::map<unsigned long, SimpleContactStats>::iterator iteratorContactStats;

class DtnNetwLayer : public BaseNetwLayer {
	/*******************************************************************
	** 							Variables section
	********************************************************************/
  protected:

	double sumOfContactDur;

	int nbrContacts;

	cOutVector contactDurVector;

	double sumOfInterContactDur;

	int nbrRecontacts;

	cOutVector intercontactDurVector;

	std::map<LAddress::L3Type, double> contacts;

	std::map<LAddress::L3Type, double> endContactTime;

	std::map<LAddress::L3Type, std::list<unsigned long> > indexContactID;

	std::map<unsigned long, SimpleContactStats> indexContactStats;

	ClassifiedContactStats Global;
	bool withGlobal;
	bool withCDFForGlobal;

	ClassifiedContactStats Succ;
	bool withSucc;
	bool withCDFForSucc;

	ClassifiedContactStats Fail;
	bool withFail;
	bool withCDFForFail;

	bool recordContactStats;

	/**
	 * @brief Prophet Control Kinds used when notified by the lower layer (i.e Mac1609_4_Opp & NicEntryDebug)
	 */
	enum DtnNetwControlKinds {
		NEWLY_CONNECTED = LAST_BASE_NETW_CONTROL_KIND,
		NEW_NEIGHBOR = NEWLY_CONNECTED + 10,
		NO_NEIGHBOR_AND_DISCONNECTED = NEWLY_CONNECTED + 20,
		NEW_NEIGHBOR_GONE = NEWLY_CONNECTED + 30,
		RESTART = NEWLY_CONNECTED + 40,
		FORCED_RESTART = NEWLY_CONNECTED + 50,
	};

	long nbrL3Sent;
	long nbrL3Received;

  	double delayed;

  	double delayedFrag;

  	/**
  	 * Boolean for the activation of PRoPHET ACK mecanism
  	 */
  	bool withAck;

	int demandedAckedBundle;

	int bundlesReceived;

  	/**
  	 * Map structures for ACKs
  	 */
  	std::list<BundleMeta> acks;

  	bool withTTL;
  	int ttl;
  	int nbrDeletedWithTTL;

  	/**
   	 * Specific map with K as serial of WSM &
  	 * V as a bndl_meta struct.
  	 * This structure simplify the search of ACKs
  	 */
  	std::map<unsigned long,BundleMeta> acksIndex;

  	/**
  	 * Size of acks structure
  	 */
  	int ackStructureSize;

  	/** Size of the WMS Storage structure */
  	int bundlesStructureSize;

  	/** Fifo structure for WMS Storage*/
  	std::list<WaveShortMessage*> bundles;

  	/** Specific map with K as recipient address of WSMessage &
  	 * V as a pointer to WSMessage.
  	 * This structure simplify the search of WSMessage destinated
  	 * to a specific node
  	 * */
  	std::map<LAddress::L3Type,innerIndexMap > bundlesIndex;

  	/** Boolean to verify if the transmission is possible */
  	bool canITransmit;

  	/**
  	 * Equiped vehicle in pourcentage, by default all vehicles
  	 * are equiped (100% = 1)
  	 */
  	double equipedVehPc;

  	/**
  	 * Determine if this vehicle is equiped or not
  	 */
  	bool isEquiped;

  	int maxPcktLength;

  	bool dontFragment;

  	int dataLength;

  	std::vector<double> cutPoitsCDF;

  	int deletedBundlesWithAck;

  	double lastBundleUpdate;

  	bool withRestart;

  	int nbrRestartedIEP;

  	int nbrCancelRestartedIEP;

  	std::map<LAddress::L3Type, double> lastBundleProposal;

    // self message to restart IEP after reception of new Bundles
    cMessage* restartIEP;

    std::set<LAddress::L3Type> neighborsAddress;

    bool withConnectionRestart;

	std::ofstream ioFile;

	simtime_t updateInterval;

	cMessage* updateTraceMsg;

	BaseMobility *mobility;


	/*******************************************************************
	** 							Methods section
	********************************************************************/

  public:
    virtual void initialize(int stage);

    virtual void finish();

    virtual bool forceRestartIEP(LAddress::L3Type addr);

  protected:
  	/**
  	 * Function that define offered bundles for the BundleOffer sub-phase of IEP Phase
  	 */
  	virtual std::vector<std::list<BundleMeta> >defineBundleOffer(NetwPkt *netwPkt);

  	/**
  	 * @brief Function that check if the WaveShortMessage identified by
  	 * @param *msg is currently stored in this node
  	 */
  	virtual bool exist(WaveShortMessage *msg);

  	/**
  	 * @brief Function that check if the WaveShortMessage identified by
  	 * @param bndlMeta is currently stored in this node
  	 */
  	virtual bool exist(BundleMeta bndlMeta);

  	/**
  	 * @brief Function that erase the WaveShortMessage identified by
  	 * @param *msg
  	 */
  	virtual bool erase(WaveShortMessage *msg);

  	/**
  	 * @brief Function that erase the WaveShortMessage identified by
  	 * @param bndlMeta
  	 */
  	virtual bool erase(BundleMeta bndlMeta);

  	/**
  	 * @brief Function that check if an ack  identified by
  	 * @param *msg serial is currently stored in this node
  	 */
  	virtual bool ackExist(WaveShortMessage *msg);

  	/**
  	 * @brief Function that check if an ack identified by
  	 * @param bndlMeta serial is currently stored in this node
  	 */
  	virtual bool ackExist(BundleMeta bndlMeta);

  	/**
  	 * @brief Function that check if the WaveShortMessage identified by
  	 * @param bndlMeta is currently stored in this node then deleted it
  	 */
  	virtual bool existAndErase(BundleMeta bndlMeta);

  	/*
  	 * Function that store bundles according to the current Queuing Strategy
  	 */
  	virtual void storeBundle(WaveShortMessage *msg);

  	virtual void storeACK(BundleMeta meta);

  	/*
  	 * Convert a string to L3Address
  	 */
  	virtual LAddress::L3Type getAddressFromName(const char * name);

  	/*
  	 * Convert a string to time
  	 */
  	virtual double getTimeFromName(const char * name);

  	virtual void deleteOldBundle(int ttl);

  	/** @brief Handle messages from upper layer */
  	virtual void handleUpperMsg(cMessage* msg);

  	/** @brief Handle messages from lower layer */
  	virtual void handleLowerMsg(cMessage* msg);

  	/** @brief Handle self messages */
	virtual void handleSelfMsg(cMessage* msg);

  	/** @brief Handle control messages from lower layer */
  	virtual void handleLowerControl(cMessage* msg);

  	/** @brief Handle control messages from lower layer */
  	virtual void handleUpperControl(cMessage* msg);

  	/*
  	 * Function to decide if we have to restart IEP
  	 */
  	virtual bool haveToRestartIEP(simtime_t t);

  	/*
  	 * generate contact serial
  	 */
  	virtual unsigned long generateContactSerial( int myAddr, int seqNumber, int otherAddr);

  	/*
  	 * start recording
  	 */
  	virtual unsigned long startRecordingContact(int addr, double time);

  	/*
  	 * start recording
  	 */
  	virtual unsigned long startRecordingContact(int addr, unsigned long contactID);

  	/*
  	 * end recording
  	 */
  	virtual unsigned long endRecordingContact(int addr, double time);

  	virtual unsigned long endRecordingContact(unsigned long contactID, bool hasForcedEnding);

  	virtual void updatingL3Sent(){
  		nbrL3Sent++;
  	}

  	virtual void updatingL3Received(){
  		nbrL3Received++;
  	}

  	virtual void recordBeginContactStats(LAddress::L3Type addr, double time);

  	virtual void recordEndContactStats(LAddress::L3Type addr, double time);

  	virtual void recordRecontactStats(LAddress::L3Type addr, double time);

//  	void updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind);

  	virtual void classify(SimpleContactStats* newContact);

  	virtual void classifyAll();

  	virtual void initAllClassifier();

  	virtual void recordClassifier(ClassifiedContactStats classifier);

  	virtual void recordAllClassifier();

  	virtual void recordAllScalars();

  	virtual void updateTraceFile(LAddress::L3Type addr, double time, char* type);

  	virtual void periodicUpdateTraceFile();

  public:
	/*
	 * Getter for isEquiped boolean
	 */
	bool isAnEquipedVehicle(){
		return isEquiped;
	}

	virtual int numInitStages() const {
		return 3;
	}
};

#endif
