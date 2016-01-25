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
/*
 * Arslane HAMZA CHERIF 10/02/2014
 */

#ifndef PROPHETV2_H_
#define PROPHETV2_H_
#include <cassert>

#include <cobject.h>
#include <iostream>
#include <map>
#include <list>
#include <iterator>
#include <math.h>
#include <algorithm>

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

typedef std::map<LAddress::L3Type, double>::iterator predsIterator;

typedef std::map<unsigned long ,WaveShortMessage*> innerIndexMap;
typedef std::map<unsigned long,WaveShortMessage*>::iterator innerIndexIterator;

typedef std::map<LAddress::L3Type, innerIndexMap>::iterator bundlesIndexIterator;

typedef std::map<LAddress::L3Type, std::list<unsigned long> >::iterator iteratorContactID;
typedef std::map<unsigned long, SimpleContactStats>::iterator iteratorContactStats;

class ProphetV2: public BaseNetwLayer {
/*******************************************************************
**
** 							Variables section
**
********************************************************************/
public:
	/**
	 * @brief The message kinds used by PRoPHETv2 as implemented.
	 * Defined as described in RFC 6693,
	 * the only difference is the Bundle Enum that is not defined in RFC
	 * it value is taken from Experimental values (bounded between 0xD0 & 0xFF)
	 */
	enum Prophetv2MessageKinds {
		HELLO = 0x00,
		ERROR = 0x01,
		RIBD  = 0xA0,
		RIB   = 0xA1,
		Bundle_Offer = 0xA4,
		Bundle_Response = 0xA5,
		Bundle_Ack = 0xD0,
		Bundle = 0xFF,
	};
	/**
	 * @brief Prophet Control Kinds used when notified by the lower layer (i.e Mac1609_4_Opp & NicEntryDebug)
	 */
	enum prophetNetwControlKinds {
		NEWLY_CONNECTED = LAST_BASE_NETW_CONTROL_KIND,
		NEW_NEIGHBOR = NEWLY_CONNECTED + 10,
		NO_NEIGHBOR_AND_DISCONNECTED = NEWLY_CONNECTED + 20,
		NEW_NEIGHBOR_GONE = NEWLY_CONNECTED + 30,
	};

protected:

	/**
	 * Enumeration for Forwarding strategy defined by RFC 6693
	 */
	enum t_prophet_forward {
		FWD_GRTR	=1,
		FWD_GTMX	=2,
		FWD_GTHR	=3,
		FWD_GRTRplus=4,
		FWD_GTMXplus=5,
		FWD_GRTRsort=6,
		FWD_GRTRmax	=7,
	};

	/**
	 * Enumeration for Queuing strategy defined by RFC 6693
	 */
	enum t_prophet_queuing {
		QUEUING_FIFO=1,
		QUEUING_MOFO,
		QUEUING_MOPR,
		QUEUING_LinearMOPR,
		QUEUING_SHLI,
		QUEUING_LEPR,
	};

	/**
	 * Comparator used to sort Bundles_Offer when using @FWD_GRTRmax forward strategy
	 */
	struct fwdGRTRmaxComparator {
	  bool operator() (std::pair<LAddress::L3Type, double> i,std::pair<LAddress::L3Type, double> j)
	  { return (i.second>j.second);}
	} fwdGRTRmax_CompObject;

private:

	int delayed;

	/** delivery predictability max value */
	double PEncMax;
	/** delivery predictability initialization constant*/
	double PFirstContact;
	/** delivery predictability min value */
	double PMinThreshold;
	/** typical interconnection time in seconds*/
	double I_TYP;
	/** delivery predictability transitivity scaling constant default value */
	double Beta;
	/** delivery predictability aging constant */
	double GAMMA;

	t_prophet_forward fwdStrategy;

	t_prophet_queuing qStrategy;
	/**
	 * Number of seconds in time unit
	 * How many seconds one time unit is when calculating aging of
	 * delivery predictions. Should be tweaked for the scenario.*/
	int secondsInTimeUnit;

	/**
	 * Boolean for the activation of PRoPHET ACK mecanism
	 */
	bool withAck;

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

	/** delivery predictabilities */
	std::map<LAddress::L3Type, double> preds;
	//std::map<DTNHost, double> preds;

	/** last encouter timestamp (sim)time */
	std::map<LAddress::L3Type, double> lastEncouterTime;
	//std::map<DTNHost, double> lastEncouterTime;

	/** last delivery predictability update (sim)time */
	double lastAgeUpdate;

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

	/*
	 * Threshold for Interval of time between first and last preds
	 */
	double I_Preds;

	std::vector<LAddress::L3Type> addrCommunities;

	/*******************************************************************
	** 							Metrics variables section
	********************************************************************/

	bool recordContactStats;

	long nbrL3Sent;
	long nbrL3Received;
//    cLongHistogram sentStats;
//    cOutVector sentVector;
//    cLongHistogram receivedStats;
//    cOutVector receivedVector;

	cOutVector nbrPredsVector;
	cOutVector predsMean;
	cOutVector predsMax;
	cOutVector predsMin;
	cOutVector predsVariance;

	cOutVector *predsMeanCommunities;

//	cOutVector *predsForRC;

	cOutVector predsForRC;

	cOutVector nbrContactsForRCVect;
	std::map<LAddress::L3Type, int > nbrContactsForRC;

	double sumOfContactDur;

	int nbrContacts;

	cOutVector contactDurVector;

	std::map<LAddress::L3Type, double> contacts;

	std::map<LAddress::L3Type, double> endContactTime;

	std::map<LAddress::L3Type, std::set<SimpleContactStats> > simpleContacts;

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

	ClassifiedContactStats FailRIB;
	bool withFailRIB;
	bool withCDFForFailRIB;

	ClassifiedContactStats FailBndlOffer;
	bool withFailBndlOffer;
	bool withCDFForFailBndlOffer;

	ClassifiedContactStats FailBndlResp;
	bool withFailBndlResp;
	bool withCDFForFailBndlResp;

	ClassifiedContactStats FailBndl;
	bool withFailBndl;
	bool withCDFForFailBndl;

	ClassifiedContactStats FailBndlAck;
	bool withFailBndlAck;
	bool withCDFForFailBndlAck;

	double sumOfInterContactDur;

	int nbrRecontacts;

	cOutVector intercontactDurVector;

	/**
	 * Contact between 2 nodes, where the initiator role had received packets of kind Bundle
	 */
	int nbrSuccessfulContact;

	int nbrFailedContactBeforeRIB;
	int nbrFailedContactAtRIB;
	int nbrFailedContactAtBundle_Offer;
	int nbrFailedContactAtBundle_Response;

	/**
	 * Map for the state of the initator role during contact
	 */
	std::map<LAddress::L3Type, Prophetv2MessageKinds> contactState;

	int deletedBundlesWithAck;

	int demandedAckedBundle;

	int bundlesReceived;

	int nbrSimpleContactStats;

	/**
	 * Stats for repeated contacts only
	 */

	std::map<LAddress::L3Type, int > nbrRepeatedContact;
	cLongHistogram histMaxRepeatedContact;

	int maxForRC;

	/*
	 * contact duration for RepeatedContact (RC)
	 */
	std::map<int, std::list<double> > contactDurForRC;

	std::map<LAddress::L3Type, std::list<double> > contactDurByAddr;
	cLongHistogram contactDurHist;

	/*
	 * Intercontact duration for RepeatedContact (RC)
	 */
	std::map<int, std::list<double> > interContactDurForRC;

	std::map<LAddress::L3Type, std::list<double> > interContactDurByAddr;
	cLongHistogram interContactDurHist;

	/*******************************************************************
	** 							end of metrics variables section
	********************************************************************/

/*******************************************************************
**
** 							Methods section
**
********************************************************************/
private:
	/** Function for calculating P(A,B)*/
	void updateDeliveryPredsFor(const LAddress::L3Type BAdress);

	/** Function for calculating P(A,C) with P(A,B) & P(B,C)*/
	void updateTransitivePreds(const LAddress::L3Type BAdress,
			std::map<LAddress::L3Type, double> Bpreds);

	/** Function for aging P(A,B)*/
	void ageDeliveryPreds();

	/** Function for updating & exchanging probabilities */
	void update(Prophet *prophetPkt);

	/** Function for partially updating & exchanging probabilities */
	void partialUpdate(Prophet *prophetPkt);

	/** Function for executing all actions of Initiator Role in the IEP Phase*/
	void executeInitiatorRole(short  kind, Prophet *prophetPkt);

	/** Function for executing all actions of Listener Role in the IEP Phase*/
	void executeListenerRole(short  kind, Prophet *prophetPkt);

	/** Function for checking if we must abord the communication due to no more things to do or due to an error	 */
	bool abortConnection(short  kind, Prophet *prophetPkt);

	/** Function for preparing Prophet message */
	Prophet* prepareProphet(short kind, LAddress::L3Type srcAddr, LAddress::L3Type destAddr,
				std::list<BundleMeta>* meta = NULL, std::map<LAddress::L3Type,double>* preds = NULL, WaveShortMessage* msg = NULL);

	/**
	 * Function that define offered bundles for the BundleOffer sub-phase of IEP Phase
	 */
	std::vector<std::list<BundleMeta> >defineBundleOffer(Prophet *prophetPkt);

	/**
	 * @brief Function that check if the WaveShortMessage identified by
	 * @param *msg is currently stored in this node
	 */
	bool exist(WaveShortMessage *msg);

	/**
	 * @brief Function that check if the WaveShortMessage identified by
	 * @param bndlMeta is currently stored in this node
	 */
	bool exist(BundleMeta bndlMeta);

	/**
	 * @brief Function that check if an ack  identified by
	 * @param *msg serial is currently stored in this node
	 */
	bool ackExist(WaveShortMessage *msg);

	/**
	 * @brief Function that check if an ack identified by
	 * @param bndlMeta serial is currently stored in this node
	 */
	bool ackExist(BundleMeta bndlMeta);

	/**
	 * @brief Function that check if the WaveShortMessage identified by
	 * @param bndlMeta is currently stored in this node then deleted it
	 */
	bool existAndErase(BundleMeta bndlMeta);

	/*
	 * Function that store bundles according to the current Queuing Strategy
	 */
	void storeBundle(WaveShortMessage *msg);

	void storeACK(BundleMeta meta);

	/*
	 * Convert a string to L3Address
	 */
	LAddress::L3Type getAddressFromName(const char * name);

	/*
	 * Convert a string to time
	 */
	double getTimeFromName(const char * name);

	void deleteOldBundle(int ttl);

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

	virtual cObject *const setDownControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr);

	virtual cObject *const setUpControlInfo(cMessage *const pMsg, const LAddress::L3Type& pSrcAddr);

	/*******************************************************************
	** 							Metrics methods section
	********************************************************************/

	/*
	 * generate contact serial
	 */
	unsigned long generateContactSerial( int myAddr, int seqNumber, int otherAddr);

	/*
	 * start recording
	 */
	unsigned long startRecordingContact(int addr, double time);

	/*
	 * start recording
	 */
	unsigned long startRecordingContact(int addr, unsigned long contactID);

	/*
	 * end recording
	 */
	unsigned long endRecordingContact(int addr, double time);

	unsigned long endRecordingContact(unsigned long contactID, bool hasForcedEnding);

	void updateContactWhenInit(Prophet* prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind);

	void updateContactWhenList(Prophet* prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind);

	/*
	 * Function for collecting data about predictions
	 */
	void recordPredsStats();

	void updatingL3Sent(){
		nbrL3Sent++;
	}

	void updatingL3Received(){
		nbrL3Received++;
	}

	void recordBeginContactStats(LAddress::L3Type addr, double time);

	void recordEndContactStats(LAddress::L3Type addr, double time);

	void recordRecontactStats(LAddress::L3Type addr, double time);

	void updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind);

	void classify(SimpleContactStats* newContact);

	void classifyAll();

	void initAllClassifier();

	void recordClassifier(ClassifiedContactStats classifier);

	void recordAllClassifier();


	/*******************************************************************
	** 							End of metrics methods section
	********************************************************************/
public:

    virtual int numInitStages() const {
		return 3;
	}

	virtual void initialize(int stage);
	virtual void finish();
	virtual ~ProphetV2();

	/*
	 * Getter for isEquiped boolean
	 */
	bool isAnEquipedVehicle(){
		return isEquiped;
	}

};

#endif /* PROPHETV2_H_ */
