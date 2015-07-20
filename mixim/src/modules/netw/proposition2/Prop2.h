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
 * Arslane HAMZA CHERIF 18/07/2015
 */

#ifndef PROP2_H_
#define PROP2_H_
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
#include "ccProphet_m.h"
#include "BundleMeta.h"
#include "SimpleContactStats.h"
#include "ClassifiedContactStats.h"
#include "iterator"
//#include "limits"
//#include "../../messages/WaveShortMessage_m.h"
//#include "../../utility/opprouting/Prophet_Enum.h"
//#include "../../messages/opprouting/Prophet_m.h"

typedef std::map<LAddress::L3Type, double>::iterator predsIterator;

typedef std::map<LAddress::L3Type, vector <double> >::iterator historicPredsIterator;
typedef std::map<LAddress::L3Type, vector <double> >::reverse_iterator rhistoricPredsIterator;

typedef std::map<int ,WaveShortMessage*> innerIndexMap;
typedef std::map<int ,WaveShortMessage*>::iterator innerIndexIterator;

typedef std::map<LAddress::L3Type, innerIndexMap>::iterator bundlesIndexIterator;

class Prop2: public BaseNetwLayer {
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

	enum nodeClass {
		VPA,
		Vehicle_Type_I,
		Vehicle_Type_II,
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

	nodeClass myClass;

	/** Map of vectors for saving historics of encounters */
	std::map<LAddress::L3Type, std::vector<double> > historyOfPreds;

	int historySize;

	LAddress::L3Type convergeCastTo;

	/** Bool for start aging actual convergeCastTo after the end of last contact*/
	bool canIAge;

	/** delivery predictability max value */
	double PEncMax;
	/** delivery predictability initialization constant*/
	double PFirstContact;
	/** delivery predictability min value */
	double PMinThreshold;
	/** delta threshold where that we oblige us to send data*/
	double deltaThreshold;

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

	/**
 	 * Specific map with K as serial of WSM &
	 * V as a bndl_meta struct.
	 * This structure simplify the search of ACKs
	 */
	std::map<int,BundleMeta> acksIndex;

	/**
	 * Size of acks structure
	 */
	unsigned int ackStructureSize;

	/** delivery predictabilities */
	std::map<LAddress::L3Type, double> preds;
	//std::map<DTNHost, double> preds;

	/** last encouter timestamp (sim)time */
	std::map<LAddress::L3Type, double> lastEncouterTime;
	//std::map<DTNHost, double> lastEncouterTime;

	/** last delivery predictability update (sim)time */
	double lastAgeUpdate;

	/** Size of the WMS Storage structure */
	unsigned int bundlesStructureSize;

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

	/*******************************************************************
	** 							Metrics variables section
	********************************************************************/
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

	double sumOfContactDur;

	int nbrContacts;

	cOutVector contactDurVector;

	std::map<LAddress::L3Type, double> contacts;

	std::map<LAddress::L3Type, std::set<SimpleContactStats> > simpleContacts;

	ClassifiedContactStats global;

	ClassifiedContactStats successful;

	ClassifiedContactStats failed;

	ClassifiedContactStats ribFail;

	ClassifiedContactStats failedAtBundleOffer;

	ClassifiedContactStats responseFail;

	ClassifiedContactStats failedAtBundle;

	ClassifiedContactStats failedAtBundleAck;

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

	int notCorrectlyDeleted;

	int bundlesReceived;


	int nbrSimpleContactStats;


	/*******************************************************************
	** 							end of metrics variables section
	********************************************************************/

/*******************************************************************
**
** 							Methods section
**
********************************************************************/
private:
	/** Function that define the node class*/
	void DefineNodeClass();

	int vpaDestAddr();

	/** Function for updating preds and class*/
	void updateDataWhenCCN(const LAddress::L3Type convergeCastAddr);

	/** Function for updating preds and class*/
	void updateDataFor(const LAddress::L3Type convergeCastAddr, const nodeClass encounteredClass, std::map<LAddress::L3Type, std::vector<double> > historyOfPredsForB);

	/** Function for aging P(A,B)*/
	void ageDeliveryPreds();

	/** Function for updating & exchanging probabilities */
	void update(ccProphet *prophetPkt);

	/** Function for executing all actions of Initiator Role in the IEP Phase*/
	void executeInitiatorRole(short  kind, ccProphet *prophetPkt = NULL, LAddress::L3Type destAddr = 0);

	/** Function for executing all actions of Listener Role in the IEP Phase*/
	void executeListenerRole(short  kind, ccProphet *prophetPkt = NULL, LAddress::L3Type destAddr = 0);

	/** Function for preparing ccProphet message */
	ccProphet* prepareProphet(short kind, LAddress::L3Type srcAddr, LAddress::L3Type destAddr,
				std::list<BundleMeta>* meta = NULL, std::map<LAddress::L3Type,std::vector<double> >* histOfPreds = NULL, WaveShortMessage* msg = NULL);

	/**
	 * Function that define offered bundles for the BundleOffer sub-phase of IEP Phase
	 */
	std::list<BundleMeta> defineBundleOffer(ccProphet *prophetPkt);

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

	/** @brief Encapsulate higher layer packet into an NetwPkt */
	virtual NetwPkt* encapsMsg(cPacket*);

	virtual cObject *const setDownControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr);

	virtual cObject *const setUpControlInfo(cMessage *const pMsg, const LAddress::L3Type& pSrcAddr);

	/*******************************************************************
	** 							Metrics methods section
	********************************************************************/

	/*
	 * Function for collecting data about predictions
	 */
	void recordPredsStats();

	void updatingL3Sent(){
		nbrL3Sent++;
	};

	void updatingL3Received(){
		nbrL3Received++;
	}

	void recordBeginContactStats(LAddress::L3Type addr, double time);

	void recordEndContactStats(LAddress::L3Type addr, double time);

	void recordRecontactStats(LAddress::L3Type addr, double time);

	void updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind);

	void recordBeginSimplContactStats(LAddress::L3Type addr, double time);

	void recordEndSimpleContactStats(LAddress::L3Type addr, double time);

	void classify(SimpleContactStats newContact);

	void classifyRemaining();

	void recordClassifier(ClassifiedContactStats classifier);

	void recordAllClassifier();


	/*******************************************************************
	** 							End of metrics methods section
	********************************************************************/
public:

 	virtual int numInitStages() const {
    	return 3;
    }

	const LAddress::L3Type getMyNetwAddress(){
		return myNetwAddr;
	}

	std::pair<SimpleContactStats, std::set<SimpleContactStats>::iterator > getSimpleContactStats(LAddress::L3Type addr, double creationTime);

	SimpleContactStats getLastSimpleContactStats(LAddress::L3Type addr, double time);

	SimpleContactStats getLastSimpleContactStats(LAddress::L3Type addr);

	void updateSimpleContactStats(LAddress::L3Type addr, SimpleContactStats newContact, std::set<SimpleContactStats>::iterator iterator);

	virtual void initialize(int stage);
	virtual void finish();
	virtual ~Prop2();

};

#endif /* PROP2_H_ */
