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

#ifndef PROCCV2_H_
#define PROCCV2_H_

#include "cmodule.h"
#include "cmessage.h"
#include <omnetpp.h>
#include "DtnNetwLayer.h"
#include "Procc_m.h"


typedef std::map<LAddress::L3Type, double>::iterator predsIterator;

typedef std::map<LAddress::L3Type, vector <double> >::iterator historicPredsIterator;
typedef std::map<LAddress::L3Type, vector <double> >::reverse_iterator rhistoricPredsIterator;

//typedef std::map<int ,WaveShortMessage*> innerIndexMap;
//typedef std::map<int ,WaveShortMessage*>::iterator innerIndexIterator;

//typedef std::map<LAddress::L3Type, innerIndexMap>::iterator bundlesIndexIterator;

//typedef std::map<LAddress::L3Type, std::list<unsigned long> >::iterator iteratorContactID;
//typedef std::map<unsigned long, SimpleContactStats>::iterator iteratorContactStats;

class ProccV2: public DtnNetwLayer {
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

protected:

	bool canIAge;

	nodeClass myClass;

	/** Map of vectors for saving historics of encounters */
	std::map<LAddress::L3Type, std::vector<double> > historyOfPreds;

	int historySize;

	LAddress::L3Type convergeCastTo;

	/** delivery predictability min value */
	double PMinThreshold;

	/** delivery predictability transitivity scaling constant default value */
	double Alpha;
	/** delivery predictability aging constant */
	double GAMMA;
	/** delta for comparing preds for CCN */
	double Delta;

	t_prophet_forward fwdStrategy;

	t_prophet_queuing qStrategy;
	/**
	 * Number of seconds in time unit
	 * How many seconds one time unit is when calculating aging of
	 * delivery predictions. Should be tweaked for the scenario.*/
	int secondsInTimeUnit;

	/** delivery predictabilities */
	std::map<LAddress::L3Type, double> preds;
	//std::map<DTNHost, double> preds;

	/** last encouter timestamp (sim)time */
	std::map<LAddress::L3Type, double> lastEncouterTime;
	//std::map<DTNHost, double> lastEncouterTime;

	/** last delivery predictability update (sim)time */
	double lastAgeUpdate;
	/*
	 * Threshold for Interval of time between first and last preds
	 */
	double I_Preds;


    bool delayedRIB;
    bool delayedBndlOffer;
    bool delayedBndlResp;
    bool delayedBundle;
    bool delayedBndlAck;
    bool delayedForFrag;


	/*******************************************************************
	** 							Metrics variables section
	********************************************************************/

//    cLongHistogram sentStats;
//    cOutVector sentVector;
//    cLongHistogram receivedStats;
//    cOutVector receivedVector;

	cOutVector nbrPredsVector;
	cOutVector predsMean;
	cOutVector predsMax;
	cOutVector predsMin;
	cOutVector predsVariance;

	cOutVector classEvolution;
	cOutVector autorizeToAgeEvolution;

	cOutVector predsForRC;

//	std::map<LAddress::L3Type, cOutVector* > predsForRC;

	cOutVector nbrContactsForRCVect;
	std::map<LAddress::L3Type, int > nbrContactsForRC;

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
protected:
	/** Function that define the node class*/
	void DefineNodeClass();

	int vpaDestAddr();

	/** Function for updating preds and class*/
	void updateDataWhenCCN(const LAddress::L3Type convergeCastAddr);

	/** Function for updating preds and class*/
	void updateDataFor(const LAddress::L3Type convergeCastAddr, const nodeClass encounteredClass, std::map<LAddress::L3Type,double> Bpreds);

	/** Function for aging P(A,B)*/
	void ageDeliveryPreds();

	/** Function for updating & exchanging probabilities */
	void update(Procc *prophetPkt);

	/** Function for partially updating & exchanging probabilities */
	void partialUpdate(Procc *prophetPkt);

	/** Function for executing all actions of Initiator Role in the IEP Phase*/
	void executeInitiatorRole(short  kind, Procc *prophetPkt);

	/** Function for executing all actions of Listener Role in the IEP Phase*/
	void executeListenerRole(short  kind, Procc *prophetPkt);

	/** Function for checking if we must abord the communication due to no more things to do or due to an error	 */
	bool abortConnection(short  kind, Procc *prophetPkt);

	/** Function for preparing Prophet message */
	Procc* prepareProphet(short kind, LAddress::L3Type srcAddr, LAddress::L3Type destAddr,
				std::list<BundleMeta>* meta = NULL, std::map<LAddress::L3Type,double>* preds = NULL, WaveShortMessage* msg = NULL);

	/**
	 * Function that define offered bundles for the BundleOffer sub-phase of IEP Phase
	 */
	std::vector<std::list<BundleMeta> >defineBundleOffer(Procc *prophetPkt);

	/**
	 * @brief Function that check if the WaveShortMessage identified by
	 * @param *msg is currently stored in this node
	 */
//	virtual bool exist(WaveShortMessage *msg);

	/**
	 * @brief Function that check if the WaveShortMessage identified by
	 * @param bndlMeta is currently stored in this node
	 */
//	virtual bool exist(BundleMeta bndlMeta);

	/**
	 * @brief Function that check if an ack  identified by
	 * @param *msg serial is currently stored in this node
	 */
//	virtual bool ackExist(WaveShortMessage *msg);

	/**
	 * @brief Function that check if an ack identified by
	 * @param bndlMeta serial is currently stored in this node
	 */
//	virtual bool ackExist(BundleMeta bndlMeta);

	/**
	 * @brief Function that check if the WaveShortMessage identified by
	 * @param bndlMeta is currently stored in this node then deleted it
	 */
//	virtual bool existAndErase(BundleMeta bndlMeta);

	/*
	 * Function that store bundles according to the current Queuing Strategy
	 */
//	virtual void storeBundle(WaveShortMessage *msg);
//
//	virtual void storeACK(BundleMeta meta);
//
//	virtual void deleteOldBundle(int ttl);

	/** @brief Handle messages from upper layer */
//	virtual void handleUpperMsg(cMessage* msg);

	/** @brief Handle messages from lower layer */
	virtual void handleLowerMsg(cMessage* msg);

	/** @brief Handle self messages */
	virtual void handleSelfMsg(cMessage* msg);

	/** @brief Handle control messages from lower layer */
	virtual void handleLowerControl(cMessage* msg);

//	/** @brief Handle control messages from lower layer */
//	virtual void handleUpperControl(cMessage* msg);

//	virtual cObject *const setDownControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr);
//
//	virtual cObject *const setUpControlInfo(cMessage *const pMsg, const LAddress::L3Type& pSrcAddr);

	/*******************************************************************
	** 							Metrics methods section
	********************************************************************/

//	/*
//	 * generate contact serial
//	 */
//	unsigned long generateContactSerial( int myAddr, int seqNumber, int otherAddr);
//
//	/*
//	 * start recording
//	 */
//	unsigned long startRecordingContact(int addr, double time);
//
//	/*
//	 * start recording
//	 */
//	unsigned long startRecordingContact(int addr, unsigned long contactID);
//
//	/*
//	 * end recording
//	 */
//	unsigned long endRecordingContact(int addr, double time);
//
//	unsigned long endRecordingContact(unsigned long contactID, bool hasForcedEnding);

	void updateContactWhenInit(Procc* prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind);

	void updateContactWhenList(Procc* prophetPkt, unsigned long contactID, SimpleContactStats contact, int kind);

	/*
	 * Function for collecting data about predictions
	 */
	void recordPredsStats();

//	void updatingL3Sent(){
//		nbrL3Sent++;
//	}
//
//	void updatingL3Received(){
//		nbrL3Received++;
//	}

//	void recordBeginContactStats(LAddress::L3Type addr, double time);

	void recordEndContactStats(LAddress::L3Type addr, double time);

	void recordRecontactStats(LAddress::L3Type addr, double time);

	void updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind);

	void classify(SimpleContactStats* newContact);

//	void classifyAll();

	void initAllClassifier();

//	void recordClassifier(ClassifiedContactStats classifier);

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
	virtual ~ProccV2();
};

#endif /* PROCCV2_H_ */
