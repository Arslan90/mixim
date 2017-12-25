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

#ifndef PROPHETNCV2_H_
#define PROPHETNCV2_H_

#include "DtnNetwLayer.h"
#include "ProphetNCPkt_m.h"
#include "iostream"

typedef std::map<LAddress::L3Type, double>::iterator predsIterator;

//typedef std::map<unsigned long ,WaveShortMessage*> innerIndexMap;
//typedef std::map<unsigned long,WaveShortMessage*>::iterator innerIndexIterator;
//
//typedef std::map<LAddress::L3Type, innerIndexMap>::iterator bundlesIndexIterator;
//
//typedef std::map<LAddress::L3Type, std::list<unsigned long> >::iterator iteratorContactID;
//typedef std::map<unsigned long, SimpleContactStats>::iterator iteratorContactStats;

class ProphetNCV2: public DtnNetwLayer {
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

//	int delayed;
//
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

	/** delivery predictabilities */
	std::map<LAddress::L3Type, double> preds;

	/** last encouter timestamp (sim)time */
	std::map<LAddress::L3Type, double> lastEncouterTime;

	/** last delivery predictability update (sim)time */
	double lastAgeUpdate;

	/*
	 * Threshold for Interval of time between first and last preds
	 */
	double I_Preds;

	bool withPartialUpdate;

	bool withPredLength;

	/*******************************************************************
	** 							Metrics variables section
	********************************************************************/

	cOutVector nbrPredsVector;
	cOutVector predsMean;
	cOutVector predsMax;
	cOutVector predsMin;
	cOutVector predsVariance;
	cOutVector predForVPA;

	std::map<LAddress::L3Type, cOutVector* > predsForRC;

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

	int maxForRC;

	/*
	 * contact duration for RepeatedContact (RC)
	 */
	std::map<int, std::list<double> > contactDurForRC;

	std::map<LAddress::L3Type, std::list<double> > contactDurByAddr;

	/*
	 * Intercontact duration for RepeatedContact (RC)
	 */
	std::map<int, std::list<double> > interContactDurForRC;

	std::map<LAddress::L3Type, std::list<double> > interContactDurByAddr;

	/*
	 * Bool withEMethod standing for erroneous method, in order to evaluate the impact of the previous error
	 */
	bool withEMethod;

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
	void update(ProphetNCPkt *prophetPkt);

	/** Function for partially updating & exchanging probabilities */
	void partialUpdate(ProphetNCPkt *prophetPkt);

	/** @brief Handle messages from lower layer */
	virtual void handleLowerMsg(cMessage* msg);

	/** @brief Handle self messages */
	virtual void handleSelfMsg(cMessage* msg);

//	/** @brief Handle control messages from lower layer */
//	virtual void handleUpperControl(cMessage* msg);

	/*******************************************************************
	** 							Metrics methods section
	********************************************************************/
	/*
	 * Function for collecting data about predictions
	 */
	void recordPredsStats();

	void recordEndContactStats(LAddress::L3Type addr, double time);

	void recordRecontactStats(LAddress::L3Type addr, double time);

	void updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind);

	void classify(SimpleContactStats* newContact);

	void initAllClassifier();

	void recordAllClassifier();

  	void sendingHelloMsg();

  	void handleHelloMsg(ProphetNCPkt *netwPkt);

  	void sendingRIBMsg( LAddress::L3Type nodeAddr);

  	void handleRIBMsg(ProphetNCPkt *netwPkt);

  	void sendingBndlOfferMsg(LAddress::L3Type nodeAddr, std::map<LAddress::L3Type, double> predsOfNode);

  	void handleBundleOfferMsg(ProphetNCPkt *netwPkt);

  	void sendingBundleResponseMsg( LAddress::L3Type nodeAddr, std::set<unsigned long> wsmResponseBndl);

  	void handleBundleResponseMsg(ProphetNCPkt *netwPkt);

  	void sendingBundleMsg(LAddress::L3Type destAddr, int destType, std::vector<WaveShortMessage* >  wsmToSend);

  	void handleBundleMsg(ProphetNCPkt *netwPkt);

	void sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmFinalDeliverd);

  	void handleBundleAckMsg(ProphetNCPkt *netwPkt);

	/*******************************************************************
	** 							End of metrics methods section
	********************************************************************/
public:

    virtual int numInitStages() const {
		return 3;
	}

	virtual void initialize(int stage);
	virtual void finish();
	virtual ~ProphetNCV2();

};

#endif /* PROPHETNCV2_H_ */
