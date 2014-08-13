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
//#include "../../messages/WaveShortMessage_m.h"
//#include "../../utility/opprouting/Prophet_Enum.h"
//#include "../../messages/opprouting/Prophet_m.h"

typedef std::map<LAddress::L3Type, double>::iterator map_it;
typedef std::multimap<LAddress::L3Type, WaveShortMessage*>::iterator multimap_it;

class ProphetV2: public BaseNetwLayer {
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

	struct fwdGRTRmaxComparator {
	  bool operator() (std::pair<LAddress::L3Type, double> i,std::pair<LAddress::L3Type, double> j)
	  { return (i.second>j.second);}
	} fwdGRTRmax_CompObject;

private:
	/** delivery predictability initialization constant*/
	double PEncMax;
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
	//std::map<DTNHost, double> preds;

	/** last encouter timestamp (sim)time */
	std::map<LAddress::L3Type, double> lastEncouterTime;
	//std::map<DTNHost, double> lastEncouterTime;

	/** last delivery predictability update (sim)time */
	double lastAgeUpdate;

	/** Fifo structure for WMS Storage*/
	std::list<WaveShortMessage*> bundles;

	/** Specific map with K as recipient address of WSMessage &
	 * V as a pointer to WSMessage.
	 * This structure simplify the search of WSMessage destinated
	 * to a specific node
	 * */
	std::multimap<LAddress::L3Type, WaveShortMessage*> mapsForBundles;

	/** Boolean to verify if the transmission will not fail*/
	bool canITransmit;

	/** Function for calculating P(A,B)*/
	void updateDeliveryPredsFor(const LAddress::L3Type BAdress);

	/** Function for calculating P(A,C) with P(A,B) & P(B,C)*/
	void updateTransitivePreds(const LAddress::L3Type BAdress,
			std::map<LAddress::L3Type, double> Bpreds);

	/** Function for aging P(A,B)*/
	void ageDeliveryPreds();

	/** Function for updating & exchanging probabilities*/
	void update(Prophet *prophetPkt);

//	/** Function to verify if the transmission will not fail*/
//	void canITransmit();

	void resumeConnection();

	void executeInitiatorRole(short  kind, Prophet *prophetPkt = NULL);

	void executeListenerRole(short  kind, Prophet *prophetPkt = NULL);

	void definingBundleOffer(Prophet *prophetPkt);

	bool existingBundle(WaveShortMessage *msg);

//	bool fwd_GRTRmax_sortingFunc(std::pair<LAddress::L3Type, double> firstPair, std::pair<LAddress::L3Type, double> secondPair);

	void storeBundle(WaveShortMessage *msg);

	/** @brief Handle messages from upper layer */
	virtual void handleUpperMsg(cMessage* msg);

	/** @brief Handle messages from lower layer */
	virtual void handleLowerMsg(cMessage* msg);

	/** @brief Handle self messages */
	virtual void handleSelfMsg(cMessage* msg);

	/** @brief Handle control messages from lower layer */
	virtual void handleLowerControl(cMessage* msg);
//
//	/** @brief Handle control messages from lower layer */
//	virtual void handleUpperControl(cMessage* msg);

	/** @brief decapsulate higher layer message from NetwPkt */
	virtual cMessage* decapsMsg(NetwPkt*);

	/** @brief Encapsulate higher layer packet into an NetwPkt*/
	virtual NetwPkt* encapsMsg(cPacket*);


//	virtual cObject *const setDownControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr);
//
//	virtual cObject *const setUpControlInfo(cMessage *const pMsg, const LAddress::L3Type& pSrcAddr);
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
		Bundle = 0xFF,
	};
	/**
	 * @brief Prophet Control Kinds used when notified by the lower layer (i.e Mac1609_4_Opp & NicEntryDebug)
	 */
	enum prophetNetwControlKinds {
		NEWLY_CONNECTED = LAST_BASE_NETW_CONTROL_KIND,
		NEW_NEIGHBOR = NEWLY_CONNECTED + 10,
		NO_NEIGHBOR_AND_DISCONNECTED = NEWLY_CONNECTED + 20,
	};
	virtual void initialize(int stage);
	virtual void finish();
	virtual ~ProphetV2();
};

#endif /* PROPHETV2_H_ */
