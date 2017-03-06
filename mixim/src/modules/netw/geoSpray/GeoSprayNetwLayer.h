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

#ifndef __MIXIM_GEOSPRAYNETWLAYER_H_
#define __MIXIM_GEOSPRAYNETWLAYER_H_

#include <omnetpp.h>
#include "DtnNetwLayer.h"
#include "GeoDtnNetwPkt_m.h"
#include "NetwSession.h"
#include "NetwRoute.h"
#include "GeoTraCIMobility.h"
#include "set"

/**
 * TODO - Generated class
 */
class GeoSprayNetwLayer : public DtnNetwLayer
{
/*******************************************************************
** 							Variables section
********************************************************************/
  protected:
	GeoTraCIMobility* geoTraci;
	std::map<LAddress::L3Type, NetwRoute> neighborhoodTable;
	std::map<LAddress::L3Type, NetwSession> neighborhoodSession;

	cOutVector bndlInterestVec;
	cOutVector missedOpprVec;

	int NBHTableNbrInsert;
	int NBHTableNbrDelete;
	int NBHAddressNbrInsert;
	int NBHAddressNbrDelete;

	int nbr2Fwds;
	int nbr1Fwds;
	int nbr0ValidFwds;
	int nbr1ValidFwds;

	int totalBundlesReceived;

	bool meetVPA;

	bool withSentTrack;

	// E2E Acks serial
	std::set<unsigned long > ackSerial;

	std::set<unsigned long > missedOpportunities;

	int sectorId;

	int bndlSentToVPA;

	bool firstSentToVPA;

	int totalBndlSentToVPA;

	int nbrReplica;

	/**
	 * @brief The same as used by Prophetv2 in order to stay consistent with it
	 */
	enum GeoDtnMsgKinds {
		HELLO = 0x00,
		ERROR = 0x01,
		RIBD  = 0xA0,
		RIB   = 0xA1,
		Bundle_Offer = 0xA4,
		Bundle_Response = 0xA5,
		Bundle_Ack = 0xD0,
		Bundle = 0xFF,
	};

	std::map<unsigned long, int> bundlesReplicaIndex;

	std::map<unsigned long, int> bundlesRmgReplica;

	std::multimap<unsigned long, LAddress::L3Type> bundlesReplicaPerAddr;

	/**
	 * Comparator used to sort Bundles to sent when using RC Asc strategy
	 */
	struct comparatorRCAsc {
		bool operator() (std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
		{
		  if (i.second != j.second){
			  return (i.second<j.second);
		  }else {
			  return (i.first->getTimestamp()>j.first->getTimestamp());
		  }

		}
	} comparatorRCAscObject;

	// comparison, not case sensitive.
	struct comparatorRLAsc {
		bool operator() (WaveShortMessage* first_TTL, WaveShortMessage* second_TTL)
		{
			simtime_t i = first_TTL->getTimestamp();
			simtime_t j = second_TTL->getTimestamp();
			return ( i <= j );
		}
	} comparatorRLAscObject;

	/*
	 * Bool withEMethod standing for erroneous method, in order to evaluate the impact of the previous error
	 */
	bool withEMethod;

	bool withExplicitE2EAck;

	bool withExplicitH2HAck;
/*******************************************************************
** 							Methods section
********************************************************************/
  public:
 	virtual int numInitStages() const {
    	return 3;
    }
    virtual void initialize(int stage);
    virtual void finish();

  protected:
  	/** @brief Handle messages from upper layer */
  	virtual void handleUpperMsg(cMessage* msg);

  	/** @brief Handle messages from lower layer */
  	virtual void handleLowerMsg(cMessage* msg);

  	/** @brief Handle self messages */
	virtual void handleSelfMsg(cMessage* msg);

  	/** @brief Handle control messages from lower layer */
  	virtual void handleLowerControl(cMessage* msg);
//
//  	/** @brief Handle control messages from lower layer */
//  	virtual void handleUpperControl(cMessage* msg);

  	void sendingHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleOfferMsg(LAddress::L3Type destAddr);

  	void handleBundleOfferMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleResponseMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmResponseBndl);

  	void handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleMsg(LAddress::L3Type destAddr, WaveShortMessage* wsm, int nbrReplica, bool custodyTransfer);

  	void sendingBundleMsgToVPA(LAddress::L3Type vpaAddr);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

	void sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::list<unsigned long> wsmFinalDeliverd);

	void sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::list<unsigned long> wsmDeliverd, int nbrReplica, bool custodyTransfer);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

//  	std::pair<LAddress::L3Type, double> getBestFwdMETD();
//
//  	std::pair<LAddress::L3Type, double> getBestFwdDist();
//
//  	/** Build KnownNeighbors set based on neighborhood table*/
//  	std::set<LAddress::L3Type> getKnownNeighbors();
//
//  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);
//
////  	void sendingBundleMsg(GeoDtnNetwPkt *netwPkt, std::pair<LAddress::L3Type, double> FwdDist, std::pair<LAddress::L3Type, double> FwdMETD);
//
//  	void sendingBundleMsg();
//
//  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);
//
//  	void sendingBundleAckMsg(GeoDtnNetwPkt *netwPkt, std::list<unsigned long> wsmDelivred, std::list<unsigned long> wsmFinalDeliverd);
//
//  	void sendDown(cMessage* msg);
//
//  	void recordStatsFwds(std::pair<LAddress::L3Type, double> fwdDist, std::pair<LAddress::L3Type, double> fwdMETD);
//
//  	std::vector<WaveShortMessage*> bundleForVPA(LAddress::L3Type vpaAddr);
//
//  	std::vector<WaveShortMessage*> bundleForFwds(LAddress::L3Type fwdDist, LAddress::L3Type fwdMETD);
//
//  	std::vector<WaveShortMessage*> bundleForNode(LAddress::L3Type node);
//
//  	bool existInNetwSession(WaveShortMessage* wsm);
//

//
//	void updateStoredBndlForSession(LAddress::L3Type srcAddr, std::set<unsigned long> storedBundle);

	////////////////////////// Others Methods //////////////////////

  	/** Function for preparing GeoDtnNetwPkt */
  	GeoDtnNetwPkt* prepareNetwPkt(short kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, LAddress::L3Type vpaAddr);

  	void updateNeighborhoodTable(LAddress::L3Type neighboor, NetwRoute neighboorEntry);

  	void storeAckSerial(unsigned long serial);

  	void storeAckSerial(std::set<unsigned long> setOfSerials);

  	bool erase(unsigned long serial);

  	bool exist(unsigned long serial);

  	GeoTraCIMobility* getGeoTraci();
};

#endif
