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

#ifndef __MIXIM_GEODTNNETWLAYER_H_
#define __MIXIM_GEODTNNETWLAYER_H_

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
class GeoDtnNetwLayer : public DtnNetwLayer
{
/*******************************************************************
** 							Variables section
********************************************************************/
  protected:
	GeoTraCIMobility* geoTraci;
	double heartBeatMsgPeriod;
	double netwRouteExpirency;
	double netwRoutePending;
	cMessage* heartBeatMsg;
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

	// E2E Acks serial
	std::set<unsigned long > ackSerial;

	std::set<unsigned long> custodyAckSerial;

	std::set<unsigned long > missedOpportunities;

	enum NodeType {
		VPA = 0x0A,
		Veh = 0x01,
	};

	NodeType nodeType;

	int sectorId;

	int bndlSentToVPA;

	bool firstSentToVPA;

	int totalBndlSentToVPA;

	/**
	 * Current Number of Insert operation in ACKSerial & Bundle storage
	 * (in order to allow us to check if something new have been added)
	 */
	int currentNbrIsrt;

	/**
	 * Last Number of Insert operation in ACKSerial & Bundle storage
	 * (in order to allow us to check if something new have been added)
	 */
	int lastNbrIsrt;

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

	bool withMETDFwd;
	bool withDistFwd;

	enum CustodyModeEnum {
		No,
		Yes_WithoutACK,
		Yes_WithACK,
	};
	int custodyMode;

	// CBH: CheckBeforeHello
	bool withCBH;

	simsignal_t inRadioWithVPA;

	/**
	 * Comparator used to sort Bundles to sent when using RC Asc strategy
	 */
//	bool comparatorRCAsc (std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
//	{
//	  if (i.second != j.second){
//		  return (i.second<j.second);
//	  }else {
//		  return (i.first->getTimestamp()>j.first->getTimestamp());
//	  }
//	}
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

  	/** Function for preparing GeoDtnNetwPkt */
  	GeoDtnNetwPkt* prepareNetwPkt(short kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, LAddress::L3Type vpaAddr);

  	/** Build KnownNeighbors set based on neighborhood table*/
  	std::set<LAddress::L3Type> getKnownNeighbors();

  	void updateNeighborhoodTable(LAddress::L3Type neighboor, NetwRoute neighboorEntry);

  	std::pair<LAddress::L3Type, double> getBestFwdMETD();

  	std::pair<LAddress::L3Type, double> getBestFwdDist();

  	void DefineNodeType();

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingHelloMsg(GeoDtnNetwPkt *netwPkt, double distance, double METD, Coord currentPos);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

//  	void sendingBundleMsg(GeoDtnNetwPkt *netwPkt, std::pair<LAddress::L3Type, double> FwdDist, std::pair<LAddress::L3Type, double> FwdMETD);

  	void sendingBundleMsg();

  	void sendingBundleMsgToVPA(LAddress::L3Type vpaAddr);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleAckMsg(GeoDtnNetwPkt *netwPkt, std::list<unsigned long> wsmDelivred, std::list<unsigned long> wsmFinalDeliverd);

  	void sendDown(cMessage* msg);

  	void recordStatsFwds(std::pair<LAddress::L3Type, double> fwdDist, std::pair<LAddress::L3Type, double> fwdMETD);

  	std::vector<WaveShortMessage*> bundleForVPA(LAddress::L3Type vpaAddr);

  	std::vector<WaveShortMessage*> bundleForNode(LAddress::L3Type node);

  	std::vector<WaveShortMessage*> bundleForFwds(LAddress::L3Type fwdDist, LAddress::L3Type fwdMETD);

  	std::vector<std::pair<WaveShortMessage*, int> > bundleFor1Fwd(LAddress::L3Type fwd);

  	bool existInNetwSession(WaveShortMessage* wsm);

  	void storeAckSerial(unsigned long serial);

  	void storeCustodyAckSerial(unsigned long serial);

  	void storeAckSerial(std::set<unsigned long> setOfSerials);

  	void storeCustodyAckSerial(std::set<unsigned long> setOfSerials);

  	bool erase(unsigned long serial);

	void updateStoredBndlForSession(LAddress::L3Type srcAddr, std::set<unsigned long> storedBundle);
};

#endif
