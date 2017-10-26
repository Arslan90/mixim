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

#ifndef __MIXIM_GEODTNICNETWLAYER_H_
#define __MIXIM_GEODTNICNETWLAYER_H_

#include "DtnNetwLayer.h"
#include "GeoDtnNetwPkt_m.h"
#include "GeoTraCIMobility.h"

/**
 * TODO - Generated class
 */
class GeoDtnICNetwLayer : public DtnNetwLayer
{
/*******************************************************************
** 							Variables section
********************************************************************/
  protected:
	GeoTraCIMobility* geoTraci;

	cOutVector bndlInterestVec;
	cOutVector missedOpprVec;

	int nbr2Fwds;
	int nbr1Fwds;
	int nbr0ValidFwds;
	int nbr1ValidFwds;

	int gDistFwd;
	int bDistFwd;

	int gMETDFwd;
	int bMETDFwd;

	int totalBundlesReceived;

	std::set<unsigned long> custodyAckSerial;

	std::set<unsigned long > missedOpportunities;

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

  	void updateNeighborhoodTable(LAddress::L3Type neighboor, NetwRoute neighboorEntry);

  	std::pair<LAddress::L3Type, double> getBestFwdMETD();

  	std::pair<LAddress::L3Type, double> getBestFwdDist();

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingHelloMsg(double distance, double METD, Coord currentPos);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

//  	void sendingBundleMsg(GeoDtnNetwPkt *netwPkt, std::pair<LAddress::L3Type, double> FwdDist, std::pair<LAddress::L3Type, double> FwdMETD);

  	void sendingBundleMsg();

  	void sendingBundleMsgToVPA(LAddress::L3Type vpaAddr);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDelivred, std::set<unsigned long> wsmFinalDeliverd);

  	void recordStatsFwds(std::pair<LAddress::L3Type, double> fwdDist, std::pair<LAddress::L3Type, double> fwdMETD);

  	std::vector<WaveShortMessage*> bundleForNode(LAddress::L3Type node);

  	std::vector<std::pair<WaveShortMessage*, int> > bundleFor1Fwd(LAddress::L3Type fwd);

  	virtual void storeAckSerial(unsigned long serial);

  	void storeCustodyAckSerial(unsigned long serial);

  	void storeCustodyAckSerials(std::set<unsigned long> setOfSerials);

  	bool erase(unsigned long serial);
  	
  	bool checkBeforeHelloMechanism();

  	GeoTraCIMobility* getGeoTraci();

  	void emitInRadioWithVPA(LAddress::L3Type neighbor, int neighborNodeType, int flagValue);

  	void updateInRadioWithVPA(short kind, int neighborNodeType);
};

#endif
