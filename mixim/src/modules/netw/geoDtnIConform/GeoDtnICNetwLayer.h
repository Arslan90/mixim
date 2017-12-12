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

	int Fwd_No;
	int Fwd_Yes_METD;
	int Fwd_Yes_Dist;
	int Fwd_Yes_Both;

	std::set<unsigned long> custodySerial;

	std::set<unsigned long > missedOpportunities;

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

	enum ForwardStratEnum{
		AtLeastOne = 0,
		Both = 1
	};
	int multiMetricFwdStrat;

	bool withAddressedAck;

	// CBH: CheckBeforeHello
	bool withCBH;

	simsignal_t inRadioWithVPA;

	bool withExplicitE2EAck;

	int custodyList;

	std::set<unsigned long> custodySerialDeleted;
	std::multimap<double, unsigned long> custodySerialTimeStamp;

	enum CustodyListEnum {
		No_Diffuse = 0,
		Diffuse = 1,
		Diffuse_Delete = 2
	};

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

  	void sendingHelloMsg();

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);

//  	void sendingBundleMsg(GeoDtnNetwPkt *netwPkt, std::pair<LAddress::L3Type, double> FwdDist, std::pair<LAddress::L3Type, double> FwdMETD);

//  	void sendingBundleMsg();

  	void newSendingBundleMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleMsgToVPA(LAddress::L3Type vpaAddr);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDelivred, std::set<unsigned long> wsmFinalDeliverd);

  	void sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmFinalDeliverd);

  	void sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDeliverd, bool custodyTransfer);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

////////////////////////// Others Methods //////////////////////

  	void recordStatsFwds(std::pair<LAddress::L3Type, double> fwdDist, std::pair<LAddress::L3Type, double> fwdMETD);

  	virtual void storeAckSerial(unsigned long serial);

  	void storeCustodySerial(unsigned long serial);

  	void storeCustodySerials(std::set<unsigned long> setOfSerials);
  	
  	void updateNeighborhoodTable(LAddress::L3Type neighboor, NetwRoute neighboorEntry);

  	void emitInRadioWithVPA(LAddress::L3Type neighbor, int neighborNodeType, int flagValue);

  	void updateInRadioWithVPA(short kind, int neighborNodeType);
  	  	
  	bool checkBeforeHelloMechanism();

  	std::vector<std::pair<WaveShortMessage*,int> > specific_scheduleFilterBundles(std::vector<std::pair<WaveShortMessage*,int> > unsortedWSMPair, LAddress::L3Type destAddr, int destType);

  	GeoTraCIMobility* getGeoTraci();
  	
	double getCurrentMETD();

	double getCurrentDist();

	bool makeForwardingDecision(double srcDist, double srcMETD);

	bool makeCustodyDecision(double srcDist);

  	virtual void storeAckSerials(std::set<unsigned long > setOfSerials);

  	virtual void deletedCustodySerials();
};

#endif
