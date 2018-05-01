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

#include "CustStorageHelper.h"

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

	int Fwd_No;
	int Fwd_Yes_METD;
	int Fwd_Yes_Dist;
	int Fwd_Yes_Both;

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

	enum ForwardStratEnum{
		AtLeastOne = 0,
		Both = 1
	};
	int multiMetricFwdStrat;

	bool withAddressedAck;

	// CBH: CheckBeforeHello
	bool withCBH;

	bool withExplicitE2EAck;

	bool withCustodyList;

	enum CustodyModeEnum {
		No,
		Yes_WithoutACK,
		Yes_WithACK,
	};
	int custodyMode;

	enum CustodyListEnum {
		No_Diffuse = 0,
		Diffuse = 1,
		Diffuse_Delete = 2
	};
	int custodyList;

	CustStorageHelper cusModule;

    /** Size of the Custody Storage structure */
    unsigned int custodyStructureSize;

	bool withTTLForCus;

	double ttlForCus;

	double majorationOfTTLForCus;

    TTLForCtrlType typeTTLForCus;

	simsignal_t t_custodyLifeTime;

	cOutVector nbrStoredCustodyVector;

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

  	virtual void sendingHelloMsg();

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);
  	
  	void sendingInitMsg( LAddress::L3Type nodeAddr);

  	void handleInitMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleMsgToVPA(LAddress::L3Type vpaAddr);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmFinalDeliverd);

  	void sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDeliverd, bool custodyTransfer);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

////////////////////////// Others Methods //////////////////////

  	virtual bool store1AckSerial(unsigned long  serial, double expireTime);
  	
  	std::set<unsigned long> buildCustodySerialWithTimeStamp();

  	bool checkBeforeHelloMechanism();

  	std::vector<std::pair<WaveShortMessage*,int> > specific_scheduleFilterBundles(std::vector<std::pair<WaveShortMessage*,int> > unsortedWSMPair, LAddress::L3Type destAddr, int destType);

  	GeoTraCIMobility* getGeoTraci();
  	
	double getCurrentMETD();

	double getCurrentDist();

	bool makeForwardingDecision(double srcDist, double srcMETD);

	bool makeCustodyDecision(double srcDist);

	void emitSignalForCustodyLifeTime(unsigned long serial, double startTime, double endTime);

	void gen1CustodySerial(unsigned long serial, double currentMETD);

	void storeNCustodySerial(std::map<unsigned long ,double> ackSerialsWithTimestamp);

	void updateNCustodySerial();

	virtual bool store1CustodySerial(unsigned long  serial, double expireTime, bool shouldDelete);

  	virtual void initCustodyManagementOptions();

  	virtual void updateStoredCustodysForSession(LAddress::L3Type srcAddr, std::map<unsigned long, double > custodysToStore);

  	virtual std::map<unsigned long ,double> getUnStoredCustodysForSession(LAddress::L3Type srcAddr, std::map<unsigned long ,double> custodysToFilter);

  	virtual long estimateInBitsCtrlSize(bool isHelloCtrl, std::set<unsigned long>* SB_Ctrl, std::map<unsigned long, double >* SA_Ctrl, std::map<unsigned long, double >* CL_Ctrl, std::set<unsigned long>* RCC_Ctrl);
};

#endif
