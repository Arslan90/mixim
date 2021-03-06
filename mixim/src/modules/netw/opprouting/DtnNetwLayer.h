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

#ifndef __MIXIM_DTNNETWLAYER_H_
#define __MIXIM_DTNNETWLAYER_H_

#include <omnetpp.h>
#include "cmodule.h"
#include "cmessage.h"
#include <cassert>

#include <map>
#include <set>
#include <iterator>
#include <list>
#include "fstream"

#include "TraCIMobility.h"
#include "BaseNetwLayer.h"
#include "NetwControlInfo.h"

#include "WaveShortMessage_m.h"
#include "DtnNetwPkt_m.h"
#include "Mac80211Pkt_m.h"

#include "BundleMeta.h"
#include "SimpleContactStats.h"
#include "ClassifiedContactStats.h"
#include "NetwSession.h"
#include "NetwRoute.h"
#include "BndlStorageHelper.h"
#include "AckStorageHelper.h"



const double maxDbl = std::numeric_limits<double>::max();

/**
 * TODO - Generated class
 */

typedef std::map<unsigned long,WaveShortMessage*> innerIndexMap;
typedef std::map<unsigned long,WaveShortMessage*>::iterator innerIndexIterator;

typedef std::map<LAddress::L3Type, innerIndexMap>::iterator bundlesIndexIterator;

typedef std::map<LAddress::L3Type, std::list<unsigned long> >::iterator iteratorContactID;
typedef std::map<unsigned long, SimpleContactStats>::iterator iteratorContactStats;

class DtnNetwLayer : public BaseNetwLayer {
	/*******************************************************************
	** 							Variables section
	********************************************************************/
  protected:
	/*******************************************************************
	** 							Neighborhood Mechanism variables
	********************************************************************/
	double heartBeatMsgPeriod;
	double netwRouteExpirency;
	double netwRoutePending;
	cMessage* heartBeatMsg;
	std::map<LAddress::L3Type, NetwRoute> neighborhoodTable;
	std::map<LAddress::L3Type, NetwSession> neighborhoodSession;

	bool recomputeMyNetwRoute;
	NetwRoute myNetwRoute;

	int NBHTableNbrInsert;
	int NBHTableNbrDelete;
	int NBHAddressNbrInsert;
	int NBHAddressNbrDelete;

	/*******************************************************************
	** 							Main  variables
	********************************************************************/
	int maxSimulationTime;

	TraCIMobility* traci;

	enum TTLForCtrlType {
		Fixed_TTL = 0,
		Adaptative_TTL = 1,
	};

	enum NodeType {
		VPA = 0x0A,
		Veh = 0x01,
	};

	NodeType nodeType;

	int sectorId;

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

	/**
	 * @brief Prophet Control Kinds used when notified by the lower layer (i.e Mac1609_4_Opp & NicEntryDebug)
	 */
	enum DtnNetwControlKinds {
		NEWLY_CONNECTED = LAST_BASE_NETW_CONTROL_KIND,
		NEW_NEIGHBOR = NEWLY_CONNECTED + 10,
		NO_NEIGHBOR_AND_DISCONNECTED = NEWLY_CONNECTED + 20,
		NEW_NEIGHBOR_GONE = NEWLY_CONNECTED + 30,
		RESTART = NEWLY_CONNECTED + 40,
		FORCED_RESTART = NEWLY_CONNECTED + 50
	};

	/**
	 * @brief The same as used by Prophetv2 in order to stay consistent with it
	 */
	enum DtnNetwMsgKinds {
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
	 * @brief The same as used by Prophetv2 in order to stay consistent with it
	 */
	enum DtnNetwSchedulingPolicy {
		RCAscRLDesc = 0,
		RCAscRLAsc,
		RCDescRLDesc,
		RCDescRLAsc,
		RLAsc,
		RLDesc,
	};

	DtnNetwSchedulingPolicy scheduleStrategy;

	/**
	 * @brief Various control for re-addressing of bundles after sector changes
	 */
	bool withCtrlForSectorReAddr;
	LAddress::L3Type oldSectorAddr;
	LAddress::L3Type newSectorAddr;

	/************************* Related to Bundle Storage ***********/
    bool withTTL;
    double ttl;

    /** Size of the WMS Storage structure */
    unsigned int bundlesStructureSize;

	BndlStorageHelper bndlModule;
	/************************* Related to Bundle Storage ***********/

	/************************* Related to ACKs ***********/
	bool withTTLForAck;

	double ttlForAck;

	TTLForCtrlType typeTTLForAck;

	/**
  	 * Boolean for the activation of PRoPHET ACK mecanism
  	 */
    bool withAck;

    /**
  	 * Size of acks structure
  	 */
    unsigned int ackStructureSize;

    AckStorageHelper ackModule;

	/************************* Related to ACKs ***********/

	/*******************************************************************
	** 					Variables used to compute main statistics
	********************************************************************/
    long nbrL3Sent;
    long nbrL3Received;

	int bundlesReceived;

    bool firstSentToVPA;
	int totalBundlesReceived;
	int bndlSentToVPA;
	int totalBndlSentToVPA;

    simsignal_t receiveL3SignalId;
    simsignal_t sentL3SignalId;
    simsignal_t sentBitsLengthSignalId;
    simsignal_t helloCtrlBitsLengthId;
    simsignal_t otherCtrlBitsLengthId;

	simsignal_t t_ackLifeTime;

	cMessage* updateMsg;

	double updateInterval;

    /*******************************************************************
	** 					Variables used to compute other statistics
	********************************************************************/
	bool recordContactStats;

    double sumOfContactDur;

	int nbrContacts;

	cOutVector contactDurVector;

	double sumOfInterContactDur;

	int nbrRecontacts;

	cOutVector intercontactDurVector;

	cOutVector nbrStoredBundleVector;

	cOutVector nbrStoredAcksVector;

	std::map<LAddress::L3Type, double> contacts;

	std::map<LAddress::L3Type, double> endContactTime;

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

    /*******************************************************************
	** 					Scheduling strategies
	********************************************************************/

	/**
	 * Comparator used to sort Bundles to sent when using RC Asc strategy
	 */
    std::vector<std::pair<WaveShortMessage*, int> > compAsFn_schedulingStrategy(std::vector<std::pair<WaveShortMessage*, int> > vectorToSort);

    static bool func_RCAscRLDesc(std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
	{
	  if (i.second != j.second){
		  return (i.second<j.second);
	  }else {
		  return (func_RLDesc(i,j));
//		  return (i.first->getTimestamp()>j.first->getTimestamp());
	  }
	}

    static bool func_RCAscRLAsc(std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
	{
	  if (i.second != j.second){
		  return (i.second<j.second);
	  }else {
		  return (func_RLAsc(i,j));
	  }
	}

    static bool func_RCDescRLDesc(std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
	{
	  if (i.second != j.second){
		  return (i.second>j.second);
	  }else {
		  return (func_RLDesc(i,j));
	  }
	}

    static bool func_RCDescRLAsc(std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
	{
	  if (i.second != j.second){
		  return (i.second>j.second);
	  }else {
		  return (func_RLAsc(i,j));
	  }
	}

    static bool func_RLAsc(std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
	{
    	simtime_t first_TTL = i.first->getTimestamp();
		simtime_t second_TTL = j.first->getTimestamp();
		return ( first_TTL < second_TTL );
	}

    static bool func_RLDesc(std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
	{
    	simtime_t first_TTL = i.first->getTimestamp();
		simtime_t second_TTL = j.first->getTimestamp();
		return ( first_TTL > second_TTL );
	}

    /*******************************************************************
	** 							Methods section
	********************************************************************/
public:
    virtual void initialize(int stage);
    virtual void finish();

  protected:
    virtual void gen1AckSerial(WaveShortMessage *msg);

    virtual bool store1AckSerial(unsigned long serial, double expireTime = maxDbl);

    virtual void storeNAckSerial(std::map<unsigned long, double > ackSerialsWithTimestamp);

  	/*
  	 * Convert a string to L3Address
  	 */
  	virtual LAddress::L3Type getAddressFromName(const char * name);

  	/*
  	 * Convert a string to time
  	 */
  	virtual double getTimeFromName(const char * name);

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

  	/** @brief Generic method to prepareNetwPkt */
  	virtual void prepareNetwPkt(DtnNetwPkt* myNetwPkt, short  kind, LAddress::L3Type destAddr);

  	/*
  	 * Function to decide if we have to restart IEP
  	 */
//  	virtual bool haveToRestartIEP(simtime_t t);

  	/*
  	 * generate contact serial
  	 */
  	virtual unsigned long generateContactSerial( int myAddr, int seqNumber, int otherAddr);

  	/*
  	 * start recording
  	 */
  	virtual unsigned long startRecordingContact(int addr, double time);

  	/*
  	 * end recording
  	 */
  	virtual unsigned long endRecordingContact(int addr, double time);

  	virtual unsigned long endRecordingContact(unsigned long contactID, bool hasForcedEnding);

  	virtual void updatingL3Sent(){
  		nbrL3Sent++;
  	}

  	virtual void updatingL3Received(){
  		nbrL3Received++;
  	}

  	virtual void recordBeginContactStats(LAddress::L3Type addr, double time);

  	virtual void recordEndContactStats(LAddress::L3Type addr, double time);

  	virtual void recordRecontactStats(LAddress::L3Type addr, double time);

//  	void updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind);

  	virtual void classify(SimpleContactStats* newContact);

  	virtual void classifyAll();

  	virtual void initAllClassifier();

  	virtual void recordClassifier(ClassifiedContactStats classifier);

  	virtual void recordAllClassifier();

  	virtual void recordAllScalars();

  	virtual void DefineNodeType();

  	virtual int getCurrentSector();

  	virtual Coord getCurrentPos();

  	virtual void updateNeighborhoodTable(LAddress::L3Type neighbor, NetwRoute neighborEntry);

  	virtual void updateStoredBndlForSession(LAddress::L3Type srcAddr, std::set<unsigned long > bundlesToStore);

  	virtual void updateStoredAcksForSession(LAddress::L3Type srcAddr, std::map<unsigned long, double > acksToStore);

  	void sendDown(cMessage* msg);

  	void sendDown(cMessage *msg, long HelloCtrlLength, long OtherCtrlLength, short nbrEncapData);

//  	virtual WaveShortMessage* getStoredWSMFromSerial(unsigned long serial);

  	virtual std::vector<WaveShortMessage* > scheduleFilterBundles(std::vector<std::pair<WaveShortMessage*,int> > unsortedWSMPair, LAddress::L3Type destAddr, int destType);

  	string lg2Str(long longToConvert)
  	{
  		std::stringstream ss;
  		ss << longToConvert;
  		std::string str = std::string(ss.str());
  		return str;
  	}

  	string dbl2Str(double doubleToConvert)
  	{
  		std::stringstream ss;
  		ss << doubleToConvert;
  		std::string str = std::string(ss.str());
  		return str;
  	}

  	void emitSignalForHelloCtrlMsg(long sizeHC_SB_Octets, long sizeHC_SA_Octets, long sizeHC_CL_Octets, long sizeHC_RCC_Octets);

  	void emitSignalForOtherCtrlMsg(long sizeOC_SB_Octets, long sizeOC_SA_Octets, long sizeOC_CL_Octets, long sizeOC_RCC_Octets);

  	void emitSignalForAckLifeTime(unsigned long serial, double startTime, double endTime);

  	virtual void initBndlManagementOptions();

  	virtual void initAckManagementOptions();

  	virtual void initEquipedVehicle();

  	virtual void initContactStats();

  	virtual void sendingHelloMsg();

  	virtual long estimateInBitsCtrlSize(bool isHelloCtrl, std::set<unsigned long>* SB_Ctrl, std::map<unsigned long, double >* SA_Ctrl, std::map<unsigned long, double >* CL_Ctrl, std::set<unsigned long>* RCC_Ctrl);

  public:
	/*
	 * Getter for isEquiped boolean
	 */
	bool isAnEquipedVehicle(){
		return isEquiped;
	}

	virtual int numInitStages() const {
		return 3;
	}
};

#endif
