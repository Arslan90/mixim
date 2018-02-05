//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef LEG_VPApOpp_H_
#define LEG_VPApOpp_H_

#include "BaseModule.h"
//#include <BaseWaveApplLayer.h>
#include <NetwPkt_m.h>
#include "DtnApplLayer.h"

#ifndef DBG
#define DBG EV
#endif
//#define DBG std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath()

class LEG_VPApOpp  : 	public DtnApplLayer{
	public:

		virtual ~LEG_VPApOpp();

		virtual void initialize(int stage);

		virtual void receiveSignal(cComponent *source, simsignal_t signalID, const char *s);

    /**ARTURO  @brief Message kinds used by this layer */
    enum MyTestApplMessageKinds{ SEND_BROADCAST_TIMER = LAST_BASE_APPL_MESSAGE_KIND, //internal timer
    BROADCAST_MESSAGE = 10, //simple broadcast
    BROADCAST_REPLY_MESSAGE = 20, //simple reply
    BROADCAST_VPA_WMS = 30, //identified VPA beacon
    BROADCAST_VEH_WMS = 40, //identified Vehicular beacon
    DO_THINGS_EVERY_SECOND = 50, //internal timer for vehicular stuffs
    LAST_TEST_APPL_MESSAGE_KIND = 60, //I do not..
    DTN_TEST_MODE = 70, // Added by Arslan HAMZA CHERIF
    UPDATE = 80};
    // Added by Arslan HAMZA CHERIF


	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);

	protected:
		uint32_t receivedBeacons;
		uint32_t receivedData;
		  int messageSequence;//sequence of the VPA message number sent
		  double CW; //Contention Window value.
		  double T; //Timer to send periodic VPA WMS Broadcast. NOTE:This value must be de the same for VPA/vehicles beacons.
		  /*
		   * bool variable for enabling dtnTestMode
		   */
//		  bool dtnTestMode;
		  /*
		   * bool variable for enabling silentMode for VPA, VPA are only receiving messages
		   */
//		  bool silentMode;
//		  int nbrBundleSent;
//		  int nbrBundleReceived;
//		  int nbrUniqueBundleReceived;

//		  int nbrUniqueForAxe1;
//		  int nbrUniqueForAxe2;
//		  int nbrUniqueForAxe3;
//		  int nbrUniqueForAxe4;
//		  int nbrUniqueForAxe5;
//		  int nbrUniqueForAxe6;

//		  bool anyVPA;
//
//		  double avgDelay;
//		  double totalDelay;

		  cOutVector delays;
//		  cDoubleHistogram delayStats;

//		cLongHistogram hopCountStats;
		cOutVector hopCountVector;

//		std::map<unsigned long ,WaveShortMessage*> receivedBundles;

		std::set<int> vehiclesAddr;
		int updateSectorCycle;
		cMessage* update;
		cOutVector vehicleDensity;
		int currentVehDensity;

		cOutVector vehicleInContact;
		int currentVehInContact;

		int vehPassedBy;
		int vehHasBndls;
		int vehInRadio;
		int vehInRadioHasBndls;
		int vehNotSentBndl;
		int vehSent0Bndl;
		int vehSentFewBndl;
		int vehSentAllBndl;
		int vehMeanBndlSent;
		int nbrMeanBndlSent;
		int vehMeanBndlSuccSent;
		int nbrMeanBndlSuccSent;

		cOutVector vVehPassedBy;
		cOutVector vVehInRadio;
		cOutVector vVehHasBndls;
		cOutVector vVehInRadioHasBndls;
		cOutVector vVehNotSentBndl;
		cOutVector vVehSent0Bndl;
		cOutVector vVehSentFewBndl;
		cOutVector vVehSentAllBndl;
		cOutVector vVehMeanBndlSent;
		cOutVector vVehMeanBndlSuccSent;

		long totalMeanNeighborAllSent;
		long nbrMeanNeighborAllSent;
		long totalMeanNeighborNotSent;
		long nbrMeanNeighborNotSent;
		cOutVector vVehMeanNeighborsAllSent;
		cOutVector vVehMeanNeighborsNotSent;

		long totalMeanNeighbor0Sent;
		long nbrMeanNeighbor0Sent;
		long totalMeanNeighborFewSent;
		long nbrMeanNeighborFewSent;
		cOutVector vVehMeanNeighbors0Sent;
		cOutVector vVehMeanNeighborsFewSent;

		double totalMeanVPAContactAllSent;
		double nbrMeanVPAContactAllSent;
		double totalMeanVPAContactNotSent;
		double nbrMeanVPAContactNotSent;
		cOutVector vVehMeanVPAContactsAllSent;
		cOutVector vVehMeanVPAContactsNotSent;

		double totalMeanVPAContact0Sent;
		double nbrMeanVPAContact0Sent;
		double totalMeanVPAContactFewSent;
		double nbrMeanVPAContactFewSent;
		cOutVector vVehMeanVPAContacts0Sent;
		cOutVector vVehMeanVPAContactsFewSent;

		double totalMeanVPADistanceAllSent;
		double nbrMeanVPADistanceAllSent;
		double totalMeanVPADistanceNotSent;
		double nbrMeanVPADistanceNotSent;
		cOutVector vVehMeanVPADistancesAllSent;
		cOutVector vVehMeanVPADistancesNotSent;

		double totalMeanVPADistance0Sent;
		double nbrMeanVPADistance0Sent;
		double totalMeanVPADistanceFewSent;
		double nbrMeanVPADistanceFewSent;
		cOutVector vVehMeanVPADistances0Sent;
		cOutVector vVehMeanVPADistancesFewSent;

		////////////////////Custody Section ////////////////
		int vehCustody;
		cOutVector vVehCustody;

		int vehCusInRadio;
		int vehCusInRadioHasBndls;
		int vehCusNotSentBndl;
		int vehCusSent0Bndl;
		int vehCusSentFewBndl;
		int vehCusSentAllBndl;

		cOutVector vVehCusInRadio;
		cOutVector vVehCusInRadioHasBndls;
		cOutVector vVehCusNotSentBndl;
		cOutVector vVehCusSent0Bndl;
		cOutVector vVehCusSentFewBndl;
		cOutVector vVehCusSentAllBndl;

		long totalMeanCusNeighborAllSent;
		long nbrMeanCusNeighborAllSent;
		long totalMeanCusNeighborNotSent;
		long nbrMeanCusNeighborNotSent;
		cOutVector vVehMeanCusNeighborsAllSent;
		cOutVector vVehMeanCusNeighborsNotSent;

		long totalMeanCusNeighbor0Sent;
		long nbrMeanCusNeighbor0Sent;
		long totalMeanCusNeighborFewSent;
		long nbrMeanCusNeighborFewSent;
		cOutVector vVehMeanCusNeighbors0Sent;
		cOutVector vVehMeanCusNeighborsFewSent;

		double totalMeanCusVPAContactAllSent;
		double nbrMeanCusVPAContactAllSent;
		double totalMeanCusVPAContactNotSent;
		double nbrMeanCusVPAContactNotSent;
		cOutVector vVehMeanCusVPAContactsAllSent;
		cOutVector vVehMeanCusVPAContactsNotSent;

		double totalMeanCusVPAContact0Sent;
		double nbrMeanCusVPAContact0Sent;
		double totalMeanCusVPAContactFewSent;
		double nbrMeanCusVPAContactFewSent;
		cOutVector vVehMeanCusVPAContacts0Sent;
		cOutVector vVehMeanCusVPAContactsFewSent;

		double totalMeanCusVPADistanceAllSent;
		double nbrMeanCusVPADistanceAllSent;
		double totalMeanCusVPADistanceNotSent;
		double nbrMeanCusVPADistanceNotSent;
		cOutVector vVehMeanCusVPADistancesAllSent;
		cOutVector vVehMeanCusVPADistancesNotSent;

		double totalMeanCusVPADistance0Sent;
		double nbrMeanCusVPADistance0Sent;
		double totalMeanCusVPADistanceFewSent;
		double nbrMeanCusVPADistanceFewSent;
		cOutVector vVehMeanCusVPADistances0Sent;
		cOutVector vVehMeanCusVPADistancesFewSent;

		long countRcvH;
		long countRcvB;
		long countRcvA;

		long totalRcvH;
		long totalRcvB;
		long totalRcvA;

		cOutVector vMeanRcvH;
		cOutVector vMeanRcvB;
		cOutVector vMeanRcvA;

		long total0RcvH;
		long total0RcvB;
		long total0RcvA;
		cOutVector vMean0RcvH;
		cOutVector vMean0RcvB;
		cOutVector vMean0RcvA;
		long total1RcvH;
		long total1RcvB;
		long total1RcvA;
		cOutVector vMean1RcvH;
		cOutVector vMean1RcvB;
		cOutVector vMean1RcvA;
		long total2RcvH;
		long total2RcvB;
		long total2RcvA;
		cOutVector vMean2RcvH;
		cOutVector vMean2RcvB;
		cOutVector vMean2RcvA;
		long total3RcvH;
		long total3RcvB;
		long total3RcvA;
		cOutVector vMean3RcvH;
		cOutVector vMean3RcvB;
		cOutVector vMean3RcvA;
		long L20Sent;
		long L20Dropped;
		long L20Received;
		long L20Lost;
		long L20BackOff;
	  double L20Duration;
		cOutVector vL20Sent;
        cOutVector vL20Dropped;
        cOutVector vL20Received;
        cOutVector vL20Lost;
        cOutVector vL20BackOff;
        cOutVector vL20Duration;
		long L21Sent;
		long L21Dropped;
		long L21Received;
		long L21Lost;
		long L21BackOff;
	  double L21Duration;
		cOutVector vL21Sent;
        cOutVector vL21Dropped;
        cOutVector vL21Received;
        cOutVector vL21Lost;
        cOutVector vL21BackOff;
        cOutVector vL21Duration;
		long L22Sent;
		long L22Dropped;
		long L22Received;
		long L22Lost;
		long L22BackOff;
	  double L22Duration;
		cOutVector vL22Sent;
        cOutVector vL22Dropped;
        cOutVector vL22Received;
        cOutVector vL22Lost;
        cOutVector vL22BackOff;
        cOutVector vL22Duration;
		long L23Sent;
		long L23Dropped;
		long L23Received;
		long L23Lost;
		long L23BackOff;
	  double L23Duration;
		cOutVector vL23Sent;
        cOutVector vL23Dropped;
        cOutVector vL23Received;
        cOutVector vL23Lost;
        cOutVector vL23BackOff;
        cOutVector vL23Duration;

		  void sendVPApBroadcast(int messageSequenced);//send numerated broadcast.
		  void handleLowerMsg(cMessage* msg);
		  void handleSelfMsg(cMessage* msg);
		  //Adding my own prepareWSM messages.
		  virtual WaveShortMessage* prepareWSM(std::string name, int dataLengthBits, t_channel channel, int priority, int rcvId, unsigned long serial=0);

		  /*
		   * Return true if
		   */
		  bool bundleExistUnderOtherVPA(unsigned long serial);

		  virtual void finish();

		  void sendDtnMessage();
		  int randomVPADestAddr();
		  void resetL20Stats();
		  void updateL20Stats(long rcvH, long rcvB, long rcvA, long L2S, long L2D, long L2R, long L2L, long L2Back, long L2BackDur);
		  void resetL21Stats();
		  void updateL21Stats(long rcvH, long rcvB, long rcvA, long L2S, long L2D, long L2R, long L2L, long L2Back, long L2BackDur);
		  void resetL22Stats();
		  void updateL22Stats(long rcvH, long rcvB, long rcvA, long L2S, long L2D, long L2R, long L2L, long L2Back, long L2BackDur);
		  void resetL23Stats();
		  void updateL23Stats(long rcvH, long rcvB, long rcvA, long L2S, long L2D, long L2R, long L2L, long L2Back, long L2BackDur);
		  void recordL2Stats();
};

#endif /* VPApOpp_H_ */
