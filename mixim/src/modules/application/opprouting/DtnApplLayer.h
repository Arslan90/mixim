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

#ifndef __MIXIM_DTNAPPLLAYER_H_
#define __MIXIM_DTNAPPLLAYER_H_

#include <omnetpp.h>

#include "BaseWaveApplLayer.h"


/**
 * TODO - Generated class
 */
class DtnApplLayer : public BaseWaveApplLayer	{
	public:

		virtual int numInitStages() const {
			return 3;
		}

		~DtnApplLayer();
		virtual void initialize(int stage);
		virtual void finish();

	protected:
		/**ARTURO/Arslan  @brief Message kinds used by Dtn Appl Layer */
		enum MyTestApplMessageKinds{
			SEND_BROADCAST_TIMER = LAST_BASE_APPL_MESSAGE_KIND, //internal timer
			BROADCAST_MESSAGE = 10,								//simple broadcast
			BROADCAST_REPLY_MESSAGE = 20,						//simple reply
			BROADCAST_VPA_WMS = 30,								//identified VPA beacon
			BROADCAST_VEH_WMS = 40,								//identified Vehicular beacon
			DO_THINGS_EVERY_SECOND = 50,						//internal timer for vehicular stuffs
			LAST_TEST_APPL_MESSAGE_KIND = 60,					//I do not..
			DTN_MSG_MODE = 70,									// Added by Arslan HAMZA CHERIF
		};

/***************** Scenario Model *****************/
		/**
		 * Scenario Model Type (Cologne -Koln- like) or Free
		 */
		enum t_scenarioType {
			Free	= 1,
			Sector	= 2,
		};

		/*
		 * Variable for specifying Scenario Model Type
		 */
		t_scenarioType scenarioModel;

		// Parameters related to Sector Mode
		int oldSector;
		int currentSector;
		int rowSectorGrid;
		int colSectorGrid;
		double sectorSizeX;
		double sectorSizeY;
		bool useNegativeValues; // Allow us the use of negative values for Offset - Coord(0.0) of sector[0]
		double sectorOffsetX;  // Traci Coord X for - Coord(0.0) of sector[0]
		double sectorOffsetY;  // Traci Coord Y for - Coord(0.0) of sector[0]

/***************** Data Traffic *****************/
		/*
		 * bool variable for enabling DtnMsg;
		 */
		bool withDtnMsg;

		/*
		 * the Dtn Msg that will be exchanged among App Layer
		 */
		cMessage *dtnMsg;

		/**
		 * Enumeration for Data Traffic Generator Mode (Dtn Msgs)
		 */
		enum t_data_generatorMode {
			ALL2ALL	= 1,
			ALL2VEH	= 2,
			ALL2VPA	= 3,
			VPA2ALL = 4,
			VPA2VEH = 5,
			VPA2VPA = 6,
			VEH2ALL	= 7,
			VEH2VEH	= 8,
			VEH2VPA	= 9,
		};

		/*
		 * Variable for specifying data traffic generation mode (Dtn Msgs)
		 */
		t_data_generatorMode genStrategy;

		/*
		 * Variable for storing data traffic generation mode (Dtn Msgs) as a string
		 */
		std::string strategy;

		/**
		 * Enumeration for Data Traffic Forwarder Mode (Mule)
		 */
		enum t_data_forwarderMode {
			ALL = 1,
			VPA = 2,
			VEH = 3,
		};

		/*
		 * Variable for specifying data traffic forwarding mode (Mule)
		 */
		t_data_forwarderMode fwdStrategy;

/***************** Sending/Receiving Strategies *****************/
		/**
		 * Enumeration for Data Traffic Sending type (SectorMode or PeriodicMode) - Typically used by Veh
		 */
		enum t_dtnMsgSendingType {
			SectorEntry 	 = 1, // At sector entrance (like in Cologne)
			Periodic = 2, // Generate periodically DtnMsg
		};

		// Parameters related to Periodic Mode
		double dtnMsgPeriod;
		double dtnMsgMaxTime;
		double dtnMsgMinTime;
		bool dtnMsgSynchronized;

		/*
		 * Variable for specifying Data Traffic Sending type
		 */
		t_dtnMsgSendingType sendingStrategy;

		/**
		 * Enumeration for Data Traffic Reception type (Unique vs Random vs Any) - Typically used by VPA
		 */
		enum t_dtnMsgReceptionType {
			Unique = 1, // Unique receipt for generated data traffic (have to specify the recipient) - By default VPA[0]
			Random = 2, // Choose randomly the recipient
			Any    = 3, // Recipient is the current VPA, which changes every time we switch to another sector
		};

		/*
		 * Variable for specifying Data Traffic Receiving type
		 */
		t_dtnMsgReceptionType receivingStrategy;

		// Different stats for Bundle Sending/Reception
		int nbrBundleSent;
		int nbrBundleReceived;
		int nbrUniqueBundleReceived;

		bool isEquiped;

//		/** @brief handle messages from below */
//		virtual void handleLowerMsg(cMessage* msg);
//		/** @brief handle self messages */
//		virtual void handleSelfMsg(cMessage* msg);
//
//		virtual WaveShortMessage* prepareWSM(std::string name, int dataLengthBits, t_channel channel, int priority, int rcvId, int serial=0);
//		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
//
//		virtual void handlePositionUpdate(cObject* obj);

		/*
		 *	Method to get the Data Source string from strategy
		 */
		virtual std::string getDataSrcFromStrategy(std::string currentStrategy);

		/*
		 *	Method to get the Data Destination string from strategy
		 */
		virtual std::string getDataDestFromStrategy(std::string currentStrategy);

};

#endif
