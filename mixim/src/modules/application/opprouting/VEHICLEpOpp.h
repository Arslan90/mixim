//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
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

#ifndef VEHICLEpOpp_H
#define VEHICLEpOpp_H
#define MYDEBUG EV //Just for printing Messages

#include "BaseWaveApplLayer.h"
#include "TraCIMobility.h"
#include "NetwControlInfo.h" //ARturo
#include "SimpleAddress.h" //ARturo
//#include "ApplPkt_m.h" //Arturo
#include <math.h> // for euclidian distance
#include <cstring> // for spliting strings
#include <NetwPkt_m.h>
#include "BaseNetwLayer.h"

/**
 * Small IVC Demo using 11p
 */
class VEHICLEpOpp : public BaseWaveApplLayer {
	public:

	 	virtual int numInitStages() const {
	    	return 3;
	    }

		virtual void initialize(int stage);
		//bool junctionRange; // True if vehicle position <=  epicenterRange
		void entra(int currentNumBackoffs, int currentCW, int myNicId); //REceive information from the MAC layer.
		//~VEHICLEp() { DBG << "logs, destruct," <<std::endl;	} //Arturo testing...

	    /**ARTURO  @brief Message kinds used by this layer */
	    enum MyTestApplMessageKinds{
			SEND_BROADCAST_TIMER = LAST_BASE_APPL_MESSAGE_KIND, //internal timer
			BROADCAST_MESSAGE = 10,								//simple broadcast
			BROADCAST_REPLY_MESSAGE = 20,						//simple reply
			BROADCAST_VPA_WMS = 30,								//identified VPA beacon
			BROADCAST_VEH_WMS = 40,								//identified Vehicular beacon
			DO_THINGS_EVERY_SECOND = 50,						//internal timer for vehicular stuffs
			LAST_TEST_APPL_MESSAGE_KIND = 60,					//I do not..
			DTN_TEST_MODE = 70									// Added by Arslan HAMZA CHERIF
	    };

	protected:
		static const simsignalwrap_t mobilityStateChangedSignal;

	protected:
		TraCIMobility* traci;


		//Arturo: mon information.
		//TraCIScenarioManager* traciManager; //voila voici mon poitteur pour attirer ce class.
		//bool sentMessage;
		cMessage *delayTimer; //timer to send periodically the vehicle messages.
		/* Note: je pars du principe que les voitures son muettes. Et les VPA/beacons ils parlent seulement
		 * en anoncant le WMS.  De  l'autre facon si les voitures font la demande on aurat beaucoup de TX.
		 * Un beacon c'est un voiture avec un copy WMS sans exipiration et qu'il peut envoyer à ses voisins. */
		double T; //Timer T to send periodic WMS by VPAs/Beacons. NOTE:This value must be de the same for VPA/vehicles beacons.
		bool modeDissemination; //True when mode is in dissemination, false if only in listening
		cMessage *everySecond; //timer to do vehicular things every seconds.
		double epicenterRange; // (meters) konstante in meters from the epicenter
		bool junctionRange; // True id vehicle position <= epicenterRange
		int messageSequence;//sequence of the VPA message number sent.
		int messageSequenceVPA;//ID number of the VPA sent.
		std::string junctionID; //JunctionId where I'm in this moment.
		double timeToSend; // Flag used to send one time the print logs of number of vehicles
		int counterRx; //This is a counter of messages reception (Rx).
		int neighbors; //this keep the value of the neighbors detected pending a determined period of time (e.g. 1sg)
		int currentSector; //This maintain the current sector where the vehicle is transit.
		int lastSector; //This maintain the last sector where the vehicle transited.
		int currentSectorTimer; //This is used to time the vehicle message reception and sectorId's.
		bool receivedSectorId; //This check in case I've received the VPAid of the sector.
	    bool appCW; //wheter I'll use or not the APP WC.
	    int appMinCW; //my own APPS CW

		int droppedMessages;//average Mac80211p dropped message. In order to figure out how to calculate my STmax.
		int counterDroppedMessages;//Total count Mac80211p dropped message. In order to figure out how to calculate my STmax.
		int currentCW; //Mac80211p currentCW
		int Tupdate; //periodic time to obtain the dropped packets.

	    int offsetX; // I've to adjust adding the X axis value
		int offsetY; // I've to adjust adding the Y axis value
		int maxY; //This is the Maximun Y axis value of the MAP.
		simtime_t vehTimeIn; //Time the vehicle enter into the simulation
		simtime_t vehTimeOut; //Time the vehicle get out of the simulation
		int vehRx; //Vehicle total Receptions.

		// Variables added by me
		/*
		 * bool variable for enabling dtnTestMode
		 */
		bool dtnTestMode;
		cMessage *dtnTestMsg;
		int dtnTestCycle;
		int dtnTestMaxTime;
		int nbrBundleSent;
		int nbrBundleReceived;
		bool dtnSynchronized;

		void sendDtnMessage();
		int vpaDestAddr();
		/*
		 * Warning: In this function, we considerate that the sectorID is the same as the vpaID, so Sector 0 => VPA 0
		 */
		int vpaDestAddr(int sectorID);

		void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj);

		//AND My functions
		void inJunction(); //Check if vehicleID is or not in junction area.
		//void splitMessagesReceived(std::string n); //Split the string: const char* "VPAId+sequenceNumber"
		double CW; //Contention Window value.
		void vpaPosition(); //this contain The list of VPA positions.
		void whatSectorIm(); //Check in which sector is the vehicle located.
		void vehicleVideos(); //This is to generate videos, CAUTION it generates copious logs.
		void WMS(); //Gather the Weight Map Sector.
		WaveShortMessage* prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial); //overrinding with my modified own function
		void readNicFiles();//Read the nic/files.txt in order to retrieve the node information and update internal messages.

		//OLDER AND TO REVIEW!!
		//simtime_t lastDroveAt;
		//bool sentMessage;
		virtual void handleSelfMsg(cMessage* msg);
		virtual void handleLowerMsg(cMessage* msg);
		//bool inJunction();
		//My variables
		//cMessage *delayTimer; //timer to send periodically the vehicle messages.
		//cMessage *everySecond; //timer to do vehicular things every seconds.
		//bool modeDissemination; //Flag to indicate when in junction areas.
		//int epicenterRange;// distance from the junction.
		//int T; //Time to broadcasting
		//virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj);
		//void handlePositionUpdate();
		virtual void finish(); //Create final statistics for every vehicle

		// My variables
		bool loopVehicle;
		std::string currentRouteId;
		std::list<std::string> currentRoute;
		bool reroutedToLoopRoute;
		std::string edgeForLooping;

		int netwAddr;
		int nbrMsgSent;
		bool isNetwAddrInit;

	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage();
		int generateUniqueSerial(const int netwAddr, const int nbrMsgSent);


};

#endif
