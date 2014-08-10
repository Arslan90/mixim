//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//

#ifndef VEHICLE_H
#define VEHICLE_H

#include "BaseApplLayer.h"
#include "mobility/traci/TraCIMobility.h" //mobility module for hosts controlled by TraCIScenarioManager.


/**
 * Small IVC Demo
 */
class VEHICLE : public BaseApplLayer
{
	public:
		virtual void initialize(int);

	    /** @brief Message kinds used by this layer */
	    enum MyTestApplMessageKinds{
			SEND_BROADCAST_TIMER = LAST_BASE_APPL_MESSAGE_KIND, //internal timer
			BROADCAST_MESSAGE = 10,								//simple broadcast
			BROADCAST_REPLY_MESSAGE = 20,						//simple reply
			BROADCAST_VPA_WMS = 30,								//identified VPA beacon
			BROADCAST_VEH_WMS = 40,								//identified Vehicular beacon
			DO_THINGS_EVERY_SECOND = 50,						//internal timer for vehicular stuffs
			LAST_TEST_APPL_MESSAGE_KIND = 60					//I do not..
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
		int currentSectorTimer; //This is used to time the vehicle message reception and sectorId's.
		bool receivedSectorId; //This check in case I've received the VPAid of the sector.
	    bool appCW; //wheter I'll use or not the APP WC.
	    int appMinCW; //my own APPS CW
		double ST; //Adding the Mac80211 Slot Time (I can't call from the library, so I declare here).

		//double offsetY; //I'll subtract this to the Y position given for the traci function. Y axis is in inverse way!
		//double maxY; //Omnet and sumo inverse the Y value. I've to adjust resting subdstracting the Y value
		//double offsetX; //Omnet and sumo decalage the X value. I've to adjust adding the X value
	    int offsetX; // I've to adjust adding the X axis value
		int offsetY; // I've to adjust adding the Y axis value
		int maxY; //This is the Maximun Y axis value of the MAP.

		//AND My functions
		void inJunction(); //Check if vehicleID is or not in junction area.
		void splitMessagesReceived(cMessage*); //Split the string: const char* "VPAId+sequenceNumber"
		double CW; //Contention Window value.
		void vpaPosition(); //this contain The list of VPA positions.
		void whatSectorIm(); //Check in which sector is the vehicle located.
		void vehicleVideos(); //This is to generate videos, CAUTION it generates copious logs.
		void WMS(); //Gather the Weight Map Sector.



	protected:
		virtual void handleSelfMsg(cMessage*); //cela c'est pour le control de timers ou autre chose avec selfMessages
		virtual void handleLowerMsg(cMessage*); // cela recoi messages de basse couches
		void sendMessage();
		//virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj);

};

#endif
