
/***********************************************************
 * My file VEHICLE.cc Based on the file TraCIDemo.cc
 *
 * OK , this is my own description: This file governs the Vehicle ACTIVITY.
 * This is the 2nd version of Vehicle.
 * I'll clean this version up and add the options of:
 * Vehicle traffic-probe
 * WMS exchange V2V, V2I.
 *
 * Last UPDATE: Fri Apr 20 19:03:50 CEST 2012
 *
 ***********************************************************/

#include "application/traci/VEHICLE.h"
#include "NetwControlInfo.h"
#include "SimpleAddress.h"
#include "ApplPkt_m.h" //Arturo
#include <math.h> // for euclidian distance
#include <cstring> // for spliting strings

#define MYDEBUG EV //cela c'est pour envoyer data vers OUPUT.

Define_Module(VEHICLE);

const simsignalwrap_t VEHICLE::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

//Voici c'est pour initilizer tous les entites qui demarrent pour la premier fois
void VEHICLE::initialize(int stage) {
	BaseApplLayer::initialize(stage);
	if (stage == 0) {
		//TRACI STUFF
		debug = par("debug");
		traci = TraCIMobilityAccess().get(getParentModule());
		findHost()->subscribe(mobilityStateChangedSignal, this);

		//T=5;//Set vehicle-Beacon periodic timer broadcast.
        //epicenterRange= 20; //Distance from the vehicle to the junction epicenter..
		T= hasPar("timeT") 	? par("timeT").doubleValue(): 5;
		epicenterRange= hasPar("epicenterValue") ? par("epicenterValue").doubleValue() : 20;
	    appCW = hasPar("appCW") ? par("appCW").boolValue(): false; //wheter I'll use or not the APP WC.
	    appMinCW = hasPar("appMinCW") ? par("appMinCW").doubleValue(): 100; //my APPS CW
		EV << "logs, Using value T=" << T <<" epicenter=" << epicenterRange<< ",appCW,"<< appCW <<",appMinCW,"<< appMinCW <<" in:"<< traci->getExternalId() <<endl;

		//offsetY= 41947; //I'll subtract this to the Y position given for the traci function. Y axis is in inverse way!
		//maxY= 200; //Add the Max value of Y of the scenario simulated.
		//maxY= 2100; //Add the Max value of Y of the scenario simulated.
		//offsetX= 100; //Omnet and sumo decalage the X value. I've to adjust adding the X value
		offsetX= hasPar("offsetX") ? par("offsetX").doubleValue() : 4000;
		offsetY= hasPar("offsetY") ? par("offsetY").doubleValue() : 5000;
		maxY= hasPar("maxY") ? par("maxY").doubleValue() : 41947;
		EV << "logs, offset values offsetX=" << offsetX <<" offsetY=" << offsetY<< ",maxY,"<< maxY <<endl;

        modeDissemination= false; //When creating the vehicle is in mode listening
        messageSequence= 0; //set to CERO the initial value
        junctionRange= false; //set false the junction range
		CW= 0; //Contention Window value.
		messageSequence= 0;//sequence of the VPA message number sent.
		messageSequenceVPA= 0;//ID number of the VPA sent.

		//junctionID= 0; //initialize. A value= 0 is vehicle not in any junction.
		counterRx=0; //This is a counter of the messages reception (Rx). Maybe I'll use to calculate the CW
		neighbors=0; //this keep the value of the neighbors detected pending a determined period of time (e.g. 1sg)
		currentSector= 0; //start in clean.
		currentSectorTimer= 0; //start in clean.
		receivedSectorId= false; //reset the Veh sectorId reception
		ST = 20E-6; //Adding the Mac80211 Slot Time (I can't call from the library, so I declare here).

        //The self message
		delayTimer = new cMessage( "delay-timer", SEND_BROADCAST_TIMER );
		everySecond = new cMessage( "delay-timer", DO_THINGS_EVERY_SECOND );
	}
	else if(stage==1) {
	        scheduleAt(simTime() + 1, delayTimer); //Scheduling the self message.
	        scheduleAt(simTime() + 1, everySecond); //Scheduling the self message.
	}
}


// SELF-MESSAGES RECEIVED.
void VEHICLE::handleSelfMsg(cMessage *msg) {
	EV << "logs, SELF-message RECEIVED in, "<< traci->getExternalId() <<" of:"<< msg->getKind() <<endl; // pour voir qui recoi le message..

    switch(msg->getKind())
    {
    case SEND_BROADCAST_TIMER:
    	if (modeDissemination and junctionRange) { //Vehicle send WMS beacon when in mode Dissemination & near enough to junction.
    		EV << "logs, VEH Sending PACKET. " << endl;
    		sendMessage();
    	}

        //re-Set  self-message.
        if (!appCW){  //NO APPS_CW
            EV << "logs, no apps CW " <<endl;
        	scheduleAt(simTime() + T, delayTimer);
        }
        else 		//using APPS_CW
        {
        	CW= intuniform(0, appMinCW)* ST; //computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW, "<< CW  <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
        }
        /*
    	//Slot times Delay
		CW= intuniform(0, 10)* 20E-6;
		EV << "logs, veh, " << myApplAddr() << ", using CW, "<< CW  <<endl;
        //delayTimer = new cMessage( "delay-timer", SEND_BROADCAST_TIMER );
        scheduleAt(simTime() + T + CW, delayTimer); //Adding APPS_CW
        */
     break;

    case DO_THINGS_EVERY_SECOND:
    	inJunction(); //Update if vehicleId is in junctionId area
    	whatSectorIm(); //Check in which sector is the vehicle
    	vehicleVideos(); //This is to generate videos, CAUTION it generates copious logs.
    	//TODO: Is missing how to agregate the WMS.Check the function for further information
    	WMS(); //Gather the Weight Map Sector.

    	if (modeDissemination){ EV << "logs, Im in modeDissemination "<< endl;} else {EV << "logs, Im not in modeDissemination" << endl;};
        //if (modeDissemination) { EV <<"logs, Flag, dissemination ON,"<< simTime() <<"," << traci->getExternalId() <<endl; }
    	if (junctionRange){
    		EV << "logs, Im in junction "<< endl;} else {EV << "logs, Im not in junction" << endl;
    		EV << "logs, neighbors:" << neighbors << ", reports,"<< traci->getExternalId() <<", at," << simTime() <<","<<endl;
        }

        neighbors= counterRx; //Count current neighbors with the value of RX events.
        counterRx= 0; //Reset Rx counter every N seconds.

    	//re-schedule self-message.
		//everySecond = new cMessage( "delay-timer", DO_THINGS_EVERY_SECOND ); //self_message
        scheduleAt(simTime() + 1, everySecond); //Scheduling the self message.
		break;

    default:
        EV <<"logs, Unkown selfmessage! -> delete, kind: "<<msg->getKind()<<endl;
        break;
    }
}



//PACKETS RECEIVED
void VEHICLE::handleLowerMsg(cMessage* msg) {
	ApplPkt *m = static_cast<ApplPkt *>(msg);
	int currentMessageSequence;
	int currentMessageSequenceVPA;

    switch(msg->getKind())
    {
    case BROADCAST_VPA_WMS: //When receiving VPA Broadcast
    	//here WE DO NOT HAVE Voiture dormant, 'cause VPA sources always are listened no matter what, when, where.
    	//I'll profit of this function to know if vehicles have received the SectorID.
    	splitMessagesReceived(msg);//Split the string: const char* "VPAId+sequenceNumber" & update vehicle's variables

/*********************IDENTIFYING  VEHICLE MESSAGE **************/
        //if I noticed a Reception of the current sectorID where the vehicleId transit.
    	// Check only once per sector, in case the receivedSectorId is false.
    	if (!receivedSectorId){
    		if (currentSector == messageSequenceVPA){
    			MYDEBUG <<"logs, vehicle sector,"<< traci->getExternalId() << ",sector," << currentSector <<",timer,"<< currentSectorTimer <<",receivedsectorId,"<< receivedSectorId <<",from VPA," << m->getSrcAddr() <<"," <<endl;
    			receivedSectorId= true; //update the Veh sectorId
    			currentSectorTimer= 0; //Reset timer
    		}
    	}
/*********************IDENTIFYING  VEHICLE MESSAGE **************/

    	modeDissemination = true;
        cancelEvent(delayTimer); //Delete anterior event and schedule a newer one.


        //re-Set  self-message.
        if (!appCW){  //NO APPS_CW
            EV << "logs, no apps CW " <<endl;
        	scheduleAt(simTime() + T, delayTimer);
        }
        else 		//using APPS_CW
        {
        	CW= intuniform(0, appMinCW)* ST; //computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW, "<< CW  <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
        }
        /*
        //re-Set  self-message.
        CW= intuniform(0, 10)* 20E-6;
		EV << "logs, veh, " << myApplAddr() << ", using CW, "<< CW  <<endl;
        //delayTimer = new cMessage( "delay-timer", SEND_BROADCAST_TIMER );
        scheduleAt(simTime() +  T + CW, delayTimer); // T  without CW delay
		*/

    	EV << "logs, VEHICLE RECEIVED: BROADCAST_VPA_WMS in: "<< traci->getExternalId() << ",kind: " << m->getKind() << ",from VPA["<<m->getSrcAddr()<<"] ,messageSequenceVPA: " << messageSequenceVPA << ", messageSequence: "  << messageSequence <<"," <<endl;
    	//EV << "logs, (Now Im in modeDissemination) Rescheduling TX Timer " <<endl;
        EV <<"logs, from VPA, " << m->getSrcAddr() << ", TO VEH, " << myApplAddr() << ","<< traci->getExternalId() <<","<< simTime() << ",rx," <<  junctionID << ", messageSequenceVPA, " << messageSequenceVPA << ", messageSequence, "  << messageSequence <<"," <<endl;
        counterRx++; //count the RX events.
        break;

    case BROADCAST_VEH_WMS: //When receiving Vehicle Broadcast
    	EV << "logs, VEHICLE Received BROADCAST_VEH_WMS " <<endl;
    	if (!junctionRange) { EV << "logs, NOT Listening Out of JunctionRange" << endl; break;} //Voiture dormant

    	currentMessageSequence = messageSequence; //retain current values
    	currentMessageSequenceVPA = messageSequenceVPA; //retain current values
    	splitMessagesReceived(msg);//Split the string: const char* "VPAId+sequenceNumber" & update vehicle's variables

    	EV << "logs, My Current messageSequenceVPA: " << currentMessageSequenceVPA  << ", current MessageSequence: " << currentMessageSequence <<","<< endl;
    	if (messageSequence > currentMessageSequence) {
        	EV << "logs, Updated messageSequenceVPA: " << messageSequenceVPA << ", messageSequence: "  << messageSequence <<"," <<endl;
    	}else { // If received values are not bigger return to current values.
    		messageSequenceVPA = currentMessageSequenceVPA;
    		messageSequence = currentMessageSequence;
    	}
    	EV << "logs, My Final messageSequenceVPA, " << messageSequenceVPA << ", messageSequence: "  << messageSequence <<"," <<endl;

    	/*********************IDENTIFYING  VEHICLE MESSAGE **************/
    	        //if I noticed a Reception of the current sectorID where the vehicleId transit.
    	    	// Check only once per sector, in case the receivedSectorId is false.
    	    	if (!receivedSectorId){
    	    		if (currentSector == messageSequenceVPA){
    	    			MYDEBUG <<"logs, vehicle sector,"<< traci->getExternalId() << ",sector," << currentSector <<",timer,"<< currentSectorTimer <<",receivedsectorId,"<< receivedSectorId <<",from VEH," << m->getSrcAddr() <<","<<endl;
    	    			receivedSectorId= true; //update the Veh sectorId
    	    			currentSectorTimer= 0; //Reset timer
    	    		}
    	    	}
    	/*********************IDENTIFYING  VEHICLE MESSAGE **************/


    	modeDissemination = true; //here vehicle becomes a beacon but it have to expire accordingly with the hold WMS
        cancelEvent(delayTimer); //Delete anterior event and create delayed newer one.

        //re-Set  self-message.
        if (!appCW){  //NO APPS_CW
            EV << "logs, no apps CW " <<endl;
        	scheduleAt(simTime() + T, delayTimer);
        }
        else 		//using APPS_CW
        {
        	CW= intuniform(0, appMinCW)* ST; //computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW, "<< CW  <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
        }
        /*
        //re-Set timer to send a newer self message.
		CW= intuniform(0, 10)* 20E-6;//computing CW
		EV << "logs, veh, " << myApplAddr() << ", using CW, "<< CW  <<endl;
    	//delayTimer = new cMessage( "delay-timer", SEND_BROADCAST_TIMER );
        scheduleAt(simTime() +  T + CW, delayTimer); // T + CW
		*/

    	EV << "logs, (Now Im in modeDissemination) Rescheduling TX Timer " <<endl;
    	MYDEBUG <<"logs, stats, from VEH,"<< m->getSrcAddr() << ",TO VEH, "<< myApplAddr() <<","<< traci->getExternalId() <<","<< simTime() << ",rx," << junctionID << ", messageSequenceVPA, " << currentMessageSequenceVPA  << ", current MessageSequence, " << currentMessageSequence <<",currentSector," << currentSector <<","<<endl;
        counterRx++; //count the RX events.
    	break;

    default:
        EV <<"logs, Unkown selfmessage! -> delete, kind, "<<msg->getKind()<<endl;
        break;
    }
    delete msg; //finally delete the message
}



void VEHICLE::splitMessagesReceived(cMessage* msg) {
	ApplPkt *m = static_cast<ApplPkt *>(msg);
	const char* messageReceived= m->getFullName(); //have a copy of the message received
	char* y;//I'll use this to do the cast_down.
	char* pch; //I'll use this to receive the split data

	// This long and complex code is just to changing types to split data.
	y= const_cast<char *> (messageReceived); //cast from const char*  TO char*
	pch = strtok (y,"+"); //obten el valor de la derecha del signo +
	sscanf(pch, "%d", &messageSequenceVPA); //This magic line Convert the payload-string to Int. UPDATE THE VEHICLE: messageSequenceVPA
	pch = strtok (NULL,"+"); //obten el siguiente valor despues del signo +
	sscanf(pch, "%d", &messageSequence); //This magic line Convert the payload-string to Int.UPDATE THE VEHICLE: messageSequence

}



void VEHICLE::sendMessage() {
	//Next paragraph is to Transmit the message (VPAiD+sequenceNumber) by VPA or Vehicles.

	// TO FIX! clean this arrays to avoid carrying trash.
	char numstr[5]; // Numbered Message
	sprintf(numstr, "%d+%d", messageSequenceVPA,messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	char* result = numstr; //concatenate in VPAiD,messageSequence

	ApplPkt *pkt = new ApplPkt(result, BROADCAST_VEH_WMS);
    pkt->setDestAddr(LAddress::L3BROADCAST); //instead of	pkt->setDestAddr(-1);
	pkt->setSrcAddr(myApplAddr()); // we use the host modules getIndex() as a appl address
	pkt->setBitLength(headerLength);
	// set the control info to tell the network layer the layer 3 address;
	NetwControlInfo::setControlInfo(pkt, LAddress::L3BROADCAST );
	sendDown(pkt);
	MYDEBUG <<"logs, VEH, " <<simTime() <<",From, " << traci->getExternalId() << "," << myApplAddr() <<",tx," <<  junctionID << ", messageSequence, " <<  messageSequence << ", messageSequenceVPA, " << messageSequenceVPA << "," <<endl;
}


void VEHICLE::inJunction() {

	/* OK I drop the idea about using a big array of junctions.
	 * What I really need is to tag and identify the nodes where vehicles are rolling.
	 * What I'll do is:
	 * 1. Identify the current edgeId and position where the vehicle's running.
	 * 2. Verify if I'm on head or tail of the coverage node area
	 */

    std::string laneId;
    double laneLength;
    double lanePosition;

    laneId = traci->getRoadId() + "_0"; //I need the lane ID to ask for the lane length.
    laneLength= traci->cmdGetLaneLength(laneId);
    lanePosition= traci->cmdGetVehiclelanePosition();
    laneId.erase( laneId.end()-2, laneId.end() ); //This chomp the last two strings in order to retain only the edge ID.

    MYDEBUG << "logs, Flag1 Junction() position, "<<traci->getExternalId()<<  ", roadID, " << laneId << " ,length, " << laneLength << ",Position, "<< lanePosition  <<endl;

    //Find if the car is in the middle of a junction.
    size_t found=laneId.find(":");// si ':' esta en laneId agrega un uno a found
      if (found!=std::string::npos){ //si found es diferente de npos (npos= const -1) entonces hay un match.
          //EV << "logs, Found : this is a Junction," << laneId << endl;
          junctionRange = true;
          junctionID= laneId;
      }
    //OK aqui checa si estoy en la cabeza en la cola o en el cruce.
      else if ( lanePosition <= epicenterRange ){
		junctionRange = true;
		junctionID= laneId+"_head";
      }
      else if(lanePosition >= (laneLength - epicenterRange)) {
		junctionRange = true;
		junctionID= laneId+"_tail";
      }
      else {
		junctionRange = false;
		junctionID="";
      }

      //Print the results.
    if(junctionRange){
    	MYDEBUG <<"logs, Flag Im in Junction, "<< traci->getExternalId() << ",junctionId," << junctionID <<","<<endl;
    }
}


void VEHICLE::whatSectorIm() {
	/* OK the Idea here is to maintain an vehicle variable about where is located
	 * 1) I'll obtain the correct XY position of the vehicle
	 * 2) Calculate to determine in which sectorID is the vehicle
	 * 3) Update the variable: currentSector
	 */
	Coord vehiclePosition;//actual XY vehicle position
	int row; //row of sector
	int col; //column of sector
	int inSector; //the sector in.

	//step 1
	vehiclePosition = traci->getCurrentPosition();
	double axeY= maxY

			- vehiclePosition.y;
	//MYDEBUG <<"logs, vehicle position," << traci->getExternalId() << ",x," << vehiclePosition.x <<",y,"<< axeY  <<endl;

	//step 2
	row= int( (vehiclePosition.x - offsetX) /1000); //Substract the 4k axis offset. Divide by 1000 and get the integer.
	col= int( (axeY - offsetY) /1000); //Substract the 5k axis offset. Divide by 1000 and get the integer.
	inSector= (row * 33) + col; //Every column has 33rows. Sectors starts in cero from bottom to top and left to right.
	//MYDEBUG <<"logs, vehicle sector," << traci->getExternalId() << ",row," << row <<",colum,"<< col<<",sector number,"<< currentSector <<"," <<endl;

	//if I noticed a change of sectorID reset everything. Also gives the current counter o'course.
	//Note: this counter is complemented with the counter in received messages by VPAs or Vehicles.
	if (inSector != currentSector){
		MYDEBUG <<"logs, vehicle sector," << traci->getExternalId() << ",sector," << currentSector <<",timer,"<< currentSectorTimer <<",receivedsectorId,"<< receivedSectorId <<",," <<endl;
		currentSector= inSector; //update Veh sector transit
		currentSectorTimer= 0; //Reset timer
		receivedSectorId= false; //reset the Veh sectorId reception
	}
	currentSectorTimer++; //Counter of the Vehicle sector.
}


void VEHICLE::vehicleVideos() {
	/*
	 * OK, here I'll create the logs in order to generate nice vehicles videos
	 * Videos have an step of 5sg so I'll call this function every five seconds.
	 * The log format is the next:
	 *
	 * carPosition, time, carId, X, Y, speed, mesageOrNot,numberOfNeighbors,InJunction, InDissemination, messageSequence,
	 *
	 * 1srt. get the list of all active vehicles. ERROR! I've to understand that every function is instantiated for EVERY CAR!!
	 * 2nd. Retrieve for each vehicle its information and finally creates the log
	 */

	Coord vehPos = traci->getCurrentPosition();
	double axeY= maxY - vehPos.y;

    MYDEBUG << "logs, carPosition," << simTime() <<","<< traci->getExternalId()  <<","<< vehPos.x <<","<<  axeY <<","<< traci->getSpeed() <<","<< receivedSectorId <<","<<  neighbors <<","<< junctionRange <<","<< modeDissemination <<","<< messageSequence <<","<<endl;
}


void VEHICLE::WMS() {

	/* OK Here I'll get the Weigth Map Sector.
	 * In order to gather the Arcs' average Travel Time I'll have the vehicles report this value.
	 * Every Vehicle will register the IN/OUT times of every traversed ARC. If not info gather
	 * from an ARC I'll the default TT of the ARC. It means that I'll only give away the ARCs with
	 * interesting information.
	 *
	 * This information I'll set in a global array that will be used by the VPAs.
	 * Also Every vehicle will be able to retransmit this information through the VPA in order to
	 * use vehicles as traffic probes.
	 *
	 * NOTE: THIS MODULE WAS SUPPPOSED TO BE FINISHED AND ALREADY POLISHED!!! DAMN IT! MOVE u'rASS!
	 */
}


////////////////// TESTING AREA /////////////    ////////////////// TESTING AREA /////////////

/*
MYDEBUG << "DEBUG getRoadId: " << traci->getRoadId() <<endl;
MYDEBUG << "DEBUG cmdGetVehiclelanePosition: " << traci->cmdGetVehiclelanePosition()  <<endl;
std::string laneId = traci->getRoadId() + "_0";
MYDEBUG << "DEBUG cmdGetLaneLength: " << laneId << " length: " << traci->cmdGetLaneLength(laneId)  <<endl;

// TRACI GET DATA
MYDEBUG << "DEBUG getID: " << traci->getId() <<endl;
MYDEBUG << "DEBUG getIndex: " << traci->getIndex() <<endl;
MYDEBUG << "DEBUG getCurrentPosition: " << traci->getCurrentPosition() <<endl;
MYDEBUG << "DEBUG getCurrentSpeed: " << traci->getCurrentSpeed() <<endl;
MYDEBUG << "DEBUG getfullPath: " << traci->getFullPath() <<endl;
MYDEBUG << "DEBUG getDefaultOwner: " << traci->getDefaultOwner() <<endl;
MYDEBUG << "DEBUG getExternalId: " << traci->getExternalId() <<endl;
MYDEBUG << "DEBUG getFullName: " << traci->getFullName() <<endl;
MYDEBUG << "DEBUG getLiveObjectCount: " << traci->getLiveObjectCount() <<endl;
MYDEBUG << "DEBUG getManager: " << traci->getManager() <<endl;
MYDEBUG << "DEBUG getSpeed: " << traci->getSpeed() <<endl;
MYDEBUG << "DEBUG getAngleRad: " << traci->getAngleRad() <<endl;
MYDEBUG << "DEBUG getSubscribeCount: " << traci->getSubscribeCount() <<endl;
MYDEBUG << "DEBUG getPositionAt(simTime()): " << traci->getPositionAt(simTime()) <<endl;
MYDEBUG << "DEBUG getVectorSize: " << traci->getVectorSize() <<endl;
MYDEBUG << "DEBUG getClassName: " << traci->getClassName() <<endl;
MYDEBUG << "DEBUG getComponentType: " << traci->getComponentType() <<endl;
MYDEBUG << "DEBUG getDefaultOwner: " << traci->getDefaultOwner() <<endl;
MYDEBUG << "DEBUG getDescriptor: " << traci->getDescriptor() <<endl;
MYDEBUG << "DEBUG getDisplayString: " << traci->getDisplayString() <<endl;
//MYDEBUG << "DEBUG getVectorSize: " << traci->getLocalListenedSignals() <<endl;
//MYDEBUG << "DEBUG getGateNames: " << traci->getGateNames() <<endl;
MYDEBUG << "DEBUG getTotalObjectCount: " << traci->getTotalObjectCount() <<endl;
MYDEBUG << "DEBUG commandGetArcCO2: " << traci->commandGetArcCO2("C2")  <<endl;
MYDEBUG << "DEBUG commandGetArcOccupancy: " << traci->commandGetArcOccupancy("C2")  <<endl;

//Get the full list of active vehicles.
MYDEBUG << "DEBUG getExternalId: " << traci->getExternalId() <<endl;
std::list<std::string> vehiclesId = traci->commandGetVehiclesIds();
for (std::list<std::string>::const_iterator i = vehiclesId.begin(); i != vehiclesId.end(); ++i) {
    MYDEBUG << "DEBUG commandGetVehiclesIds: " << *i  <<endl;
}

//Get the full list of ArcsID Cela donne beaucoup d'info
MYDEBUG << "DEBUG getExternalId: " << traci->getExternalId() <<endl;
std::list<std::string> arcId = traci->commandGetArcIds();
for (std::list<std::string>::const_iterator i = arcId.begin(); i != arcId.end(); ++i) {
    MYDEBUG << "DEBUG commandGetArcIds: " << *i  <<endl;
}

//Get the route plan that left from the a particular vehicle perspective.
MYDEBUG << "DEBUGHERE getExternalId: " << traci->getExternalId() <<endl; //print Vehicle.ID
std::list<std::string> vehicleId = traci->commandGetSingleVehicleRoutes(); // the vehicleId is append in the class
for (std::list<std::string>::const_iterator i = vehicleId.begin(); i != vehicleId.end(); ++i) {
    MYDEBUG << "DEBUG commandGetSingleVehicleRoutes: " << *i  <<endl;
}

// TRACI SET COMMANDS
//Ici On change le parcours d'un voiture..
if ( traci->getExternalId() == "veh.1" and simTime() < 35) {
    MYDEBUG << "DEBUG commandChangeRoute: " <<endl;
   // traci->commandSetSpeed(2); //set speed to 2m/s
   // traci->commandStopNode("B1",80.0,0,5,5);
    traci->commandChangeRoute("B2",30.0); //set TT
    traci->commandChangeRoute("C2",30.0);
    traci->commandChangeRoute("A3",30.0);
    traci->commandChangeRoute("B3",30.0);
}

//Cela je pense que ecoute et se active quand on recoi une recepcion de signal.
// This is executed all the time, for the moment I'll stop it
void VEHICLE::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj) {
	EV << "logs, ReceiveSignal source: "<< source << " signal: " <<  signalID << endl;

	Enter_Method_Silent(); //cela à qoui serve?
	if (signalID == mobilityStateChangedSignal) { //cela c'est quoi? si est que il y a de signal presente essais d'envoyer
		//handlePositionUpdate();
	}
}

*/
