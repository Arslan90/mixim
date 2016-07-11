//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
/*********************************************
 * THIS IS MY VEHICLE application.
 * ADD OWN MAP,  OK
 * ADD TRACI GET DATA, OK
 * ADD VPA ENTITY. OK
 * ADD VEHICLE FLAG WHEN in/out of JUNCTIONS.
 *
 * The scenario is: Simple re-forwarding of messages..
 *
 *TODO: Ok, Here I've finished the VEHICLEp.h based on VEHICLE.h and what left to do is
 *		to agregate the functions here.
 *
 *********************************************/

#include "simpleVEHICLEpOpp.h"
#include "multiFunctions.h"
#include "ApplOppControlInfo.h"
#include "DtnNetwLayer.h"
#include "TraCIMobility.h"


Define_Module(simpleVEHICLEpOpp);

const simsignalwrap_t simpleVEHICLEpOpp::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void simpleVEHICLEpOpp::initialize(int stage) {

	DtnApplLayer::initialize(stage);
	if (stage == 0) {
		//TRACI STUFF
		TraCIMobility* traciMobility = TraCIMobilityAccess().get(getParentModule());
		traci = check_and_cast<simpleTraCIMobility*>(traciMobility);
		findHost()->subscribe(mobilityStateChangedSignal, this);

		//ARTURO my passed values:
		T= hasPar("timeT") 	? par("timeT").doubleValue(): 5;
		epicenterRange= hasPar("epicenterValue") ? par("epicenterValue").doubleValue() : 20;
	    appCW = hasPar("appCW") ? par("appCW").boolValue(): false; //wheter I'll use or not the APP WC.
	    appMinCW = hasPar("appMinCW") ? par("appMinCW").doubleValue(): 100; //my APPS CW
		EV << "logs, Using value T=" << T <<" epicenter=" << epicenterRange<< ",appCW,"<< appCW <<",appMinCW,"<< appMinCW <<" in:"<< traci->getExternalId() <<endl;

		//offsetX= 4000; // I've to adjust adding the X axis value
		//offsetY= 5000; // I've to adjust adding the Y axis value
		//maxY= 41947; //This is the Maximun Y axis value of the MAP.
		//maxY= 200; //Add the Max value of Y of the scenario simulated.
		//maxY= 2100; //Add the Max value of Y of the scenario simulated.
//		offsetX= hasPar("offsetX") ? par("offsetX").doubleValue() : 4000;
//		offsetY= hasPar("offsetY") ? par("offsetY").doubleValue() : 5000;
//		maxY= hasPar("maxY") ? par("maxY").doubleValue() : 41947;
//		EV << "logs, offset values offsetX=" << offsetX <<" offsetY=" << offsetY<< ",maxY,"<< maxY <<endl;

		headerLength = par("headerLength");
        modeDissemination= false; //When creating the vehicle is in mode listening
        junctionRange= false; //set false the junction range
		CW= 0; //Contention Window value.
		messageSequence= 0;//sequence of the VPA message number sent.
		messageSequenceVPA= 0;//ID number of the VPA sent.
		//junctionID= 0; //initialize. A value= 0 is vehicle not in any junction.
		counterRx=0; //This is a counter of the messages reception (Rx). Maybe I'll use to calculate the CW
		neighbors=0; //this keep the value of the neighbors detected pending a determined period of time (e.g. 1sg)
		currentSectorTimer= 0; //start in clean.
		receivedSectorId= false; //reset the Veh sectorId reception

		droppedMessages= 0;//received Mac80211p dropped messages
		currentCW= appMinCW; //CWMIN_11P/20; //0; //received Mac80211p CW. NOTA Menor el divisor mejor los resultados
		counterDroppedMessages=0; //Total count Mac80211p dropped message.
		Tupdate= 5; //periodic time to obtain the dropped packets.

		vehTimeIn= simTime(); //Time the vehicle enter into the simulation
		vehTimeOut= 0; //Time the vehicle get out of the simulation
		vehRx= 0; //Vehicle total Receptions.

        //The self messages
		delayTimer = new cMessage( "delay-timer", SEND_BROADCAST_TIMER );
		everySecond = new cMessage( "everySecond", DO_THINGS_EVERY_SECOND );

		// added by me for testing
//		dtnTestMode = par("dtnTestMode").boolValue();
//		dtnSynchronized = par("dtnSynchronized").boolValue();

//		sectorMode = par("sectorMode").boolValue();
//		anyVPA = par("updateMode").boolValue();
//		nbrBundleSent = 0;
//		nbrBundleReceived = 0;
//		if (dtnTestMode){
//			dtnTestMsg = new cMessage( "dtn Test", DTN_TEST_MODE);
//			dtnTestCycle = par("dtnTestCycle");
//			dtnTestMaxTime = par("dtnTestMaxTime");
//
//		}

    	periodForMsg = par("periodForMsg");
    	nbrMsgByStep = par("nbrMsgByStep");

    	remainingMsgByStep = 0;

        connected = registerSignal("connected");
        simulation.getSystemModule()->subscribe(connected, this);

		vehStopped = registerSignal("vehStopped");
		simulation.getSystemModule()->subscribe(vehStopped, this);

		stopPos = par("stopPos").doubleValue();
		stopDuration = par("stopDuration").doubleValue();
		stopDistStep = par("stopDistStep").doubleValue();
		releasePos = par("releasePos").doubleValue();

		targetPos = par("targetPos").doubleValue();

		with127M = par("with127M").boolValue();

		for (double i = targetPos-stopPos;i > targetPos-releasePos; i-=stopDistStep){
			std::ostringstream iss;
			iss << i;
			nbrSentByDist.insert(std::pair<std::string,double>(iss.str(),0));
//			nbrReceivedByDist.insert(std::pair<std::string,double>(iss.str(),0));
		}

		if (with127M){
			std::ostringstream iss;
			iss << 127;
			nbrSentByDist.insert(std::pair<std::string,double>(iss.str(),0));
//			nbrReceivedByDist.insert(std::pair<std::string,double>(iss.str(),0));
		}
	}
	if(stage==1){
		traci->getManager()->addVehicleID(traci->getExternalId());
		traci->setStopDuration(nbrMsgByStep*periodForMsg);
		loopVehicle = false;
		loopVehicle = traci->getManager()->isALoopVehicle(traci->getExternalId());

		currentRoute = traci->commandGetSingleVehicleRoutes();
		currentRouteId = traci->commandGetRouteId();

		EV << "My current route id is : " << currentRouteId << endl;

		reroutedToLoopRoute = false;
		edgeForLooping ="";


	}
	if(stage==2) {
		scheduleAt(simTime() + 1, delayTimer); //Scheduling the self message.
		scheduleAt(simTime() + 1, everySecond); //Scheduling the self message.

//		// added by me for testing
//		if (dtnTestMode){
//			double tmp;
//			tmp =  (dtnSynchronized)? 0 : uniform(0,dtnTestCycle) ;
//			if ((simTime() + tmp <dtnTestMaxTime) && (!sectorMode)){
//				scheduleAt(simTime() + tmp, dtnTestMsg);
//			}
//		}

	}
}

//SELF-MESSAGES
void simpleVEHICLEpOpp::handleSelfMsg(cMessage* msg) {
	EV << "logs, SELF-message RECEIVED in, "<< traci->getExternalId() <<" kind= "<< msg->getKind() <<endl; // pour voir qui recoi le message..

	if (isEquiped){

    switch(msg->getKind())
    {
    // added by me
    case DTN_TEST_MODE:
    	if (withDtnMsg){
    		sendDtnMessage();
    		// Finally reschedule message
			if(withDtnMsg && (sendingStrategy == Periodic)) {
				if (remainingMsgByStep > 0){
					scheduleAt(simTime() + periodForMsg, dtnMsg);
				}
				nbrMsgSent++;
			}

//    		if ((simTime() + dtnTestCycle < dtnTestMaxTime) && (!sectorMode)){
//    			scheduleAt(simTime() + dtnTestCycle, dtnTestMsg);
//				nbrMsgSent++;
//			}
    	}
    	break;
    case SEND_BROADCAST_TIMER:
    	if (modeDissemination and junctionRange) { //Vehicle send WMS beacon when in mode Dissemination & near enough to junction.
    		if (withOtherMsg){
    			EV << "logs, VEH Sending PACKET. " << endl;
				sendMessage();//Finally send the message.
				nbrMsgSent++;
    		}
    	}

        //Re-schedule the self-message.
        if (appCW){ // With APPS_CW
        	/*double dinamicST= double(droppedMessages) * (CWMAX_11P - currentCW)+ appMinCW;
        	CW= intuniform(0, int(dinamicST)) * SLOTLENGTH_11P; //DYNAMIC computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW value, "<< CW <<",currentCW," << currentCW <<",intentos,"<< droppedMessages <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
			*/
        	CW= intuniform(0, appMinCW)* SLOTLENGTH_11P; //DYNAMIC computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW value, "<< CW <<",currentCW," << currentCW <<",intentos,"<< droppedMessages <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
        }
        else 	//Without APPS_CW
        {
            EV << "logs, no apps CW " <<endl;
        	scheduleAt(simTime() + T, delayTimer);
        }

     break;

    case DO_THINGS_EVERY_SECOND:
    	inJunction(); //Update if vehicleId is in junctionId area
    	whatSectorIm(); //Check in which sector is the vehicle

    	// check if the sector changed
		if (withDtnMsg && (sendingStrategy == SectorEntry) && (currentSector != oldSector)){
			scheduleAt(simTime(), dtnMsg);
			nbrMsgSent++;
		}

//    	if (sectorMode){
//    		if ((dtnTestMode) && (currentSector != oldSector)){
//    			scheduleAt(simTime(), dtnTestMsg);
//				nbrMsgSent++;
//    		}
//    	}

//		oldSector = currentSector;



	//HACK turn off Videos for faster simulation.
    	//vehicleVideos(); //This is to generate videos, CAUTION it generates copious logs.
    	WMS(); //Gather the Weight Map Sector.//TODO: Is missing how to agreggate the WMS.Check the function for further information
    	readNicFiles();//Read the nicFiles and update the local information.

    	if (modeDissemination){ EV << "logs, Im in modeDissemination "<< endl;} else {EV << "logs, Im not in modeDissemination" << endl;};
        //if (modeDissemination) { EV <<"logs, Flag, dissemination ON,"<< simTime() <<"," << traci->getExternalId() <<endl; }
    	if (junctionRange){
    		EV << "logs, Im in junction "<< endl;} else {EV << "logs, Im not in junction" << endl;
    		EV << "logs, neighbors:" << neighbors << ", reports,"<< traci->getExternalId() <<", at," << simTime() <<","<<endl;
        }

        neighbors= counterRx; //Count current neighbors with the value of RX events.
        counterRx= 0; //Reset Rx counter every N seconds.
    	vehTimeOut= simTime(); //Updating vehicle timeOut, in finish I'll check when stop.


    	//re-schedule self-message.
		//everySecond = new cMessage( "delay-timer", DO_THINGS_EVERY_SECOND ); //self_message
        scheduleAt(simTime() + 1, everySecond); //Scheduling the self message.
		break;

    default:
        EV <<"logs, Unknown self-message! -> delete, kind: "<<msg->getKind()<<endl;
        break;
    }

	}
}



//RECEIVED PACKETS.
void simpleVEHICLEpOpp::handleLowerMsg(cMessage* msg) {
	NetwPkt* netw = dynamic_cast<NetwPkt*>(msg);
	ASSERT(netw);
	WaveShortMessage*  wsm =  dynamic_cast<WaveShortMessage*>(netw->decapsulate());

	EV << "logs, PACKET RECEIVED in VEHICLE: "<< traci->getExternalId() <<" Data: " << wsm->getName() << ",Kind,"<< wsm->getKind() <<endl; // pour voir qui recoi le message..
	int currentMessageSequence;
	int currentMessageSequenceVPA;
	const char* messageReceived;
	char* y;//I'll use this to do the cast_down
	char* pch;//I'll use this to receive the split data


	nbrMsgReceived++;

    switch(wsm->getKind())
    {
    case DTN_TEST_MODE:
		if ((withDtnMsg) && ((getDataDestFromStrategy(strategy)=="ALL") || (getDataDestFromStrategy(strategy)=="VEH"))){
			nbrBundleReceived++;
			simtime_t time = (simTime()-wsm->getTimestamp());

			if ((receivedBundles.empty())	||	(receivedBundles.find(wsm->getSerial())== receivedBundles.end())){
				receivedBundles.insert(std::pair<unsigned long long ,WaveShortMessage*>(wsm->getSerial(), wsm));
				nbrUniqueBundleReceived++;

				totalDelay = totalDelay + time.dbl();
				if (nbrUniqueBundleReceived>0){
					avgDelay = totalDelay / nbrUniqueBundleReceived;
				}else{
					opp_error("Nbr Unique Bundle Received equal or less then zero(VPApOpp::handleLowerMsg)");
				}

				delayStats.collect(time.dbl());
				hopCountStats.collect(wsm->getHopCount());
			}
		}
		break;
//When receiving VPA Broadcast
	case BROADCAST_VPA_WMS:

    	//here WE DO NOT HAVE Voiture dormant, 'cause VPA sources always are listened no matter what, when, where.
    	//I'll profit of this function to know if vehicles have received the SectorID.

/////////////// SPLIT MESSSAGE ////////////
		messageReceived= wsm->getName() ; //have a copy of the message received
		// This long and complex code is just to changing types to split data.
		y= const_cast<char *> (messageReceived); //cast from const char*  TO char*
		pch = strtok (y,"+"); //obten el valor de la derecha del signo +
		sscanf(pch, "%d", &messageSequenceVPA); //This magic line Convert the payload-string to Int. UPDATE THE VEHICLE: messageSequenceVPA
		pch = strtok (NULL,"+"); //obten el siguiente valor despues del signo +
		sscanf(pch, "%d", &messageSequence); //This magic line Convert the payload-string to Int.UPDATE THE VEHICLE: messageSequence
/////////////// SPLIT MESSSAGE ////////////

////////////////////// IDENTIFYING  VEHICLE MESSAGE **************
        //if I noticed a Reception of the current sectorID where the vehicleId transit.
    	// Check only once per sector, in case the receivedSectorId is false.
    	if (!receivedSectorId){
    		if (currentSector == messageSequenceVPA){
    			EV <<"logs, vehicle sector,"<< traci->getExternalId() << ",sector," << currentSector <<",timer,"<< currentSectorTimer <<",receivedsectorId,"<< receivedSectorId <<",from VPA," <<  myApplAddr()<<"," <<endl; //m->getSrcAddr() <<"," <<endl;
    			receivedSectorId= true; //update the Veh sectorId
    			currentSectorTimer= 0; //Reset timer
    		}
    	}
////////////////////// IDENTIFYING  VEHICLE MESSAGE **************

    	modeDissemination = true;
        cancelEvent(delayTimer); //Delete anterior event and schedule a newer one.


        //Re-schedule the self-message.
        if (appCW){ // With APPS_CW
        	/*double dinamicST= double(droppedMessages) * (CWMAX_11P - currentCW)+ appMinCW;
        	CW= intuniform(0, int(dinamicST)) * SLOTLENGTH_11P; //DYNAMIC computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW value, "<< CW <<",currentCW," << currentCW <<",intentos,"<< droppedMessages <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
			*/
        	CW= intuniform(0, appMinCW)* SLOTLENGTH_11P; //DYNAMIC computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW value, "<< CW <<",currentCW," << currentCW <<",intentos,"<< droppedMessages <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
        }
        else 	//Without APPS_CW
        {
            EV << "logs, no apps CW " <<endl;
        	scheduleAt(simTime() + T, delayTimer);
        }


    	EV << "logs, VEHICLE RECEIVED: BROADCAST_VPA_WMS in: "<< traci->getExternalId() << ",kind: " << msg->getKind() << ",from VPA["<< myApplAddr() <<"] ,messageSequenceVPA: " << messageSequenceVPA << ", messageSequence: "  << messageSequence <<"," <<endl;
        EV <<"logs, from VPA, " << messageSequenceVPA << ", TO VEH, " << myApplAddr() << ","<< traci->getExternalId() <<","<< simTime() << ",rx," <<  junctionID << ", messageSequenceVPA, " << messageSequenceVPA << ", messageSequence, "  << messageSequence <<"," <<endl;
        EV <<"logs, positon,"<< traci->getCurrentPosition()  <<"," <<endl;
        //EV <<"logs, backoff,RX," << currentSector << "," << traci->getExternalId() <<","<< simTime()  <<"," <<endl;

        counterRx++; //count the RX events.
		vehRx++; //Vehicle total Receptions.
        break;
//When receiving Vehicle Broadcast
	case BROADCAST_VEH_WMS:
    	EV << "logs, VEHICLE Received BROADCAST_VEH_WMS" <<endl;
    	/*IMPORTANTE!  VOITURE SOURDE ACTIVE! REVISAR SI ESTO TIENE ALGUN EFECTO NOCIVO EN LOS RESULTADOS!!!!
    	 * YO creo que sera importante mostrar los resultados con y sin diseminacion de vehiculos.
    	 * Para mejorar este valor puedo poner los VPAs que transmitan cada 1 o 5min y que los
    	 * vehiculos hagan el resto del trabajo de la diseminacion*/

	//OK, this is important to make a difference in the messages received.
    	if (!junctionRange) { EV << "logs, NOT Listening Out of JunctionRange" << endl; break;} //Voiture dormant
    	else { EV << "logs, YES Listening IN JunctionRange" << endl; } //Voiture dormant


/////////////// SPLIT MESSSAGE ////////////
    			messageReceived= wsm->getName() ; //have a copy of the message received
    			// This long and complex code is just to changing types to split data.
    			y= const_cast<char *> (messageReceived); //cast from const char*  TO char*
    			pch = strtok (y,"+"); //obten el valor de la derecha del signo +
    			sscanf(pch, "%d", &messageSequenceVPA); //This magic line Convert the payload-string to Int. UPDATE THE VEHICLE: messageSequenceVPA
    			pch = strtok (NULL,"+"); //obten el siguiente valor despues del signo +
    			sscanf(pch, "%d", &messageSequence); //This magic line Convert the payload-string to Int.UPDATE THE VEHICLE: messageSequence
/////////////// SPLIT MESSSAGE ////////////

    	//NOTE: THIS seems to neutralize the vehicle dissemination
    	// because always set to 0 their messageSequenceVPA
    	EV << "logs, My Current messageSequenceVPA: " << currentMessageSequenceVPA  << ", current MessageSequence: " << currentMessageSequence <<","<< endl;
    	/*if (messageSequence > currentMessageSequence) {
        	EV << "logs, Updated messageSequenceVPA: " << messageSequenceVPA << ", messageSequence: "  << messageSequence <<"," <<endl;
    	}else { // If received values are not bigger return to current values.
    		messageSequenceVPA = currentMessageSequenceVPA;
    		messageSequence = currentMessageSequence;
    	}*/
    	EV << "logs, My Final messageSequenceVPA, " << messageSequenceVPA << ", messageSequence: "  << messageSequence <<"," <<endl;

////////////////////// IDENTIFYING  VEHICLE MESSAGE **************
    	        //if I noticed a Reception of the current sectorID where the vehicleId transit.
    	    	// Check only once per sector, in case the receivedSectorId is false.
   	    	if (!receivedSectorId){
    	    		if (currentSector == messageSequenceVPA){
    	    			EV <<"logs, vehicle sector,"<< traci->getExternalId() << ",sector," << currentSector <<",timer,"<< currentSectorTimer <<",receivedsectorId,"<< receivedSectorId <<",from VEH," << myApplAddr() <<","<<endl;
    	    			receivedSectorId= true; //update the Veh sectorId
    	    			currentSectorTimer= 0; //Reset timer
    	    		}
    	    	}
////////////////////// IDENTIFYING  VEHICLE MESSAGE **************


    	modeDissemination = true; //here vehicle becomes a beacon but it have to expire accordingly with the hold WMS
        cancelEvent(delayTimer); //Delete anterior event and create delayed newer one.


        //Re-schedule the self-message.
        if (appCW){ // With APPS_CW
        	/*double dinamicST= double(droppedMessages) * (CWMAX_11P - currentCW)+ appMinCW;
        	CW= intuniform(0, int(dinamicST)) * SLOTLENGTH_11P; //DYNAMIC computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW value, "<< CW <<",currentCW," << currentCW <<",intentos,"<< droppedMessages <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
			*/
        	CW= intuniform(0, appMinCW)* SLOTLENGTH_11P; //DYNAMIC computing my APPS CW_min based on the 802.11p slot time.
        	EV << "logs, veh, " << myApplAddr() << ", my apps CW value, "<< CW <<",currentCW," << currentCW <<",intentos,"<< droppedMessages <<endl;
        	scheduleAt(simTime() + T + CW, delayTimer);
        }
        else 	//Without APPS_CW
        {
            EV << "logs, no apps CW " <<endl;
        	scheduleAt(simTime() + T, delayTimer);
        }


    	EV << "logs, (Now Im in modeDissemination) Rescheduling TX Timer " <<endl;
    	MYDEBUG <<"logs, from VEH,"<< netw->getSrcAddr() << ",TO VEH, "<< myApplAddr() <<","<< traci->getExternalId() <<","<< simTime() << ",rx," << junctionID << ", messageSequenceVPA, " << currentMessageSequenceVPA  << ", current MessageSequence, " << currentMessageSequence <<",currentSector," << currentSector <<","<<endl;
        //EV <<"logs, backoff,RX," << currentSector << "," << traci->getExternalId() <<","<< simTime()  <<"," <<endl;

    	counterRx++; //count the RX events.
		vehRx++; //Vehicle total Receptions.
    	break;

    default:
        EV <<"logs, Unknown self-message! -> delete, kind, "<<msg->getKind()<<endl;
        break;
    }

    delete msg; //finally delete the message
    delete wsm;
}




void simpleVEHICLEpOpp::inJunction() {

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

    //MYDEBUG << "logs, overall position, "<<traci->getExternalId()<<  ", roadID, " << laneId << " ,length, " << laneLength << ",Position, "<< lanePosition  <<endl;

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



void simpleVEHICLEpOpp::whatSectorIm() {
	/* OK the Idea here is to maintain an vehicle variable about where is located
	 * 1) I'll obtain the correct XY position of the vehicle
	 * 2) Calculate to determine in which sectorID is the vehicle
	 * 3) Update the variable: currentSector
	 */

	if (scenarioModel == Sector){
		Coord vehiclePosition;//actual XY vehicle position
		Coord vehiclePosSumo;//actual XY vehicle position
		int row; //row of sector
		int col; //column of sector
		int inSector; //the sector in.

		//step 1
		vehiclePosition = traci->getCurrentPosition();
		vehiclePosSumo.x = traci->getManager()->omnet2traci(vehiclePosition).x;
		vehiclePosSumo.y = traci->getManager()->omnet2traci(vehiclePosition).y;
		//MYDEBUG <<"logs, vehicle position," << traci->getExternalId() << ",x," << vehiclePosition.x <<",y,"<< axeY  <<endl;

		//step 2
		col= int( (vehiclePosSumo.x - sectorOffsetX) / sectorSizeX); //Substract the 4k X-axis offset. Divide by 1000 and get the integer.
		if (!((0 <= col)&&(col < colSectorGrid))){
			opp_error("Error with column index calculation");
		}
		row= int( (vehiclePosSumo.y - sectorOffsetY) / sectorSizeY); //Substract the 5k Y-axis offset. Divide by 1000 and get the integer.
		if (!((0 <= row)&&(row < rowSectorGrid))){
			opp_error("Error with row index calculation");
		}
		inSector= (col * rowSectorGrid) + row; //Every column has 33rows. Sectors starts in cero from bottom to top and left to right.
		//MYDEBUG <<"logs, vehicle sector," << traci->getExternalId() << ",row," << row <<",colum,"<< col<<",sector number,"<< currentSector <<"," <<endl;

		//if I noticed a change of sectorID reset everything. Also gives the current counter o'course.
		//Note: this counter is complemented with the counter in received messages by VPAs or Vehicles.
		if (inSector != currentSector){
			MYDEBUG <<"logs, vehicle sector," << traci->getExternalId() << ",sector," << currentSector <<",timer,"<< currentSectorTimer <<",receivedsectorId,"<< receivedSectorId <<",," <<endl;
			//MYDEBUG <<"logs, backoff,out,"<< currentSector <<","<< traci->getExternalId() << "," << simTime() <<"," <<endl;
			//MYDEBUG <<"logs, backoff,in," << inSector      <<","<< traci->getExternalId() << "," << simTime() <<"," <<endl;

			currentSector= inSector; //update Veh sector transit
			currentSectorTimer= 0; //Reset timer
			receivedSectorId= false; //reset the Veh sectorId reception


			if ((receivingStrategy == Any)){
	    		if ((currentSector != oldSector) && (oldSector != -1)){
					cMessage* ctrlMsg = new cMessage();
					ctrlMsg->setName("ControlMsg");
					ApplOppControlInfo* controlInfo = new ApplOppControlInfo(vpaDestAddr(oldSector),vpaDestAddr(currentSector));
					ctrlMsg->setControlInfo(controlInfo);
					sendControlDown(ctrlMsg);
	    		}
			}

		}else {
			oldSector = currentSector;
		}
		currentSectorTimer++; //Counter of the Vehicle sector.
	}
}


void simpleVEHICLEpOpp::vehicleVideos() {
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

//	Coord vehPos = traci->getCurrentPosition();
//	double axeY= maxY - vehPos.y;
//
//    MYDEBUG << "logs, carPosition," << simTime() <<","<< myApplAddr() <<","<< vehPos.x <<","<<  axeY <<","<< traci->getSpeed() <<","<< receivedSectorId <<","<<  neighbors <<","<< junctionRange <<","<< modeDissemination <<","<< messageSequence <<","<<endl;
}

void simpleVEHICLEpOpp::WMS() {

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
	 * NOTE: THIS MODULE WAS SUPPPOSED TO BE FINISHED AND ALREADY POLISHED!!!
	 */
}



void simpleVEHICLEpOpp::sendDtnMessage()
{
	if (getDataDestFromStrategy(strategy)=="VPA"){
		int addr = 0;
		switch (receivingStrategy) {
			case Unique:
				if (scenarioModel == Free){
					addr = destAddr;
				}else if (scenarioModel == Sector){
					whatSectorIm();
					addr = vpaDestAddr(currentSector);
				}
				break;
			case Random:
				addr = randomVPADestAddr();
				break;
			case Any:
				if (scenarioModel == Free){
					opp_error("Must Define an anycast addr for Free model");
				}else if (scenarioModel == Sector){
					whatSectorIm();
					addr = vpaDestAddr(currentSector);
				}
				break;
			default:
				opp_error("Undefined behavior when selecting destinated VPA");
				break;
		}
	//	MYDEBUG <<"logs, VEH," <<simTime() <<",From," << myApplAddr() << "," << traci->getExternalId()  <<",tx," <<  junctionID << ", messageSequence, " <<  messageSequence << ", messageSequenceVPA, " << messageSequenceVPA << ","<< vehPos.x <<","<<  axeY<<"," <<endl;
		//MYDEBUG <<"logs, backoff,tx,"<< currentSector <<","<< traci->getExternalId()<< ","  << simTime() << "," <<endl;


	//	char numstr[6]; // Numbered Message
	//	sprintf(numstr, "%d+%d", messageSequenceVPA,messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	//	char* result = numstr; //concatenate in VPAiD,messageSequence

		//Sending message
		t_channel channel = dataOnSch ? type_SCH : type_CCH;
		//	sendWSM(prepareWSM(result, dataLengthBits, channel, dataPriority, 0,2));

		MYDEBUG <<"periodic DtnMessage sent at " <<simTime() <<",From," << netwAddr << " to address "<< addr <<endl;
		std::string s = "Periodic DTN message sent from :"+netwAddr;
		std::string tmp = "to the current netw addr : "+addr;
		s = s+tmp;
		if (isNetwAddrInit){
		sendWSM(prepareWSM(dist, dataLengthBits, channel, dataPriority, addr,multiFunctions::cantorPairingFunc(netwAddr,nbrMsgSent)));
		nbrBundleSent++;
		remainingMsgByStep--;
		nbrSentByDist[dist]=nbrSentByDist[dist]+1;
		}else {
			opp_error("netw address not yet initialized");
		}
	}else{
		opp_error("Sending DtnMsg to VEH & ALL mode is not supported");
	}
}

void simpleVEHICLEpOpp::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj)
{
	if (signalID == mobilityStateChangedSignal) {
		if (loopVehicle){
			if (!reroutedToLoopRoute){
				// list of all avaibales routes
				std::list<std::string> routes = traci->commandGetRouteIds();
				std::vector<std::string> allRoutes;
				for (std::list<std::string>::iterator it = routes.begin(); it != routes.end(); it++){
					EV << *it << endl;
					allRoutes.push_back(*it);
				}

				// detecting loop routes that are compatible with my actual route
				std::string startingEdge = currentRoute.front();
				std::string startingEdgeLoopRoute;
				std::list<std::string>edgesLoopRoute;

				std::vector<std::string> compatibleLoopRoutes = std::vector<std::string>();
				for (unsigned int i = 0 ; i < allRoutes.size(); i++){
					// check if this route is a loop route
					if (allRoutes[i].find(traci->getManager()->getPrefixForLoopRoute()) == 0){

						edgesLoopRoute = traci->commandGetEdgesOfRoute(allRoutes[i]);
						startingEdgeLoopRoute = edgesLoopRoute.front();

						// check if this route is compatible with my actual route (same starting edge)
						if (startingEdge.compare(startingEdgeLoopRoute) == 0){
							compatibleLoopRoutes.push_back(allRoutes[i]);
						}
					}
				}

				// selecting randomly a compatible loop route
				if (compatibleLoopRoutes.size()>0){
					currentRouteId = compatibleLoopRoutes[intrand(compatibleLoopRoutes.size())];
					currentRoute = traci->commandGetEdgesOfRoute(currentRouteId);

				// detecting the edge for infinite looping

					std::vector<std::string> tmpVector = std::vector<std::string>(currentRoute.begin(),currentRoute.end());
					for (unsigned int i = 0; i < tmpVector.size()-1; ++i) {
						for (unsigned int j = i+1; j < tmpVector.size(); ++j) {
							if (tmpVector[i]==tmpVector[j]){
								std::string tmp1 = tmpVector[i];
								std::string tmp2 = tmpVector[j];
								edgeForLooping = tmp1;
								break;
							}
						}
					}

				// rerouting and setting boolean
				traci->commandChangeRouteById(currentRouteId);
				reroutedToLoopRoute = true;
				}
			} else {
				if (traci->getRoadId() == edgeForLooping){
					traci->commandChangeRouteById(currentRouteId);
				}
			}
		}
	}
}

void simpleVEHICLEpOpp::sendMessage() {
	//This is just to track the current Vehicle Position Tracking,
	//and just Something that I need at this moment in my repport.
	Coord vehPos = traci->getCurrentPosition();
	Coord vehiclePosSumo;//actual XY vehicle position

	//step 1
	vehiclePosSumo.x = traci->getManager()->omnet2traci(vehPos).x;
	vehiclePosSumo.y = traci->getManager()->omnet2traci(vehPos).y;


//	double axeY= maxY - vehPos.y;
	double axeY = vehiclePosSumo.y;

//	int myAddr = isNetwAddrInit ? netwAddr : myId;

	MYDEBUG <<"logs, VEH," <<simTime() <<",From," << myApplAddr() << "," << traci->getExternalId()  <<",tx," <<  junctionID << ", messageSequence, " <<  messageSequence << ", messageSequenceVPA, " << messageSequenceVPA << ","<< vehPos.x <<","<<  axeY<<"," <<endl;
	//MYDEBUG <<"logs, backoff,tx,"<< currentSector <<","<< traci->getExternalId()<< ","  << simTime() << "," <<endl;



	char numstr[6]; // Numbered Message
	sprintf(numstr, "%d+%d", messageSequenceVPA,messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	char* result = numstr; //concatenate in VPAiD,messageSequence

	//Sending message
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
//	sendWSM(prepareWSM(result, dataLengthBits, channel, dataPriority, 0,2));
	if (isNetwAddrInit){
		sendWSM(prepareWSM(result, dataLengthBits, channel, dataPriority, 0,multiFunctions::cantorPairingFunc(netwAddr,nbrMsgSent)));
	}else{
		opp_error("netw address not yet initialized");
	}


}


// I'm Overriding this function from inherit class 'cause I got to add the wsm->setKind() to 30=BROADCAST_VPA_WMS or 40=BROADCAST_VEH_WMS
// I've to add a different KIND message when sending from vehicle or from VPA.
WaveShortMessage*  simpleVEHICLEpOpp::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, unsigned long serial) {
	WaveShortMessage* wsm =		new WaveShortMessage(name.c_str());
	wsm->addBitLength(headerLength);
	wsm->addBitLength(lengthBits);

	switch (channel) {
		case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
	}
	if (withDtnMsg){
		wsm->setKind(DTN_TEST_MODE);
	}else {
		wsm->setKind(BROADCAST_VEH_WMS);//30=BROADCAST_VPA_WMS, 40=BROADCAST_VEH_WMS
	}

	wsm->setPsid(0);
	wsm->setPriority(priority);
	wsm->setWsmVersion(1);
	wsm->setTimestamp(simTime());
	wsm->setSenderAddress(netwAddr);
	wsm->setRecipientAddress(rcvId);
//	wsm->setSenderPos(curPosition);
	wsm->setSenderPos(traci->getCurrentPosition());
	wsm->setSerial(serial);

	return wsm;
}

//THE FINISH FUNCTION.
void simpleVEHICLEpOpp::finish() {
	/* Ok, Al final voy a presentar una serie de stadisticas de
	 * de cada vehiculo que me permitan hacer màs rapido las
	 * courbas sin tener que procesar demasiado los logs.
	 * COOL! Crear de datos finales por cada vehiculo al termino de la simulacion.
	 */
//	DBG << "logs, finish," <<  traci->getExternalId() <<","<< myApplAddr() <<",vehTimeIn,"<< vehTimeIn <<",vehTimeOut,"<< vehTimeOut << ",RX,"<< vehRx <<","<<std::endl;

	DtnApplLayer::finish();

    simulation.getSystemModule()->unsubscribe(vehStopped, this);
    simulation.getSystemModule()->unsubscribe(connected, this);

	for (std::map<std::string,int>::iterator it = nbrSentByDist.begin(); it != nbrSentByDist.end(); it++) {
		std::string tmp = "# nbrBndlSent_"+it->first;
		recordScalar(tmp.c_str(), it->second);
	}
}

////////////////// TESTING AREA /////////////    ////////////////// TESTING AREA /////////////

//EVENEMENT OF PAQUET JETé qui rapport la couche Mac80211p.cc
void simpleVEHICLEpOpp::entra(int droppedMac80211p, int currentCW, int myNicId) {

	//EV << "HELLO, received from Mac80211p, Dropped:"<< droppedMac80211p <<",currentCW,"<< currentCW <<",myNicId," << myNicId<<",VPAiD," << myApplAddr() <<endl;

	//here I write in a nic/NicId.txt the dropped packet.
	//Concatenating the NicId as the name of file. I'm using this value in order to trace the file.
	std::stringstream sstm;
	sstm << "nic/" <<myNicId << ".txt";
	std::string result = sstm.str();
	//write down file.
	const char* fileNameConst= result.c_str(); //convert from string to const char*
	std::ofstream outputFile;
	outputFile.open(fileNameConst);
	outputFile << simTime() << " " <<  droppedMac80211p <<" " << currentCW;// <<endl; //este puto endl me mueve todo!
	outputFile.close();
}


//Read the nic/files.txt in order to retrieve the node information and update internal messages.
void simpleVEHICLEpOpp::readNicFiles() {
	// Ok, Here the objective is to obtain an average per second of the dropped messages and current CW.
	// I've to open the corresponding file, retrieve the information and maintain an average
	// dropped packets every second.
	// First define the file named with the MacID. Is always the next number of myId.
	// Next read the file and update my average file.

	//Retrieve nic/file.txt based on the NicId.
	//EV << " items:"<< myApplAddr() <<", TIME= "<< simTime() <<endl;

	int myFile= myId + 2;//I noticed that myId is 2 numbers behind of the NicId. Based in this remark I can identify the file.
	std::stringstream sstm;
	sstm << "nic/" << myFile << ".txt"; //concatenate.
	std::string result = sstm.str();//pass from sstm to string
	EV << "File items:"<< myApplAddr() << ",file," << result <<endl;
	const char* fileNameConst= result.c_str(); //convert from string to const char*
	std::ifstream readFile(fileNameConst); //finally target my file.

	//read all the file by line.
	std::string line;
	if (readFile.is_open())
	{
		while ( readFile.good() )
		{
			getline(readFile,line);
			//EV <<"printing file: "<<":" << line << endl;
		}
		readFile.close();
	}
	else EV << "File items: Unable to open file: "<< fileNameConst <<endl;

	readFile.close();

	//HERE: Splitting data, Damn it's so long the string handling with C++.
	std::string item[4];//create an array to deposit the strings.
	std::istringstream iss(line);
	int i=0;//just a counter.
	std::string sub;
	do
	{
		iss >> sub; //do the split
		//EV << "Substring: " << sub <<endl;
		item[i]= sub;
		i++;
	} while (iss);

	//Finally the retrieved data from the files.
	//sample: 20.005281054597:3:7:
	//EV << "Final items: " << item[0] <<":"<<  item[1]<< ":"<< item[2] <<":" <<endl;
	double mac80211psimTimer= strtod(item[0].c_str(), NULL); //string to double
	int mac80211pDrops= atoi(item[1].c_str()); //string to int
	int mac80211pCW= atoi(item[2].c_str()); //string to int
	EV << "File items:"<< myApplAddr() <<","<< mac80211psimTimer <<":"<<  mac80211pDrops << ":"<< mac80211pCW <<":" <<endl;

	//NOW: here I define the dropped messages in the last second
	// This will contain dropped messages in the last second.
	// In case no dropped messages in the last second I use 0

	//Check if there is update of dropped message 1 second ago.
	EV << "Current items:" << myApplAddr() <<",droppedMessages," << droppedMessages <<",currentCW,"<< currentCW <<endl; //<<",counterDroppedMessages,"<<  counterDroppedMessages <<endl;

	if ( (simTime() - mac80211psimTimer) <= 6 ) {
		//EV << "operation items:" << myApplAddr() <<","<< mac80211pDrops<< "-"<<  counterDroppedMessages<<endl;
		droppedMessages= mac80211pDrops;// - counterDroppedMessages;//number of dropped messages in the last second.
		//counterDroppedMessages= mac80211pDrops;//update my own counter
	}
	else
	{
		droppedMessages= 0; //if not updates in the last second, I set my droopped messages = 0
	}

	//if (mac80211pCW){ currentCW= mac80211pCW; }  else  {	currentCW= CWMIN_11P/2;    }
	currentCW= CWMIN_11P/2;

	EV << "Updated items:" << myApplAddr() <<",droppedMessages," << droppedMessages <<",currentCW,"<< currentCW <<endl;// <<",counterDroppedMessages,"<<  counterDroppedMessages <<endl;
}



void simpleVEHICLEpOpp::onBeacon(WaveShortMessage* wsm) {
}
void simpleVEHICLEpOpp::onData(WaveShortMessage* wsm) {
}

int simpleVEHICLEpOpp::vpaDestAddr(int sectorID)
{
	int vpaDestAddr = -2;
	cModule *systemModule = this->getParentModule();
	while (systemModule->getParentModule() !=NULL){
		systemModule = systemModule->getParentModule();
	}
	int numberVPA = 0;
	if (systemModule->hasPar("numeroNodes")){
		numberVPA = systemModule->par("numeroNodes");
	}
	if (!systemModule->hasPar("numeroNodes") || (0 >= numberVPA)){
		opp_error("Impossible to determine number of VPA");
	}
	if ((sectorID < 0) || (sectorID > numberVPA)){
		opp_error("error : sectorID out of bound");
	}

	cModule *vpa = systemModule->getSubmodule("VPA", sectorID);
	if (vpa!=NULL){
		DtnNetwLayer *netw = FindModule<DtnNetwLayer*>::findSubModule(vpa);
		if (netw!=NULL){
			vpaDestAddr = netw->getMyNetwAddr();
		}
	}

	if (vpaDestAddr == -2){
		opp_error("Impossible to determine VPA address");
	}

	return vpaDestAddr;
}

int simpleVEHICLEpOpp::randomVPADestAddr()
{
	int vpaDestAddr = -2;
	cModule *systemModule = this->getParentModule();
	while (systemModule->getParentModule() !=NULL){
		systemModule = systemModule->getParentModule();
	}
	int numberVPA = 0;
	if (systemModule->hasPar("numeroNodes")){
		numberVPA = systemModule->par("numeroNodes");
	}
	if (!systemModule->hasPar("numeroNodes") || (0 <= numberVPA)){
		opp_error("Impossible to determine number of VPA");
	}

	do {
		vpaDestAddr = intuniform(0,numberVPA-1);
		cModule *vpa = systemModule->getSubmodule("VPA", vpaDestAddr);
		if (vpa!=NULL){
			DtnNetwLayer *netw = FindModule<DtnNetwLayer*>::findSubModule(vpa);
			if (netw!=NULL){
				vpaDestAddr = netw->getMyNetwAddr();
			}
		}
	}while (vpaDestAddr == netwAddr);

	if (vpaDestAddr == -2){
		opp_error("Impossible to determine VPA address");
	}

	return vpaDestAddr;
}
/*
    // TRACI GET SUMO DATA
	MYDEBUG << "DEBUG getRoadId: " << traci->getRoadId() <<endl;
    MYDEBUG << "DEBUG cmdGetVehiclelanePosition: " << traci->cmdGetVehiclelanePosition()  <<endl;
    std::string laneId = traci->getRoadId() + "_0";
    MYDEBUG << "DEBUG cmdGetLaneLength: " << laneId << " length: " << traci->cmdGetLaneLength(laneId)  <<endl;
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
    //On change le parcours d'un voiture..
    if ( traci->getExternalId() == "veh.1" and simTime() < 35) {
	    MYDEBUG << "DEBUG commandChangeRoute: " <<endl;
	   // traci->commandSetSpeed(2); //set speed to 2m/s
	   // traci->commandStopNode("B1",80.0,0,5,5);
	    traci->commandChangeRoute("B2",30.0); //set TT
	    traci->commandChangeRoute("C2",30.0);
	    traci->commandChangeRoute("A3",30.0);
	    traci->commandChangeRoute("B3",30.0);
    }


	//I thinl this draw something interesting in the TKenv GUI.
	findHost()->getDisplayString().updateWith("r=16,red");


*/

void simpleVEHICLEpOpp::receiveSignal(cComponent *source, simsignal_t signalID, long  l)
{
	Enter_Method_Silent("simpleVEHICLEpOpp::receiveSignal()");
	if (signalID == connected){
		if (l == 1){
			if (isEquiped && withDtnMsg && ((getDataSrcFromStrategy(strategy)=="ALL") || (getDataSrcFromStrategy(strategy)=="VEH"))){
				// we have to send bundles
				bool haveToSent = false;
				double tmp = 0;
				if (sendingStrategy == SectorEntry){
					whatSectorIm();
					if (currentSector != oldSector){haveToSent = true;};
				}else if(sendingStrategy == Periodic){
					tmp = (dtnMsgSynchronized)? 0: uniform(0,periodForMsg);
					haveToSent = true;
				}

				if (haveToSent){
					scheduleAt(simTime() + tmp, dtnMsg);
					nbrMsgSent++;
				}
			}
		}
	}
}

void simpleVEHICLEpOpp::receiveSignal(cComponent *source, simsignal_t signalID, double d)
{
	Enter_Method_Silent("simpleVEHICLEpOpp::receiveSignal()");
	if (signalID == vehStopped){
		remainingMsgByStep = nbrMsgByStep;
		std::ostringstream iss;
		iss << targetPos - ceil(d);
		dist = iss.str();
	}
}