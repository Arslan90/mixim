/* -*- mode:c++ -*- ********************************************************
 * My file:        VPA.cc  based on the file BurstApplLayer.cc
 *
 * OK , this is my own description: This file governs the VPA ACTIVITY.
 * This is the 2nd. version.
 * Here I'll re-organize and clean the code.
 * Add the Vehicle traffic-probe functionality
 * Add the WMS exchange.
 *
 * Last UPDATE: Fri Apr 20 18:39:55 CEST 2012
 ***********************************************************/


#include <stdlib.h> //for itoa
#include "VPA.h"
#include "ApplPkt_m.h"
#include "NetwControlInfo.h" //This is to implement local sending packages


using std::endl;

Define_Module(VPA);

// do some initialization
void VPA::initialize(int stage)
{
	//TestApplLayer::initialize(stage); //Bypassing initiaization of parent class
    BaseApplLayer::initialize(stage);// Using upper parent class instead

    if(stage == 0) {
    	//Receive the T value
        T = hasPar("timeT") ? par("timeT").doubleValue(): 5;
        EV <<"logs, T time: " << T <<endl;
        messageSequence = 0; //First message sequence.
    	CW= 0; //Contention Window value.

    	//Schedule the next first VPA TX
        delayTimer = new cMessage( "delay-timer", SEND_BROADCAST_TIMER );
        scheduleAt(simTime() + 0, delayTimer);
    }
}


// SELF MESSAGES (TEMPORIZERS)
void VPA::handleSelfMsg(cMessage *msg)
{	//Switch case for Types of self messages received.
    switch(msg->getKind())
    {
    case SEND_BROADCAST_TIMER:
        EV <<"logs, VPA self MESSAGE" <<endl;

        sendVPABroadcast(messageSequence++); //Sending periodic VPA Beacon (BROADCAST_MESSAGE) with broadcast counter.

		//Reschedule the self-message
        //delayTimer = new cMessage( "delay-timer", SEND_BROADCAST_TIMER );
        scheduleAt(simTime() + T, delayTimer);
    	break;
    default:
        EV <<"logs, Unkown selfmessage kind: "<<msg->getKind()<<endl;
        break;
    }
}



//PACKET RECEIVED FROM THE NETWORK.
void VPA::handleLowerMsg( cMessage* msg )
{
	ApplPkt *m  = static_cast<ApplPkt *>(msg);
	EV << "logs, Receiving packet from host["<<m->getSrcAddr()<<"] " << endl;
	delete msg;//just delete message do nothing

	//TestApplLayer::handleLowerMsg(msg); //Example of handling messages.
}


void VPA::sendVPABroadcast(int messageSequenceTEST)
{
	//This paragraph is to transmit a message (VAPiD+sequenceMessage).
	char numstr[5]; // Numbered Message
	sprintf(numstr, "%d+%d", myApplAddr(),messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	char* result = numstr; //concatenate in VPAiD,messageSequence

	//CONSTRUCT THE WHOLE PACKET.
    ApplPkt *pkt = new ApplPkt(result, BROADCAST_VPA_WMS);
    pkt->setDestAddr(LAddress::L3BROADCAST);
    // we use the host modules getIndex() as a appl address
    pkt->setSrcAddr( myApplAddr() );
    pkt->setBitLength(headerLength);
    // set the control info to tell the network layer the layer 3 address;
    NetwControlInfo::setControlInfo(pkt, LAddress::L3BROADCAST );
    sendDown( pkt );

    EV << "logs, Sending VPA broadcast packet!" << endl;
	//EV <<"logs, stats, VPA, " << myApplAddr() <<", "<< simTime() << ", tx, " <<  result  << endl;
}
