/* -*- mode:c++ -*- ********************************************************
 * My file modified:        TestingBurstApplLayer.cc
 * based on:				BurstApplLayer.cc
 *
 *ARTURO, Ok This file is to test the broadcasting problem with component  Mac80211
 * After this brief review integrate this communication module to our scenario but ASAP!!!
 * Move your Ass men! Just finish this part of the thesis DAMN it!!
 *
 * Thu Apr 19 12:37:57 CEST 2012
 **************************************************************************/


#include <stdlib.h> //for itoa
#include "TestingBurstApplLayer.h"
#include "ApplPkt_m.h"
#include "NetwControlInfo.h" //This is to implement local sending packages


using std::endl;

Define_Module(TestingBurstApplLayer);

// do some initialization
void TestingBurstApplLayer::initialize(int stage)
{
	//Run the upper class and set a VPA sel message timer
	TestApplLayer::initialize(stage); //calling from the parent class TestApplLayer.cc

    T = hasPar("timeT") ? par("timeT").doubleValue(): 5;
    appCW = hasPar("appCW") ? par("appCW").boolValue(): false;
    appMinCW = hasPar("appMinCW") ? par("appMinCW").doubleValue(): 0.01;
    EV <<"logs, T," << T <<",appCW,"<<appCW <<",appMinCW,"<< appMinCW <<endl;

	//burstSize = par("burstSize"); //number of messages to send
    //bSendReply = par("burstReply"); //if you want to send replies.
    messageSequence = 0; //First message sequence.
	CW= 0; //Contention Window value.
}


// SELF MESSAGES (TEMPORIZERS)
void TestingBurstApplLayer::handleSelfMsg(cMessage *msg)
{	//Switch case for Types of self messages received.
    switch(msg->getKind())
    {
    case SEND_BROADCAST_TIMER:
    	EV <<"logs, self MESSAGE RECEIVED (sending VPA beacon) message kind: "<<msg->getKind()<<endl;

    	/* OK, Here I send a burst instead of a single broadcast in order to demonstrate that the backoff
    	 * mechanism will wake up once traffic is active.
    	 * Otherwise if medium free time > DIFS the  backoff mechanism is not utilized.
    	*/
        /*for(int i=1; i<=2; i++) {
        	sendVPABroadcast(messageSequence++); //Sending periodic VPA Beacon (BROADCAST_MESSAGE) with broadcast counter.
        }*/

        sendVPABroadcast(messageSequence++); //Sending periodic VPA Beacon (BROADCAST_MESSAGE) with broadcast counter.

        //NO APPS_CW
        if (!appCW){
        	scheduleAt(simTime() + T, delayTimer);
        }
        else //using APPS_CW
        {
        CW= intuniform(0, 10)* appMinCW;//computing CW
        EV << "logs, veh, " << myApplAddr() << ", using CW, "<< CW  <<endl;
        scheduleAt(simTime() + T + CW, delayTimer);
        }

    	break;
    default:
        EV <<"logs, logs, Unkown selfmessage! -> delete, kind, "<<msg->getKind()<<endl;
        break;
    }
}


//PACKET RECEIVED FROM THE NETWORK.
void TestingBurstApplLayer::handleLowerMsg( cMessage* msg )
{
	ApplPkt *m  = static_cast<ApplPkt *>(msg);
	EV << "logs,"<< simTime() <<",me RX," << myApplAddr() <<", VPA Received a PACKET from TX,"<<m->getSrcAddr()<<"," << endl;
	delete msg;//just delete message do nothing

	//TestApplLayer::handleLowerMsg(msg); //Example of handling messages.
}


void TestingBurstApplLayer::sendVPABroadcast(int messageSequenceTEST)
{
	//This paragraph is to transmit a message (VAPiD+sequenceMessage).
	char numstr[5]; // Numbered Message
	//sprintf(numstr, "%d+%d", myApplAddr(),messageSequence); // convert INT to STRING. VPAId+SequenceNumber
	sprintf(numstr, "from host=%d", myApplAddr()); // convert INT to STRING. VPAId+SequenceNumber
	char* result = numstr; //concatenate in VPAiD,messageSequence

    ApplPkt *pkt = new ApplPkt(result, BROADCAST_VPA_WMS);
    pkt->setDestAddr(LAddress::L3BROADCAST);
    // we use the host modules getIndex() as a appl address
    pkt->setSrcAddr( myApplAddr() );
    pkt->setBitLength(headerLength);
    // set the control info to tell the network layer the layer 3 address;
    NetwControlInfo::setControlInfo(pkt, LAddress::L3BROADCAST );
    sendDown( pkt );

    EV << "logs, logs,Sending VPA broadcast packet!" << endl;
	EV <<"logs, stats, VPA, " << myApplAddr() <<", "<< simTime() << ", tx, " <<  result  << endl;
}
