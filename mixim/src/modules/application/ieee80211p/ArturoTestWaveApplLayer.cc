/*
 * This program is based on the TestWaveApplLayer.cc
 * I'm using this to test the collision, CW in 802.11p OMNETpp implementation.
 */


#include "ArturoTestWaveApplLayer.h"
#include <fstream> // for writing files.
#include <sstream> //ARturo splitting strings
#include <iostream>
#include <string>



#define	DO_THINGS_EVERY_SECOND 50 //This is to active the scheduler of doing things every N time.


Define_Module(ArturoTestWaveApplLayer);

void ArturoTestWaveApplLayer::initialize(int stage) {
	/* Ok, here I've copied the stage 0 from BaseWaveApplLayer::initialize
	 * and call from the upper class:  BaseApplLayer::initialize
	 * With the purpose of handling here all the initialization stuffs.
	 */
	//BaseWaveApplLayer::initialize(stage); //I've copied the init stage 0
	BaseApplLayer::initialize(stage); //I'm calling the mother class

	if (stage==0) {
		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
		            getParentModule()->getParentModule());
		assert(myMac);

		myId = getId();

		headerLength = par("headerLength").longValue();

		sendBeacons = par("sendBeacons").boolValue();
		beaconLengthBits = par("beaconLengthBits").longValue();
		beaconPriority = par("beaconPriority").longValue();

		sendData = par("sendData").boolValue();
		dataLengthBits = par("dataLengthBits").longValue();
		dataOnSch = par("dataOnSch").boolValue();
		dataPriority = par("dataPriority").longValue();

		//ARTURO my passed values:
	    T = hasPar("timeT") ? par("timeT").doubleValue(): 5;
	    appCW = hasPar("appCW") ? par("appCW").boolValue(): false; //wheter I'll use or not the APP WC.
	    appMinCW = hasPar("appMinCW") ? par("appMinCW").doubleValue(): 100; //my APPS CW
	    EV <<"logs, T," << T <<",appCW,"<<appCW <<",appMinCW,"<< appMinCW <<endl;

	    messageSequence = 0; //First message sequence.
		CW= 0; //Contention Window value.
		droppedMessages= 0;//received Mac80211p dropped messages
		currentCW= 0; //received Mac80211p CW
		counterDroppedMessages=0; //Total count Mac80211p dropped message.
		Tupdate= 1; //periodic time to obtain the dropped packets.
		//When starting erase files from the nic directory
		//if (myId == 67 )//Hack, execute once when the myId number 64 is created.
		//however this number changes when changing the number of participant nodes.
			system("rm nic/*.txt");// caution this is executed every time an object is created.

		sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
		doEverySecond = new cMessage("beacon evt", DO_THINGS_EVERY_SECOND);

		scheduleAt(simTime() + 0, sendBeaconEvt);
		scheduleAt(simTime() + 0, doEverySecond);
	}

}

//Overriding handleSelfMsg from inherited class: BaseWaveApplLayer
//in order to manipulate here the self-messages
void ArturoTestWaveApplLayer::handleSelfMsg(cMessage* msg) {
	switch (msg->getKind()) {
	EV <<"logs, self MESSAGE RECEIVED  message kind: "<<msg->getKind()<<endl;

		case SEND_BEACON_EVT: { //Arturo, OK here send the packet and schedule the next.

	        sendVPABroadcast(messageSequence++); //Sending periodic VPA Beacon (BROADCAST_MESSAGE) with broadcast counter.

        	EV << "logs, sending, droppedMessages "<< droppedMessages << ", myCW , "<< CW <<endl;


	        //NO APPS_CW
	        if (!appCW){
	        	scheduleAt(simTime() + T, sendBeaconEvt);
	        }
	        else //using APPS_CW
	        {
	        	/* first equation. SIMPLE  */
	        	//int dynamicST= appMinCW * droppedMessages;
	        	/* Second equation. (Nblost/Tupdate) (CWmax-CWmin) + CWmin  */
	        	//int dynamicST= (droppedMessages/1) * (CWMAX_11P - currentCW) + currentCW;
	        	/* Third equation. (Nblost/Tupdate) (CWmax-CWmin)  */
	        	int dynamicST= (droppedMessages/Tupdate) * (CWMAX_11P - currentCW);
	        	CW= intuniform(0, dynamicST)* SLOTLENGTH_11P; //DYNAMIC computing my APPS CW_min based on the 802.11p slot time.
	        	//CW= intuniform(0, appMinCW)* SLOTLENGTH_11P; //STATIC computing my APPS CW_min based on the 802.11p slot time.
	        	EV << "logs, veh, " << myApplAddr() << ", my apps CW, "<< CW <<",dynamicST," << dynamicST <<endl;
	        	scheduleAt(simTime() + T + CW, sendBeaconEvt);
	        }

			break;
		}
		case DO_THINGS_EVERY_SECOND: { //Arturo, OK here send the packet and schedule the next.
			//Re-schecule self-message.
			scheduleAt(simTime() + 1, doEverySecond);
		    //DBG << "logs, my, " <<  myId <<","<< myMac << "," << mySCH << "," << myApplAddr() <<endl;

		    /* Ok, Here the objective is to obtain an average per second of the dropped messages and current CW.
		     * I've to open the corresponding file, retrieve the information and maintain an average
		     * dropped packets every second.
		     * First define the file named with the MacID. Is always the next number of myId.
		     * Next read the file and update my average file.
		     */

		    //Retrieve nic/file.txt based on the NicId.
		    EV << " items:"<< myApplAddr() <<", TIME= "<< simTime() <<endl;

		    int myFile= myId - 1;//I noticed that myId is one number ahead of the NicId. Based in this remark I can identify the file.
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
			  else EV << "Unable to open file: "<< fileNameConst <<endl;

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

			    /*NOW: here I define the dropped messages in the last second
			     * This will contain dropped messages in the last second.
			     * In case no dropped messages in the last second I use 0
			     */
			    //Check if there is update of dropped message 1 second ago.
			    EV << "Current items:" << myApplAddr() <<",droppedMessages," << droppedMessages <<",counterDroppedMessages,"<<  counterDroppedMessages <<endl;

			    if ( (simTime() - mac80211psimTimer) < 1 ) {
			    	EV << "operation items:" << myApplAddr() <<","<< mac80211pDrops<< "-"<<  counterDroppedMessages<<endl;
			    	droppedMessages= mac80211pDrops - counterDroppedMessages;//number of dropped messages in the last second.
			    	counterDroppedMessages= mac80211pDrops;//update my own counter
			    }
			    else
			    {
			    	droppedMessages= 0; //if not updates in the last second, I set my droopped messages = 0
			    }

			    currentCW= mac80211pCW;//This value is same.

			    EV << "Updated items:" << myApplAddr() <<",droppedMessages," << droppedMessages <<",counterDroppedMessages,"<<  counterDroppedMessages <<endl;

			    //IMPORTANTE! BORRAR LOS nic/files antes de cualquier simulacion. Hacerlo automaticamente.. DONE!

/* NO FUNCIONA JALAR DATOS DE MAC parece hay un problema de cuando se crean los objetos y se destruyen...
			Mac80211p* myON; //To access variable from Mac80211p.cc
			ASSERT(myON);
		 	 myON->myNicId;
		    DBG << "logs, myNicId, " <<  myON->myNicId <<",macAddress,"<< myON->myMacAddress <<endl;

	//Mac80211p myON; //To access variable from Mac80211p.cc
    //Trying to get the backoff data from Mac80211p
    long uno= myON->statsNumBackoffs ;
    long unouno= myON->statsReceivedPackets ;
    long dos= myON->statsReceivedBroadcasts ;
    long tres= myON->statsSentPackets ;
    long cuatro= myON->statsLostPackets ;
    long cinco= myON->statsDroppedPackets ;
    long seis= myON->statsNumTooLittleTime ;
    double siete= myON->statsTotalBackoffDuration ;
    int ocho= myON->currentNumBackoffs ;
    unsigned int nueve= myON->txAttempts ;
    //long diez= myON.statsDroppedPackets ;
    //int once= myON.myMacAddress ;
    //double trece= myON.txPower;
    long cator= myON->currentCW;
    //double quince= myON.bitrate;
    int dieci = myON->myNicId;
    int dieciocho= myON->arturostatsDropPackets;


    DBG << "logs, statsNumBackoffs, " <<  uno <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, statsReceivedPackets, " <<  unouno <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, statsReceivedBroadcasts, " <<  dos <<",id,"<< myApplAddr() <<",time,"<< simTime()  <<endl;
	DBG << "logs, statsSentPackets, " <<  tres <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, statsLostPackets, " <<  cuatro  <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, statsDroppedPackets, " <<  cinco <<",id,"<< myApplAddr() <<",time,"<< simTime()<<endl;
	DBG << "logs, statsNumTooLittleTime, " <<  seis <<",id,"<< myApplAddr() <<",time,"<< simTime()<<endl;
	DBG << "logs, statsTotalBackoffDuration, " <<  siete <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, currentNumBackoffs, " <<  ocho <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, txAttempts, " <<  nueve <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	//DBG << "logs, statsDroppedPackets, " <<  diez <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	//DBG << "logs, myMacAddress, " <<  once <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	//DBG << "logs, txPower, " <<  trece <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, currentCW, " <<  cator <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	//DBG << "logs, bitrate, " <<  quince <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	//DBG << "logs, myNicID, " <<  dieci <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs, arturostatsDropPackets, " <<  dieciocho <<",id,"<< myApplAddr() <<",time,"<< simTime() <<endl;
	DBG << "logs,minCW, " <<  CWMIN_11P <<", maxCW,"<< CWMAX_11P <<endl;


// JUST DEBUGGIN TYPES OF PACKET MESSAGES:
			DBG << "myId== " << myId <<std::endl;
			DBG << "msg->getArrivalModule() " << msg->getArrivalModule() <<std::endl;
			DBG << "msg->getClassName() " << msg->getClassName() <<std::endl;
			DBG << "msg->getContextPointer() " << msg->getContextPointer() <<std::endl;
			DBG << "msg->getDescriptor() " << msg->getDescriptor() <<std::endl;
			DBG << "msg->getDisplayString() " << msg->getDisplayString() <<std::endl;
			DBG << "msg->getFullName() " << msg->getFullName() <<std::endl;
			DBG << "msg->getId() " << msg->getId() <<std::endl;
			DBG << "msg->getKind() " << msg->getKind() <<std::endl;
			DBG << "msg->getName() " << msg->getName() <<std::endl;
			DBG << "msg->getNamePooling() " << msg->getNamePooling() <<std::endl;
*/
		}
			break;
		default: {
			if (msg)
				DBG << "Apps: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
			break;
		}
	}
}


void ArturoTestWaveApplLayer::handleLowerMsg(cMessage* msg) {
	Mac80211Pkt* mac = dynamic_cast<Mac80211Pkt*>(msg);
	ASSERT(mac);
	WaveShortMessage*  wsm =  dynamic_cast<WaveShortMessage*>(mac->decapsulate());
	if (wsm != NULL) {
		EV << "logs,"<< simTime() <<",me RX," << myApplAddr() <<", VPA Received a PACKET from TX,"<<wsm->getName() <<"," << endl;
	}
	delete(msg);
}


void ArturoTestWaveApplLayer::sendVPABroadcast(int messageSequenceTEST)
{
	char numstr[5]; // Identify the message origin.
	//sprintf(numstr, "from host[%d]", myApplAddr()); // convert from * to STRING.
	sprintf(numstr, "%d", myApplAddr()); // convert from * to STRING.
	char* result = numstr;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	sendWSM(prepareWSM(result, dataLengthBits, channel, dataPriority, 0,2));
	//sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));//Original.

}

//EVENEMENT OF PAQUET JETé qui rapport la couche Mac80211p.cc
void ArturoTestWaveApplLayer::entra(long droppedMac80211p, int currentCW, int myNicId) {

	EV << "HELLO, received from Mac80211p, Dropped:"<< droppedMac80211p <<",currentCW,"<< currentCW <<",myNicId," << myNicId<<",VPAiD," << myApplAddr() <<endl;

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


/**************************** NOT USING NEXT FUNCTIONS ***************************************/
void ArturoTestWaveApplLayer::onBeacon(WaveShortMessage* wsm) {
	//receivedBeacons++;
	//DBG << "logs, Received beacon priority  " << wsm->getPriority() << " at " << simTime() << ",from "<< wsm->getName() <<std::endl;
	//int senderId = wsm->getSenderAddress();
/*	if (sendData) {
		t_channel channel = dataOnSch ? type_SCH : type_CCH;
		sendWSM(prepareWSM("data", dataLengthBits, channel, dataPriority, senderId,2));
	}
*/}

void ArturoTestWaveApplLayer::onData(WaveShortMessage* wsm) {
	/*int recipientId = wsm->getRecipientAddress();
	if (recipientId == myId) {
		DBG  << "logs, Received data priority  " << wsm->getPriority() << " at " << simTime() << std::endl;
		receivedData++;
	}
*/}

ArturoTestWaveApplLayer::~ArturoTestWaveApplLayer() {
}


