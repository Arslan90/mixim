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

#include "PyGraphServerManager.h"
#include <unistd.h>
#include "stdio.h"
#include "clistener.h"
#include "DtnApplLayer.h"
#include "math.h"
//#include <sys/socket.h> // Needed for the socket functions
//#include <netdb.h>      // Needed for the socket functions

Define_Module(PyGraphServerManager);

void PyGraphServerManager::initialize(int stage)
{
    // TODO - Generated method body
	if (stage ==0){
		debug = par("debug").boolValue();
		host = par("host").stdstringValue();
		port = par("port");
		communication_range = par("range").doubleValue();
		autoBuild = par("autoBuild").boolValue();
		edgeFile = par("edgeFile").stdstringValue();
		nodeFile = par("nodeFile").stdstringValue();
		vpaFile = par("vpaFile").stdstringValue();
		margin = par("margin");
		waitingTime = par("waitingTime");

		sentSignalId = registerSignal("sentBndl");
		simulation.getSystemModule()->subscribe(sentSignalId, this);

		receiveSignalId = registerSignal("receivedBndl");
		simulation.getSystemModule()->subscribe(receiveSignalId, this);

		receiveL3SignalId = registerSignal("receivedL3Bndl");
		simulation.getSystemModule()->subscribe(receiveL3SignalId, this);

		sentL3SignalId = registerSignal("sentL3Bndl");
		simulation.getSystemModule()->subscribe(sentL3SignalId, this);

		sentBitsLengthSignalId = registerSignal("sentBitsLength");
		simulation.getSystemModule()->subscribe(sentBitsLengthSignalId, this);

		helloCtrlBitsLengthId = registerSignal("helloCtrlBitsLength");
		simulation.getSystemModule()->subscribe(helloCtrlBitsLengthId, this);

		otherCtrlBitsLengthId = registerSignal("otherCtrlBitsLength");
		simulation.getSystemModule()->subscribe(otherCtrlBitsLengthId, this);

		tSentSignalId = registerSignal("uniqBndlSent");
		tReceiveSignalId = registerSignal("uniqBndlReceived");
		tReceiveL3SignalId = registerSignal("l3BndlReceived");
		tSentL3SignalId = registerSignal("l3BndlSent");

		dR = registerSignal("deliveryRatio");
		oT = registerSignal("totalOverhead");
		rCtrlData = registerSignal("ratioCtrlData");
		rRecvSent = registerSignal("ratioRecvSent");

		sizeHelloCtrl = registerSignal("TotalSizeHelloCtrl");
		sizeOtherCtrl = registerSignal("TotalSizeOtherCtrl");
		sizeCtrl = registerSignal("TotalSizeCtrl");
		sizeData = registerSignal("TotalSizeData");

		emit(dR, 0.0);
		emit(oT, 0.0);

		emit(tSentSignalId, 0.0);
		emit(tReceiveSignalId, 0.0);
		emit(tReceiveL3SignalId, 0.0);
		emit(tSentL3SignalId, 0.0);

		emit(rCtrlData, 0.0);
		emit(rRecvSent, 0.0);

		emit(sizeHelloCtrl, 0.0);
        emit(sizeOtherCtrl, 0.0);
        emit(sizeCtrl, 0.0);
        emit(sizeData, 0.0);

		nbrBundleSent = 0;
		nbrL3BundleSent = 0;
		nbrUniqueBundleReceived = 0;
		nbrL3BundleReceived = 0;

		helloCtrlSentSizeKbits = 0;
		otherCtrlSentSizeKbits = 0;
		dataSentSizeKbits = 0;

		t_sizeHC_SB = registerSignal("t_sizeHC_SB");
		t_sizeHC_SA = registerSignal("t_sizeHC_SA");
		t_sizeHC_CL = registerSignal("t_sizeHC_CL");
		t_sizeHC_RCC= registerSignal("t_sizeHC_RCC");

		sizeHC_SB_Kbits  = 0;
		sizeHC_SA_Kbits  = 0;
		sizeHC_CL_Kbits  = 0;
		sizeHC_RCC_Kbits = 0;

		emit(t_sizeHC_SB , 0);
		emit(t_sizeHC_SA , 0);
		emit(t_sizeHC_CL , 0);
		emit(t_sizeHC_RCC, 0);

		t_sizeOC_SB = registerSignal("t_sizeOC_SB");
		t_sizeOC_SA = registerSignal("t_sizeOC_SA");
		t_sizeOC_CL = registerSignal("t_sizeOC_CL");
		t_sizeOC_RCC= registerSignal("t_sizeOC_RCC");

		sizeOC_SB_Kbits  = 0;
		sizeOC_SA_Kbits  = 0;
		sizeOC_CL_Kbits  = 0;
		sizeOC_RCC_Kbits = 0;

		emit(t_sizeOC_SB , 0);
		emit(t_sizeOC_SA , 0);
		emit(t_sizeOC_CL , 0);
		emit(t_sizeOC_RCC, 0);

		simulation.getSystemModule()->subscribe("ackLifeTime", this);

		totalAckLifeTime = 0.0;
		counterForAckLifeTime = 0.0;

		stats_AckLifeTime = registerSignal("stats_AckLifeTime");
		emit(stats_AckLifeTime,0.0);

		simulation.getSystemModule()->subscribe("custodyLifeTime", this);

		totalCustodyLifeTime = 0.0;
		counterForCustodyLifeTime = 0.0;

		stats_CustodyLifeTime = registerSignal("stats_CustodyLifeTime");
		emit(stats_CustodyLifeTime,0.0);

		collectStatOnly = par("collectStatOnly").boolValue();

		updateInterval = par("updateInterval").doubleValue();
		if (updateInterval <= 0){
			opp_error("PyGraphServerManager::initialize - Update Interval should be a strict positive double");
		}
		updateMsg = new cMessage("updateMsg");
		double currentTime = simTime().dbl();
		double scheduleTime = ceil(currentTime/updateInterval)*updateInterval;
		if (currentTime == scheduleTime){
			scheduleTime+=updateInterval;
		}
		scheduleAt(scheduleTime, updateMsg);

		initializeConnection();
	}
}

std::string PyGraphServerManager::sendRequestToPyServer(std::string buf)
{
	std::string rep;
	if (collectStatOnly){
		// nothing to do right now
		opp_error("PyGraphServerManager::sendRequestToPyServer() - calling this function when in collection statistics only mode");
	}else{

		std::string HOST = "127.0.0.1";
		int PORT = 19999;
		int MAX_BUFFER = 4096;

		int rc, index = 0, limit = MAX_BUFFER;
		char buffer[MAX_BUFFER+1];
		if (buf.size() > limit){
			opp_warning("Buffer size smaller than the request to send");
		}

		int returnCode = 0;
		// Sending request
		sprintf( buffer, "%s", buf.c_str() );
		returnCode = ::send(connectionFd, buffer, strlen(buffer), 0 );
		if (returnCode == -1){
			std::stringstream ss;
			ss << errno;
			std::string errorMsg = "(SEND) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
			opp_error(errorMsg.c_str());
		}

		// Receiving request
		memset(&buffer[0], 0, sizeof(buffer));
		returnCode = recv(connectionFd, buffer, MAX_BUFFER, 0);
		if (returnCode == -1){
			std::stringstream ss;
			ss << errno;
			std::string errorMsg = "(RECV) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
			opp_error(errorMsg.c_str());
		}
		rep = std::string(buffer);

	}
	return rep;
}

void PyGraphServerManager::handleMessage(cMessage *msg)
{
    // TODO - Generated method body
	if (strcmp(msg->getName(),"updateMsg") == 0){
		/**
		 * Traditional metrics
		 */
		if (nbrBundleSent > 0){
			emit(dR, (double) nbrUniqueBundleReceived / (double) nbrBundleSent);
			emit(oT, (double) nbrL3BundleReceived / (double) nbrBundleSent);
		}else{
			emit(dR, 0.0);
			emit(oT, 0.0);
		}

		emit(tSentSignalId, nbrBundleSent);
		emit(tSentL3SignalId, nbrL3BundleSent);
		emit(tReceiveSignalId, nbrUniqueBundleReceived);
		emit(tReceiveL3SignalId, nbrL3BundleReceived);

		/**
		 * My old metrics
		 */
		emit(sizeHelloCtrl, helloCtrlSentSizeKbits);
		emit(sizeOtherCtrl, otherCtrlSentSizeKbits);
		emit(sizeData, dataSentSizeKbits);
		emit(sizeCtrl, helloCtrlSentSizeKbits+otherCtrlSentSizeKbits);

		if (dataSentSizeKbits != 0){
			emit(rCtrlData, (double) (helloCtrlSentSizeKbits+otherCtrlSentSizeKbits)/ (double) dataSentSizeKbits);
		}else {
			emit(rCtrlData, 0);
		}

		int dataLength = DtnApplLayer::getDataLengthBitsAsStatic();
		if ((dataSentSizeKbits != 0) || (sizeCtrl != 0)){
			double uniqDataReceived = (double) (nbrUniqueBundleReceived * dataLength) / 1024;
			double ratio = uniqDataReceived / (double) (dataSentSizeKbits + sizeCtrl);
			emit(rRecvSent, ratio);
		}

		/**
		 * My new metrics
		 */
		emit(t_sizeHC_SB, sizeHC_SB_Kbits);
		emit(t_sizeHC_SA, sizeHC_SA_Kbits);
		emit(t_sizeHC_CL, sizeHC_CL_Kbits);
		emit(t_sizeHC_RCC, sizeHC_RCC_Kbits);

		emit(t_sizeOC_SB, sizeOC_SB_Kbits);
		emit(t_sizeOC_SA, sizeOC_SA_Kbits);
		emit(t_sizeOC_CL, sizeOC_CL_Kbits);
		emit(t_sizeOC_RCC, sizeOC_RCC_Kbits);

		/**
		 * Metrics related to ACKs life time
		 */
		updateStatsForAckLifeTime(false);
		updateStatsForCustodyLifeTime(false);

		double currentTime = simTime().dbl();
		double scheduleTime = ceil(currentTime/updateInterval)*updateInterval;
		if (currentTime == scheduleTime){
			scheduleTime+=updateInterval;
		}
		scheduleAt(scheduleTime, updateMsg);
	}
}

void PyGraphServerManager::receiveSignal(cComponent *source, simsignal_t signalID, long l)
{
	Enter_Method_Silent();
	if (strcmp(getSignalName(signalID),"sentBndl") == 0){
		nbrBundleSent++;
	}
	if (strcmp(getSignalName(signalID),"receivedBndl") == 0){
		nbrUniqueBundleReceived++;
		if ((dataSentSizeKbits != 0) || (sizeCtrl != 0)){
			double uniqDataReceived = (double) (nbrUniqueBundleReceived * DtnApplLayer::getDataLengthBitsAsStatic()) / 1024;
			double ratio = uniqDataReceived / (double) (dataSentSizeKbits + sizeCtrl);
		}
	}
	if (strcmp(getSignalName(signalID),"receivedL3Bndl") == 0){
		nbrL3BundleReceived++;
	}

	if (strcmp(getSignalName(signalID),"sentL3Bndl") == 0){
		nbrL3BundleSent++;
	}
}

void PyGraphServerManager::receiveSignal(cComponent *source, simsignal_t signalID, const char *s)
{
	Enter_Method_Silent();
	char* signalStr = strdup(s);

	if (strcmp(getSignalName(signalID),"sentBitsLength") == 0){
		std::string hCtrlSizeAsStr = "", oCtrlSizeAsStr = "", nbrEncapDataAsStr = "";
		long helloCtrlSize = 0, otherCtrlSize = 0, nbrEncapData = 0;

		hCtrlSizeAsStr = std::string(strtok(signalStr,":"));
		if (hCtrlSizeAsStr != ""){ helloCtrlSize = strtol(hCtrlSizeAsStr.c_str(),NULL,10);}

		oCtrlSizeAsStr = std::string(strtok(NULL,":"));
		if (oCtrlSizeAsStr != ""){ otherCtrlSize = strtol(oCtrlSizeAsStr.c_str(),NULL,10);}

		nbrEncapDataAsStr = std::string(strtok(NULL,":"));
		if (nbrEncapDataAsStr != ""){ nbrEncapData = strtol(nbrEncapDataAsStr.c_str(),NULL,10);}

		if (helloCtrlSize != 0){
			helloCtrlSentSizeKbits += ((double)helloCtrlSize / 1024);
		}

		if (otherCtrlSize != 0){
			otherCtrlSentSizeKbits += ((double)otherCtrlSize / 1024);
		}

		int dataLength = DtnApplLayer::getDataLengthBitsAsStatic();
		if (nbrEncapData != 0){
			dataSentSizeKbits += ((double)nbrEncapData * dataLength / 1024);
		}

		if ((dataSentSizeKbits != 0) || (sizeCtrl != 0)){
			double uniqDataReceived = (double) (nbrUniqueBundleReceived * dataLength) / 1024;
			double ratio = uniqDataReceived / (double) (dataSentSizeKbits + sizeCtrl);
		}
	}

	if (strcmp(getSignalName(signalID),"helloCtrlBitsLength") == 0){
		char* sizeAsStr = strtok(signalStr,":");
		std::vector<long> sizeAsLong (4,0);
		int index = 0;
		while (sizeAsStr != NULL){
			sizeAsLong[index] = strtol(sizeAsStr,NULL,10);
			switch (index) {
				case 0:	{
						sizeHC_SB_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				case 1:	{
						sizeHC_SA_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				case 2:	{
						sizeHC_CL_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				case 3:	{
						sizeHC_RCC_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				default:{
					opp_error("PyGraphServerManager::receiveSignal - No more than 4 entries for Hello Ctrl Msg Lengths");
				}break;
			}
			index++;
			sizeAsStr = strtok(NULL,":");
		}
	}

	if (strcmp(getSignalName(signalID),"otherCtrlBitsLength") == 0){
		char* sizeAsStr = strtok(signalStr,":");
		std::vector<long> sizeAsLong (4,0);
		int index = 0;
		while (sizeAsStr != NULL){
			sizeAsLong[index] = strtol(sizeAsStr,NULL,10);
			switch (index) {
				case 0:	{
						sizeOC_SB_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				case 1:	{
						sizeOC_SA_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				case 2:	{
						sizeOC_CL_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				case 3:	{
						sizeOC_RCC_Kbits += ((double)sizeAsLong[index] / 1024);
				}break;
				default:{
					opp_error("PyGraphServerManager::receiveSignal - No more than 4 entries for Other Ctrl Msg Lengths");
				}break;
			}
			index++;
			sizeAsStr = strtok(NULL,":");
		}
	}

	if (strcmp(getSignalName(signalID),"ackLifeTime") == 0){
		char* lifeTimeAsStr = strtok(signalStr,":");
		unsigned long serial = strtol(lifeTimeAsStr,NULL,10);

		lifeTimeAsStr = strtok(NULL,":");
		double creationTime = strtod(lifeTimeAsStr,NULL);

		lifeTimeAsStr = strtok(NULL,":");
		double expireTime = strtod(lifeTimeAsStr,NULL);

		std::map<unsigned long, std::pair<double,double> >::iterator it = ackLifeTime.find(serial);
		if (it == ackLifeTime.end()){
			ackLifeTime.insert(std::make_pair(serial,std::make_pair(creationTime,expireTime)));
		}else{
			if (it->second.first == -1){
				opp_error("PyGraphServerManager::receiveSignal - CreationTime of ACKs cannot be negative");
			}
			if (it->second.second < expireTime){
				ackLifeTime[it->first] = std::make_pair(it->second.first, expireTime);
			}
		}
	}

	if (strcmp(getSignalName(signalID),"custodyLifeTime") == 0){
		char* lifeTimeAsStr = strtok(signalStr,":");
		unsigned long serial = strtol(lifeTimeAsStr,NULL,10);

		lifeTimeAsStr = strtok(NULL,":");
		double creationTime = strtod(lifeTimeAsStr,NULL);

		lifeTimeAsStr = strtok(NULL,":");
		double expireTime = strtod(lifeTimeAsStr,NULL);

		std::map<unsigned long, std::pair<double,double> >::iterator it = custodyLifeTime.find(serial);
		if (it == custodyLifeTime.end()){
			custodyLifeTime.insert(std::make_pair(serial,std::make_pair(creationTime,expireTime)));
		}else{
			if (it->second.first == -1){
				opp_error("PyGraphServerManager::receiveSignal - CreationTime of Custodies cannot be negative");
			}
			if (it->second.second < expireTime){
				custodyLifeTime[it->first] = std::make_pair(it->second.first, expireTime);
			}
		}
	}

	free((void*) signalStr);
}

void PyGraphServerManager::finish(){
	// Resenting signals before closing finishing and clearing all data
	if (nbrBundleSent > 0){
		emit(dR, (double) nbrUniqueBundleReceived / (double) nbrBundleSent);
		emit(oT, (double) nbrL3BundleReceived / (double) nbrBundleSent);
	}else{
		emit(dR, 0.0);
		emit(oT, 0.0);
	}

	if (dataSentSizeKbits != 0){
		emit(rCtrlData, (double) (helloCtrlSentSizeKbits+otherCtrlSentSizeKbits)/ (double) dataSentSizeKbits);
	}else {
		emit(rCtrlData, 0);
	}

	if ((dataSentSizeKbits != 0) || (sizeCtrl != 0)){
		double uniqDataReceived = (double) (nbrUniqueBundleReceived * DtnApplLayer::getDataLengthBitsAsStatic()) / 1024;
		double ratio = uniqDataReceived / (double) (dataSentSizeKbits + sizeCtrl);
		emit(rRecvSent, ratio);
	}

	emit(sizeHelloCtrl, helloCtrlSentSizeKbits);
	emit(sizeOtherCtrl, otherCtrlSentSizeKbits);
	emit(sizeCtrl, helloCtrlSentSizeKbits+otherCtrlSentSizeKbits);
	emit(sizeData, dataSentSizeKbits);

	emit(tSentSignalId, nbrBundleSent);
	emit(tSentL3SignalId, nbrL3BundleSent);
	emit(tReceiveSignalId, nbrUniqueBundleReceived);
	emit(tReceiveL3SignalId, nbrL3BundleReceived);

	recordScalar("# Total Bundle Sent", nbrBundleSent);
	recordScalar("# Total Bundle Sent by L3", nbrL3BundleSent);
	recordScalar("# Total Bundle Received by L3", nbrL3BundleReceived);
	recordScalar("# Total Unique Bundle Received by VPAs", nbrUniqueBundleReceived);

	emit(t_sizeHC_SB, sizeHC_SB_Kbits);
	emit(t_sizeHC_SA, sizeHC_SA_Kbits);
	emit(t_sizeHC_CL, sizeHC_CL_Kbits);
	emit(t_sizeHC_RCC, sizeHC_RCC_Kbits);

	emit(t_sizeOC_SB, sizeOC_SB_Kbits);
	emit(t_sizeOC_SA, sizeOC_SA_Kbits);
	emit(t_sizeOC_CL, sizeOC_CL_Kbits);
	emit(t_sizeOC_RCC, sizeOC_RCC_Kbits);

	updateStatsForAckLifeTime(true);
	updateStatsForCustodyLifeTime(true);

	finishConnection();
}

void PyGraphServerManager::initializeConnection()
{
	if (collectStatOnly){
		// nothing to do right now
	}else{
		std::string HOST = "127.0.0.1";
		int PORT = 19999;

		memset(&servAddr, 0, sizeof(servAddr));
		servAddr.sin_family = AF_INET;
		servAddr.sin_addr.s_addr = inet_addr(HOST.c_str());
		servAddr.sin_port = htons(PORT);


		memset(&localAddr, 0, sizeof(localAddr));
		localAddr.sin_family = AF_INET;
		localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		localAddr.sin_port = htons(0);

		// Create socket
		connectionFd = socket(AF_INET, SOCK_STREAM, 0);

		int returnCode = 0;

		// Connect to Server
		returnCode = connect(connectionFd,(struct sockaddr *)&servAddr, sizeof(servAddr));
		if (returnCode == -1){
			std::stringstream ss;
			ss << errno;
			std::string errorMsg = "(CONNECT) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
			opp_error(errorMsg.c_str());
		}
	}
}

void PyGraphServerManager::finishConnection()
{
	if (collectStatOnly){
		// nothing to do right now
	}else{
		//	std::string command = "pkill -f ProdServer.py";
		//	system(command.c_str());

			int MAX_BUFFER = 4096;

			int rc, index = 0, limit = MAX_BUFFER;
			char buffer[MAX_BUFFER+1];

			int returnCode = 0;
			// Sending a close request
			sprintf( buffer, "%s", "CLOSE");
			returnCode = ::send(connectionFd, buffer, strlen(buffer), 0 );
			if (returnCode == -1){
				std::stringstream ss;
				ss << errno;
				std::string errorMsg = "(SEND) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
				opp_error(errorMsg.c_str());
			}

			// Closing connection to Server
			close(connectionFd);
	}
}

void PyGraphServerManager::updateStatsForAckLifeTime(bool recordAllEntries)
{
	std::set<unsigned long> entriesToDelete;
	for (std::map<unsigned long, std::pair<double,double> >::iterator it = ackLifeTime.begin(); it != ackLifeTime.end(); it++){
		unsigned long serial =it->first;
		double creationTime = it->second.first;
		double expireTime = it->second.second;

		if (creationTime < 0){
			opp_error("PyGraphServerManager::updateStatsForAckLifeTime - CreationTime of ACKs cannot be negative");
		}

		if (creationTime > expireTime){
			opp_error("PyGraphServerManager::updateStatsForAckLifeTime - CreationTime of ACKs cannot be higher than ExpireTime");
		}

		if ((recordAllEntries) || ((!recordAllEntries) && (expireTime < simTime().dbl()))){
			totalAckLifeTime += expireTime - creationTime;
			counterForAckLifeTime++;
			entriesToDelete.insert(serial);
		}
	}

	for (std::set<unsigned long>::iterator it2 = entriesToDelete.begin(); it2 != entriesToDelete.end(); it2++){
		ackLifeTime.erase(*it2);
	}

	emit(stats_AckLifeTime,totalAckLifeTime/counterForAckLifeTime);
}

void PyGraphServerManager::updateStatsForCustodyLifeTime(bool recordAllEntries)
{
	std::set<unsigned long> entriesToDelete;
	for (std::map<unsigned long, std::pair<double,double> >::iterator it = custodyLifeTime.begin(); it != custodyLifeTime.end(); it++){
		unsigned long serial =it->first;
		double creationTime = it->second.first;
		double expireTime = it->second.second;

		if (creationTime < 0){
			opp_error("PyGraphServerManager::updateStatsForCustodyLifeTime - CreationTime of Custodies cannot be negative");
		}

		if (creationTime > expireTime){
			opp_error("PyGraphServerManager::updateStatsForCustodyLifeTime - CreationTime of Custodies cannot be higher than ExpireTime");
		}

		if ((recordAllEntries) || ((!recordAllEntries) && (expireTime < simTime().dbl()))){
			totalCustodyLifeTime += expireTime - creationTime;
			counterForCustodyLifeTime++;
			entriesToDelete.insert(serial);
		}
	}

	for (std::set<unsigned long>::iterator it2 = entriesToDelete.begin(); it2 != entriesToDelete.end(); it2++){
		custodyLifeTime.erase(*it2);
	}

	emit(stats_CustodyLifeTime,totalCustodyLifeTime/counterForCustodyLifeTime);
}


