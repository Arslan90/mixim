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

		sentBitsLengthSignalId = registerSignal("sentBitsLength");
		simulation.getSystemModule()->subscribe(sentBitsLengthSignalId, this);

		tSentSignalId = registerSignal("TotalSentBndl");
		tReceiveSignalId = registerSignal("TotalReceivedBndl");
		tReceiveL3SignalId = registerSignal("TotalL3ReceivedBndl");

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
		emit(rCtrlData, 0.0);
		emit(rRecvSent, 0.0);

		emit(sizeHelloCtrl, 0.0);
        emit(sizeOtherCtrl, 0.0);
        emit(sizeCtrl, 0.0);
        emit(sizeData, 0.0);

		nbrBundleSent = 0;
		nbrBundleReceived = 0;
		nbrUniqueBundleReceived = 0;
		nbrL3BundleReceived = 0;

		helloCtrlSentSizeKbits = 0;
		otherCtrlSentSizeKbits = 0;
		dataSentSizeKbits = 0;

		collectStatOnly = par("collectStatOnly").boolValue();

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

}

void PyGraphServerManager::receiveSignal(cComponent *source, simsignal_t signalID, long l)
{
	Enter_Method_Silent();
	if (strcmp(getSignalName(signalID),"sentBndl") == 0){
		nbrBundleSent++;
	}
	if (strcmp(getSignalName(signalID),"receivedBndl") == 0){
		nbrUniqueBundleReceived++;
		emit(dR, (double) nbrUniqueBundleReceived / (double) nbrBundleSent);
		if ((dataSentSizeKbits != 0) || (sizeCtrl != 0)){
			double uniqDataReceived = (double) (nbrUniqueBundleReceived * DtnApplLayer::getDataLengthBitsAsStatic()) / 1024;
			double ratio = uniqDataReceived / (double) (dataSentSizeKbits + sizeCtrl);
			emit(rRecvSent, ratio);
		}
	}
	if (strcmp(getSignalName(signalID),"receivedL3Bndl") == 0){
		nbrL3BundleReceived++;
		emit(oT, (double) nbrL3BundleReceived / (double) nbrBundleSent);
	}
}

void PyGraphServerManager::receiveSignal(cComponent *source, simsignal_t signalID, const char *s)
{
	Enter_Method_Silent();
	if (strcmp(getSignalName(signalID),"sentBitsLength") == 0){
		std::string hCtrlSizeAsStr = "", oCtrlSizeAsStr = "", nbrEncapDataAsStr = "";
		long helloCtrlSize = 0, otherCtrlSize = 0, nbrEncapData = 0;

		char* signalStr = strdup(s);

		hCtrlSizeAsStr = std::string(strtok(signalStr,":"));
		if (hCtrlSizeAsStr != ""){ helloCtrlSize = strtol(hCtrlSizeAsStr.c_str(),NULL,10);}

		oCtrlSizeAsStr = std::string(strtok(NULL,":"));
		if (oCtrlSizeAsStr != ""){ otherCtrlSize = strtol(oCtrlSizeAsStr.c_str(),NULL,10);}

		nbrEncapDataAsStr = std::string(strtok(NULL,":"));
		if (nbrEncapDataAsStr != ""){ nbrEncapData = strtol(nbrEncapDataAsStr.c_str(),NULL,10);}

		if (helloCtrlSize != 0){
			helloCtrlSentSizeKbits += ((double)helloCtrlSize / 1024);
			emit(sizeHelloCtrl, helloCtrlSentSizeKbits);
		}

		if (otherCtrlSize != 0){
			otherCtrlSentSizeKbits += ((double)otherCtrlSize / 1024);
			emit(sizeOtherCtrl, otherCtrlSentSizeKbits);
		}

		int dataLength = DtnApplLayer::getDataLengthBitsAsStatic();
		if (nbrEncapData != 0){
			dataSentSizeKbits += ((double)nbrEncapData * dataLength / 1024);
			emit(sizeData, dataSentSizeKbits);
		}

		if ((helloCtrlSize != 0) || (otherCtrlSize != 0)){
			emit(sizeCtrl, helloCtrlSentSizeKbits+otherCtrlSentSizeKbits);
		}

		if ((helloCtrlSize != 0) || (otherCtrlSize != 0) || (nbrEncapData != 0)){
			if (dataSentSizeKbits != 0){
				emit(rCtrlData, (double) (helloCtrlSentSizeKbits+otherCtrlSentSizeKbits)/ (double) dataSentSizeKbits);
			}
		}

		if ((dataSentSizeKbits != 0) || (sizeCtrl != 0)){
			double uniqDataReceived = (double) (nbrUniqueBundleReceived * dataLength) / 1024;
			double ratio = uniqDataReceived / (double) (dataSentSizeKbits + sizeCtrl);
			emit(rRecvSent, ratio);
		}
	}
}

void PyGraphServerManager::finish(){
	// Resenting signals before closing finishing and clearing all data
	emit(dR, (double) nbrUniqueBundleReceived / (double) nbrBundleSent);
	emit(oT, (double) nbrL3BundleReceived / (double) nbrBundleSent);
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

	recordScalar("# Total Bundle Sent", nbrBundleSent);
	recordScalar("# Total Bundle Received by L3", nbrL3BundleReceived);
	recordScalar("# Total Unique Bundle Received by VPAs", nbrUniqueBundleReceived);
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

//		returnCode = bind(connectionFd,
//		  (struct sockaddr *) &localAddr, sizeof(localAddr));
//		if (returnCode == -1){
//			std::stringstream ss;
//			ss << errno;
//			std::string errorMsg = "(BIND) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
//			opp_error(errorMsg.c_str());
//		}

		// Connect to Server
		returnCode = connect(connectionFd,(struct sockaddr *)&servAddr, sizeof(servAddr));
		if (returnCode == -1){
			std::stringstream ss;
			ss << errno;
			std::string errorMsg = "(CONNECT) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
			opp_error(errorMsg.c_str());
		}


//		std::string command = "pkill -f ProdServer.py";
//		system(command.c_str());

//		char cwd[1024];
//		if (getcwd(cwd, sizeof(cwd)) != NULL){
//			std::cout <<"Current working dir:"<< cwd << std::endl;
//		} else {
//			opp_error("getCWD() command failed");
//		}
//
//		command = "python "+std::string(cwd)+"/ProdServer.py";
//		popen(command.c_str(), "r");
//		sleep(waitingTime);
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




