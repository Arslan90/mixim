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

#ifndef __MIXIM_PYGRAPHSERVERMANAGER_H_
#define __MIXIM_PYGRAPHSERVERMANAGER_H_

#include <omnetpp.h>
#include "stdlib.h"
#include "iostream"
#include <unistd.h>
#include "errno.h"
#include <sys/socket.h>
#include <arpa/inet.h>

/**
 * TODO - Generated class
 */
class PyGraphServerManager : public cSimpleModule, cListener
{
  public:
	std::string sendRequestToPyServer(std::string buf);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, long l);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, const char *s);

  protected:
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    virtual void initializeConnection();

    virtual void finishConnection();

    bool debug;
    std::string host;
    int port;
    double communication_range;
    bool autoBuild;
    std::string edgeFile;
    std::string nodeFile;
    std::string vpaFile;
    int margin;
    int waitingTime;
	struct sockaddr_in servAddr, localAddr;
	int connectionFd;

	// Different stats for Bundle Sending/Reception
	long nbrBundleSent;
	long nbrL3BundleSent;
	long nbrUniqueBundleReceived;
	long nbrL3BundleReceived;

	double helloCtrlSentSizeKbits;
	double otherCtrlSentSizeKbits;
	double dataSentSizeKbits;

	simsignal_t sentSignalId, receiveSignalId, receiveL3SignalId , sentL3SignalId ,tSentSignalId, tReceiveSignalId, tReceiveL3SignalId, tSentL3SignalId;

	simsignal_t sentBitsLengthSignalId;

	simsignal_t helloCtrlBitsLengthId;

	simsignal_t otherCtrlBitsLengthId;

	simsignal_t dR;

	simsignal_t oT;

	simsignal_t rCtrlData;

	simsignal_t rRecvSent;

	simsignal_t sizeHelloCtrl;

	simsignal_t sizeOtherCtrl;

	simsignal_t sizeCtrl;

	simsignal_t sizeData;

	/**
	 * Size Hello Ctrl Messages: SB (Stored Bundles)
	 * SA (Stored ACKs), CL (Custody Lists), RCC (Replica and Custody Confirmations)
	 */

	simsignal_t t_sizeHC_SB;
	simsignal_t t_sizeHC_SA;
	simsignal_t t_sizeHC_CL;
	simsignal_t t_sizeHC_RCC;

	double sizeHC_SB_Kbits;
	double sizeHC_SA_Kbits;
	double sizeHC_CL_Kbits;
	double sizeHC_RCC_Kbits;


	/**
	 * Size Other Ctrl Messages: SB (Stored Bundles)
	 * SA (Stored ACKs), CL (Custody Lists), RCC (Replica and Custody Confirmations)
	 */

	simsignal_t t_sizeOC_SB;
	simsignal_t t_sizeOC_SA;
	simsignal_t t_sizeOC_CL;
	simsignal_t t_sizeOC_RCC;

	double sizeOC_SB_Kbits;
	double sizeOC_SA_Kbits;
	double sizeOC_CL_Kbits;
	double sizeOC_RCC_Kbits;

	cMessage* updateMsg;

	double updateInterval;

	bool collectStatOnly;
};

#endif
