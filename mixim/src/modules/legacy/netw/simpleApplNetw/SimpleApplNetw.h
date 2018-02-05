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

#ifndef __MIXIM_SIMPLEAPPLNETW_H_
#define __MIXIM_SIMPLEAPPLNETW_H_

#include <omnetpp.h>

#include <BaseNetwLayer.h>
#include <simpleTraCIMobility.h>
#include "simpleListener.h"
#include <map>
#include <string>

/**
 * TODO - Generated class
 */
class SimpleApplNetw : public BaseNetwLayer //, simpleListener
{
  protected:
	simpleTraCIMobility* mobility;

	bool sentMsg;
	int periodForMsg;
	int nbrMsgByStep;
	int msgLength;

	int remainingMsgByStep;

	long nbrReceived;
	long nbrSent;

	cMessage *msg;

	simsignal_t vehStopped;

	bool isSuscribed;

	std::map<std::string,int> nbrReceivedByDist;

	double stopPos;
	double stopDuration;
	double stopDistStep;
	double releasePos;

	bool with127M;
	double targetPos;

  public:
    virtual void initialize(int stage);
    virtual void finish();
  protected:
    virtual void handleSelfMsg(cMessage *msg);
    virtual void handleLowerMsg(cMessage *msg);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, long l);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, double d);


};

#endif
