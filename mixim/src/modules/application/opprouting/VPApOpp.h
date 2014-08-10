//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef VPApOpp_H_
#define VPApOpp_H_

#include "BaseModule.h"
#include <BaseWaveApplLayer.h>
#include <NetwPkt_m.h>

#ifndef DBG
#define DBG EV
#endif
//#define DBG std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath()

class VPApOpp  : 	public BaseWaveApplLayer {
	public:

		virtual ~VPApOpp();

		virtual void initialize(int stage);
	    /**ARTURO  @brief Message kinds used by this layer */
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
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);

	protected:
		uint32_t receivedBeacons;
		uint32_t receivedData;
		  int messageSequence;//sequence of the VPA message number sent
		  double CW; //Contention Window value.
		  double T; //Timer to send periodic VPA WMS Broadcast. NOTE:This value must be de the same for VPA/vehicles beacons.
		  void sendVPApBroadcast(int messageSequenced);//send numerated broadcast.
		  void handleLowerMsg(cMessage* msg);
		  void handleSelfMsg(cMessage* msg);
		  //Adding my own prepareWSM messages.
		  virtual WaveShortMessage* prepareWSM(std::string name, int dataLengthBits, t_channel channel, int priority, int rcvId, int serial=0);

};

#endif /* VPApOpp_H_ */
