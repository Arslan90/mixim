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

#ifndef ArturoTestWaveApplLayer_H_
#define ArturoTestWaveApplLayer_H_

#include "BaseModule.h"
#include <BaseWaveApplLayer.h>
#include <Mac80211p.h> //Arturo

#ifndef DBG
#define DBG EV
#endif


class ArturoTestWaveApplLayer  : 	public BaseWaveApplLayer {
	public:

		virtual ~ArturoTestWaveApplLayer();

		virtual void initialize(int stage);
		virtual void entra(long droppedMac80211p, int currentCW, int myNicId);//Arturo function to get data from Mac80211p.cc
		//Mac80211p testing; //To access variable from Mac80211p.cc


	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);

		//Arturo My Variables & functions.
		void handleSelfMsg(cMessage* msg);//overriding the handleSelfMsg instead to use from BaseWaveApplLayer.
		void handleLowerMsg(cMessage* msg); //overriding the handleLowerMsg instead to use from BaseWaveApplLayer.
		void sendVPABroadcast(int messageSequence); //own function to send message
		int messageSequence;//sequence of the VPA message number sent
		double CW; //Contention Window value.
		double T; //Timer to send periodic VPA WMS Broadcast. NOTE:This value must be de the same for VPA/vehicles beacons.
		bool appCW;//Using or not appCW
		double appMinCW; // my APPS CW
		long droppedMessages;//average Mac80211p dropped message. In order to figure out how to calculate my STmax.
		long counterDroppedMessages;//Total count Mac80211p dropped message. In order to figure out how to calculate my STmax.
		int currentCW; //Mac80211p currentCW
		int Tupdate; //periodic time to obtain the dropped packets.

	protected:
		uint32_t receivedBeacons;
		uint32_t receivedData;
		cMessage* doEverySecond; //Arturo, Doing this every N time.

};

#endif /* ArturoTestWaveApplLayer_H_ */
