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

#ifndef __MIXIM_DTNAPPLLAYER_H_
#define __MIXIM_DTNAPPLLAYER_H_

#include <omnetpp.h>

#include "BaseWaveApplLayer.h"


/**
 * TODO - Generated class
 */
class DtnApplLayer : public BaseWaveApplLayer	{
	public:
		~DtnApplLayer();
		virtual void initialize(int stage);
		virtual void finish();

	protected:
//		/** @brief handle messages from below */
//		virtual void handleLowerMsg(cMessage* msg);
//		/** @brief handle self messages */
//		virtual void handleSelfMsg(cMessage* msg);
//
//		virtual WaveShortMessage* prepareWSM(std::string name, int dataLengthBits, t_channel channel, int priority, int rcvId, int serial=0);
//		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
//
//		virtual void handlePositionUpdate(cObject* obj);

		/*
		 * bool variable for enabling dtnTestMode
		 */
		bool dtnTestMode;
		cMessage *dtnTestMsg;
		int dtnTestCycle;
		int dtnTestMaxTime;
		int dtnTestMinTime;
		bool dtnSynchronized;

		int nbrBundleSent;
		int nbrBundleReceived;
		int nbrUniqueBundleReceived;

		bool sectorMode;
		int oldSector;

		bool anyVPA;

		bool isEquiped;
};

#endif
