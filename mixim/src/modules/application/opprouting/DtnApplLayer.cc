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

#include "DtnApplLayer.h"

Define_Module(DtnApplLayer);

void DtnApplLayer::initialize(int stage)
{
	BaseWaveApplLayer::initialize(stage);

	if (stage==0) {

	}
}



void DtnApplLayer::finish()
{
}



//void DtnApplLayer::handleLowerMsg(cMessage *msg)
//{
//}
//
//
//
//void DtnApplLayer::handleSelfMsg(cMessage *msg)
//{
//}
//
//
//
//WaveShortMessage *DtnApplLayer::prepareWSM(std::string name, int dataLengthBits, t_channel channel, int priority, int rcvId, int serial)
//{
//}
//
//
//
//void DtnApplLayer::sendWSM(WaveShortMessage *wsm)
//{
//}
//
//
//
//void DtnApplLayer::handlePositionUpdate(cObject *obj)
//{
//}



void DtnApplLayer::onBeacon(WaveShortMessage *wsm)
{
}



std::string DtnApplLayer::getDataSrcFromStrategy(std::string currentStrategy)
{
}

std::string DtnApplLayer::getDataDestFromStrategy(std::string currentStrategy)
{
}

void DtnApplLayer::onData(WaveShortMessage *wsm)
{
}



DtnApplLayer::~DtnApplLayer()
{
}




