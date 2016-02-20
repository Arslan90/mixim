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
		const char* scenario = par("scenarioType").stringValue();
		if (strcmp(scenario,"Free")==0){
			scenarioModel = Free;
		}else if (strcmp(scenario,"Sector")==0){
			scenarioModel = Sector;
		}else {
			opp_error("Unrecognized Scenario Model Type (Free, Sector, etc..)");
		}

		// Parameters related to Sector Mode
		oldSector = -1;
		currentSector = -1;
		if (scenarioModel == Sector){
			rowSectorGrid = par("rowSectorGrid");
			colSectorGrid = par("colSectorGrid");
			sectorSizeX = par("sectorSizeX").doubleValue();
			sectorSizeY = par("sectorSizeY").doubleValue();
			useNegativeValues = par("useNegativeValues").boolValue();
			sectorOffsetX = par("sectorOffsetX").doubleValue();
			sectorOffsetY = par("sectorOffsetY").doubleValue();
		}

		withDtnMsg = par("withDtnMsg").boolValue();
		std::string strategy (par("dataGeneration").stringValue());
		if (strategy == "ALL2ALL"){
			genStrategy = ALL2ALL;
		}else if (strategy == "ALL2VEH"){
			genStrategy = ALL2VEH;
		}else if (strategy == "ALL2VPA"){
			genStrategy = ALL2VPA;
		}else if (strategy == "VPA2ALL"){
			genStrategy = VPA2ALL;
		}else if (strategy == "VPA2VEH"){
			genStrategy = VPA2VEH;
		}else if (strategy == "VPA2VPA"){
			genStrategy = VPA2VPA;
		}else if (strategy == "VEH2ALL"){
			genStrategy = VEH2ALL;
		}else if (strategy == "VEH2VEH"){
			genStrategy = VEH2VEH;
		}else if (strategy == "VEH2VPA"){
			genStrategy = VEH2VPA;
		}else {
			opp_error("Unrecognized Scenario Model Type (Free, Sector, etc..)");
		}

		std::string dataForwarding (par("dataForwarding").stringValue());


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




