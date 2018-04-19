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
#include "FindModule.h"
#include "DtnNetwLayer.h"

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
			if ((rowSectorGrid <= 0) || (colSectorGrid <= 0)){
				opp_error("Sector grid values invalid");
			}
			sectorSizeX = par("sectorSizeX").doubleValue();
			sectorSizeY = par("sectorSizeY").doubleValue();
			if ((sectorSizeX <= 0) || (sectorSizeY <= 0)){
				opp_error("Sector sizes values invalid");
			}
			useNegativeValues = par("useNegativeValues").boolValue();
			sectorOffsetX = par("sectorOffsetX").doubleValue();
			sectorOffsetY = par("sectorOffsetY").doubleValue();

			if ((!useNegativeValues) && ((sectorOffsetX < 0) || (sectorOffsetY < 0))){
				opp_error("Sector offset values invalid");
			}
		}

		strategy = std::string(par("dataGeneration").stringValue());
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
			opp_error("Unrecognized data Traffic Generator mode");
		}

		std::string dataForwarding (par("dataForwarding").stringValue());
		if (dataForwarding == "ALL"){
			fwdStrategy = ALL;
		}else if (dataForwarding == "VPA"){
			fwdStrategy = VPA;
		}else if (dataForwarding == "VEH"){
			fwdStrategy = VEH;
		}else {
			opp_error("Unrecognized data Traffic Forwarder mode");
		}

		withDtnMsg = par("withDtnMsg").boolValue();
		dtnMsg = new cMessage( "Dtn Msg", DTN_TEST_MODE);

		withOtherMsg = par("withOtherMsg").boolValue();

		std::string sendingType (par("dtnMsgSending").stringValue());
		if (sendingType == "SectorEntry"){
			sendingStrategy = SectorEntry;
		}else if (sendingType == "Periodic"){
			sendingStrategy = Periodic;
		}else {
			opp_error("Unrecognized data Traffic Sending Type");
		}

		dtnMsgPeriod = par("dtnMsgPeriod").doubleValue();
		dtnMsgMaxTime = par("dtnMsgMaxTime").doubleValue();
		dtnMsgMinTime = par("dtnMsgMinTime").doubleValue();
		dtnMsgSynchronized = par("dtnMsgSynchronized").boolValue();

		if ((dtnMsgMinTime < 0)){
			opp_error("dtnMsgMinTime is incorrect");
		}

		if ((dtnMsgMinTime >= dtnMsgMaxTime)){
			opp_error("dtnMsg Min & Max time are incorrect");
		}

		if (!((dtnMsgMinTime <= dtnMsgPeriod)&&(dtnMsgPeriod <= dtnMsgMaxTime))){
			opp_error("dtnMsgPeriod out of bound (Min & Max time)");
		}

		nbrBundleSent = 0;
		nbrBundleReceived = 0;
		nbrUniqueBundleReceived = 0;

		nbrMsgSent = 0;
		nbrMsgReceived = 0;

		DtnApplLayer::dataLengthBitsAsStatic = dataLengthBits;

		avgDelay = 0;
		totalDelay = 0;

		avgHops = 0;
		totalHops = 0;

    	delayStats.setName("DelayStats for 1st Copy");
	    hopCountStats.setName("HopCountStats for 1st Copy");

		std::string receptionType (par("dtnMsgReception").stringValue());
		if (receptionType == "Unique"){
			receivingStrategy = Unique;
		}else if (receptionType == "Random"){
			receivingStrategy = Random;
		}else if (receptionType == "Any"){
			receivingStrategy = Any;
		}else {
			opp_error("Unrecognized data Traffic Reception Type");
		}

		destAddr = par("destAddr");

		sentSignalId = registerSignal("sentBndl");
		receiveSignalId = registerSignal("receivedBndl");

	}
	if(stage==1){
		isNetwAddrInit = false;
	}
	if(stage==2) {
		DtnNetwLayer* netw = FindModule<DtnNetwLayer*>::findSubModule(this->getParentModule());
		if (netw!=NULL){
			netwAddr = netw->getMyNetwAddr();
			isNetwAddrInit = true;
			isEquiped = netw->isAnEquipedVehicle();
		}else{
			opp_error("Impossible to find a DtnNetw Module");
		}
	}
}



void DtnApplLayer::finish()
{
	// Added by Arslan HAMZA CHERIF
	recordScalar("# Bundle Sent", nbrBundleSent);
	recordScalar("# Bundle Received", nbrBundleReceived);
	recordScalar("# Unique Bundle Received", nbrUniqueBundleReceived);

	delayStats.recordAs("Delays for 1st Copy");
	hopCountStats.recordAs("HopCount for 1st Copy");

	// cancelling selfmessage
	BaseWaveApplLayer::finish();
	if(withDtnMsg){
		cancelAndDelete(dtnMsg);
	}
}

void DtnApplLayer::onBeacon(WaveShortMessage *wsm)
{
}



std::string DtnApplLayer::getDataSrcFromStrategy(std::string currentStrategy)
{
	return currentStrategy.substr(0,3);
}

std::string DtnApplLayer::getDataDestFromStrategy(std::string currentStrategy)
{
	return currentStrategy.substr(4);
}

void DtnApplLayer::onData(WaveShortMessage *wsm)
{
}



DtnApplLayer::~DtnApplLayer()
{
}

int DtnApplLayer::getDataLengthBitsAsStatic()
{
	return dataLengthBitsAsStatic;
}

int DtnApplLayer::dataLengthBitsAsStatic = 0;
