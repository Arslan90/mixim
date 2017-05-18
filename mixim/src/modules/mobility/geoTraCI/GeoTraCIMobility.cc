//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
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

#include <limits>
#include <iostream>
#include <sstream>
//#include <cstring>      // Needed for memset
//#include <sys/socket.h> // Needed for the socket functions
//#include <netdb.h>      // Needed for the socket functions
//#include <unistd.h>
#include "myConstants.h"
#include "errno.h"
#include "DtnNetwLayer.h"
#include "FindModule.h"
#include "Mac80211p.h"

#include "mobility/geoTraCI/GeoTraCIMobility.h"

Define_Module(GeoTraCIMobility);

namespace {
	const double MY_INFINITY = (std::numeric_limits<double>::has_infinity ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::max());

	double roadIdAsDouble(std::string road_id) {
		std::istringstream iss(road_id);
		double d;
		if (!(iss >> d)) return MY_INFINITY;
		return d;
	}
}

void GeoTraCIMobility::Statistics::initialize()
{
	firstRoadNumber = MY_INFINITY;
	startTime = simTime();
	totalTime = 0;
	stopTime = 0;
	minSpeed = MY_INFINITY;
	maxSpeed = -MY_INFINITY;
	totalDistance = 0;
	totalCO2Emission = 0;
}

void GeoTraCIMobility::Statistics::watch(cSimpleModule& )
{
	WATCH(totalTime);
	WATCH(minSpeed);
	WATCH(maxSpeed);
	WATCH(totalDistance);
}

double GeoTraCIMobility::getCurrentMetd() const
{
    return currentMETD;
}

NearestPoint GeoTraCIMobility::getCurrentNp() const
{
    return currentNP;
}

void GeoTraCIMobility::setCurrentMetd(double currentMetd)
{
    currentMETD = currentMetd;
}

void GeoTraCIMobility::Statistics::recordScalars(cSimpleModule& module)
{
	if (firstRoadNumber != MY_INFINITY) module.recordScalar("firstRoadNumber", firstRoadNumber);
	module.recordScalar("startTime", startTime);
	module.recordScalar("totalTime", totalTime);
	module.recordScalar("stopTime", stopTime);
	if (minSpeed != MY_INFINITY) module.recordScalar("minSpeed", minSpeed);
	if (maxSpeed != -MY_INFINITY) module.recordScalar("maxSpeed", maxSpeed);
	module.recordScalar("totalDistance", totalDistance);
	module.recordScalar("totalCO2Emission", totalCO2Emission);
}

void GeoTraCIMobility::initialize(int stage)
{
	if (stage == 0)
	{
		BaseMobility::initialize(stage);

		debug = par("debug");
		accidentCount = par("accidentCount");

		withHotSpotMode = par("withHotSpotMode").boolValue();

		currentPosXVec.setName("posx");
		currentPosYVec.setName("posy");
		currentSpeedVec.setName("speed");
		currentAccelerationVec.setName("acceleration");
		currentCO2EmissionVec.setName("co2emission");

		statistics.initialize();
		statistics.watch(*this);

		ASSERT(isPreInitialized);
		isPreInitialized = false;

		move.setStart(Coord(nextPos.x, nextPos.y, move.getCurrentPosition().z)); // keep z position
		move.setDirectionByVector(Coord(cos(angle), -sin(angle)));
		move.setSpeed(speed);

		WATCH(road_id);
		WATCH(speed);
		WATCH(angle);

		startAccidentMsg = 0;
		stopAccidentMsg = 0;
		manager = 0;
		last_speed = -1;

		if (accidentCount > 0) {
			simtime_t accidentStart = par("accidentStart");
			startAccidentMsg = new cMessage("scheduledAccident");
			stopAccidentMsg = new cMessage("scheduledAccidentResolved");
			scheduleAt(simTime() + accidentStart, startAccidentMsg);
		}
		initScenarioType();

	}
	else if (stage == 1)
	{
		// don't call BaseMobility::initialize(stage) -- our parent will take care to call changePosition later

		pyManager = FindModule<PyGraphServerManager*>::findGlobalModule();
		ASSERT(pyManager);

		this->initialRoute = commandGetEdgesOfRoute(commandGetRouteId());
		addRouteToEdgeBTTIndex(initialRoute);
		this->remainingRoute = initialRoute;
		this->indexRoadId = roadIndexInRoute(road_id,initialRoute, this->indexLastRoadId);

		computeNPMsg = new cMessage("computeNPMsg");
		scheduleAt(simTime(), computeNPMsg);

		currentNP = NearestPoint();
		currentETA_NP_VPA = currentNP.getEtaNpVpa();
		currentMETD = currentNP.getMetd();

		hadDistZero = false;

		sectorChange = registerSignal("sectorChanged");

		updateCurrentSector();



	}
	else
	{
		BaseMobility::initialize(stage);
	}

}

void GeoTraCIMobility::finish()
{
	statistics.stopTime = simTime();

	statistics.recordScalars(*this);

	cancelAndDelete(startAccidentMsg);
	cancelAndDelete(stopAccidentMsg);

	isPreInitialized = false;
}

void GeoTraCIMobility::handleSelfMsg(cMessage *msg)
{
	if (msg == startAccidentMsg) {
		commandSetSpeed(0);
		simtime_t accidentDuration = par("accidentDuration");
		scheduleAt(simTime() + accidentDuration, stopAccidentMsg);
		accidentCount--;
	}
	else if (msg == stopAccidentMsg) {
		commandSetSpeed(-1);
		if (accidentCount > 0) {
			simtime_t accidentInterval = par("accidentInterval");
			scheduleAt(simTime() + accidentInterval, startAccidentMsg);
		}
	}else if (msg == computeNPMsg){
	    std::string Old_NP_node = currentNP.getNpNode();
	    double oldMETD = currentNP.getMetd();
	    double oldEtaNpVpa = currentNP.getEtaNpVpa();
	    std::string oldNpEdgeTo = currentNP.getNpEdgeTo();
		currentNP = getNearestPoint(currentSector, remainingRoute);
		if ((currentNP.getNpNode() == Old_NP_node) && (currentNP.getNpEdgeTo() == oldNpEdgeTo)){
			currentNP.setMetd(oldMETD);
			currentNP.setEtaNpVpa(oldEtaNpVpa);
		}else{
//			std::cout << "Module ID: " << this->getId() << endl;
		}
		updateMETD(true);
		currentMETD = currentNP.getMetd();
		currentETA_NP_VPA = currentNP.getEtaNpVpa();
	}
}

void GeoTraCIMobility::preInitialize(std::string external_id, const Coord& position, std::string road_id, double speed, double angle)
{
	this->external_id = external_id;
	this->lastUpdate = 0;
	this->nextPos = position;
	this->road_id = road_id;
	this->speed = speed;
	this->angle = angle;
	move.setStart(Coord(position.x, position.y, move.getCurrentPosition().z)); // keep z position
	move.setDirectionByVector(Coord(cos(angle), -sin(angle)));
	move.setSpeed(speed);

	//My variables
	this->lastRoadId = "";
	this->indexLastRoadId = -1;

	isPreInitialized = true;
}

void GeoTraCIMobility::nextPosition(const Coord& position, std::string road_id, double speed, double angle)
{
	if (debug) EV << "nextPosition " << position.x << " " << position.y << " " << road_id << " " << speed << " " << angle << std::endl;
	isPreInitialized = false;
	nextPos = position;
	// Check if current edge changed
	if (road_id != this->road_id){
		int indexNP_edgeTo = roadIndexInRoute(currentNP.getNpEdgeTo(), initialRoute, this->indexLastRoadId, true);
		if ((currentNP.isValid()) && (indexNP_edgeTo != -1) && (indexNP_edgeTo < indexRoadId)){
			scheduleAt(simTime(), computeNPMsg);
		}

		if (roadInRoute(this->road_id, initialRoute)){
			lastRoadId = this->road_id;
			indexLastRoadId = indexRoadId;
		}

		this->road_id = road_id;
		this->indexRoadId = roadIndexInRoute(this->road_id, initialRoute,this->indexLastRoadId);
		updateRemainingRoute();
		updateMETD(false);
		currentMETD = currentNP.getMetd();
	}

	this->speed = speed;
	this->angle = angle;
	changePosition();
	updateCurrentSector();
}

void GeoTraCIMobility::changePosition()
{
	// ensure we're not called twice in one time step
	ASSERT(lastUpdate != simTime());

	// keep statistics (for current step)
	currentPosXVec.record(move.getStartPos().x);
	currentPosYVec.record(move.getStartPos().y);

	// keep statistics (relative to last step)
	if (statistics.startTime != simTime()) {
		simtime_t updateInterval = simTime() - this->lastUpdate;
		this->lastUpdate = simTime();

		double distance = move.getStartPos().distance(Coord(nextPos.x, nextPos.y, move.getCurrentPosition().z));
		statistics.totalDistance += distance;
		statistics.totalTime += updateInterval;
		if (speed != -1) {
			statistics.minSpeed = std::min(statistics.minSpeed, speed);
			statistics.maxSpeed = std::max(statistics.maxSpeed, speed);
			currentSpeedVec.record(speed);
			if (last_speed != -1) {
				double acceleration = (speed - last_speed) / updateInterval;
				double co2emission = calculateCO2emission(speed, acceleration);
				currentAccelerationVec.record(acceleration);
				currentCO2EmissionVec.record(co2emission);
				statistics.totalCO2Emission+=co2emission * updateInterval.dbl();
			}
			last_speed = speed;
		} else {
			last_speed = -1;
			speed = -1;
		}
	}

	move.setStart(Coord(nextPos.x, nextPos.y, move.getCurrentPosition().z)); // keep z position
	move.setDirectionByVector(Coord(cos(angle), -sin(angle)));
	move.setSpeed(speed);
	if (ev.isGUI()) updateDisplayString();
	fixIfHostGetsOutside();
	updatePosition();
}

void GeoTraCIMobility::updateCurrentSector()
{
	if (scenarioModel == Sector){
		Coord vehiclePosition;//actual XY vehicle position
		Coord vehiclePosSumo;//actual XY vehicle position
		int row; //row of sector
		int col; //column of sector
		int inSector; //the sector in.

		//step 1
		vehiclePosition = getCurrentPosition();
		vehiclePosSumo.x = getManager()->omnet2traci(vehiclePosition).x;
		vehiclePosSumo.y = getManager()->omnet2traci(vehiclePosition).y;
		//MYDEBUG <<"logs, vehicle position," << traci->getExternalId() << ",x," << vehiclePosition.x <<",y,"<< axeY  <<endl;

		//step 2
		col= int( (vehiclePosSumo.x - sectorOffsetX) / sectorSizeX); //Substract the 4k X-axis offset. Divide by 1000 and get the integer.
		if (!((0 <= col)&&(col < colSectorGrid))){
			opp_error("Error with column index calculation");
		}
		row= int( (vehiclePosSumo.y - sectorOffsetY) / sectorSizeY); //Substract the 5k Y-axis offset. Divide by 1000 and get the integer.
		if (!((0 <= row)&&(row < rowSectorGrid))){
			opp_error("Error with row index calculation");
		}
		inSector= (col * rowSectorGrid) + row; //Every column has 33rows. Sectors starts in cero from bottom to top and left to right.

		//if (((vehiclePosSumo.x < 150) || (vehiclePosSumo.x > 1500))
		//		|| ((vehiclePosSumo.y < 150) || (vehiclePosSumo.y > 1500))){
		//	inSector = -1;
		//}

		//if I noticed a change of sectorID reset everything. Also gives the current counter o'course.
		//Note: this counter is complemented with the counter in received messages by VPAs or Vehicles.
		if (inSector != currentSector){
			currentSector= inSector; //update Veh sector transit
			if (! computeNPMsg->isScheduled() ){
				scheduleAt(simTime(), computeNPMsg);
			}
			std::stringstream ss1,ss2;
			ss1 << oldSector;
			ss2 << currentSector;
			std::string str = ss1.str()+":"+ss2.str();
			if (getParentModule()->findSubmodule("netw")!=-1){
				DtnNetwLayer* netwMod = FindModule<DtnNetwLayer*>::findSubModule(getParentModule());
				if(netwMod->isHadBundles()){
					str= str+":"+"1";
				}else{
					str= str+":"+"0";
				}
//				std::stringstream ss5;
//				ss5 << netwMod->nbrBundles();
//				str= str+":"+ss5.str();
				if(netwMod->isMeetVpa()){
					str= str+":"+"1";
				}else{
					str= str+":"+"0";
				}
				if (hadDistZero){
					str= str+":"+"1";
				}else{
					str= str+":"+"0";
				}
				std::stringstream ss3,ss4,ss6,ss7,ss8,ss9,ss10,ss11,ss12;
//				ss3 << netwMod->nbrAckReceivedPerVpa();
//				str= str+":"+ss3.str();
//				ss4 << netwMod->nbrBundleSentPerVpa();
//				str= str+":"+ss4.str();
				str= str+":"+netwMod->BundleSentPerVpaSerialToString();
				ss3 << netwMod->getNbrNeighors();
				str= str+":"+ss3.str();
				ss4 << netwMod->getNbrCountForMeanNeighbors();
				str= str+":"+ss4.str();
				std::pair<double,double> vpaContact = netwMod->VPAContactDuration();
				ss6 << vpaContact.first;
				str= str+":"+ss6.str();
				ss7 << vpaContact.second;
				str= str+":"+ss7.str();
				std::pair<double,double> vpaDistance = netwMod->VPAContactDistance();
				ss8 << vpaDistance.first;
				str= str+":"+ss8.str();
				ss9 << vpaDistance.second;
				str= str+":"+ss9.str();
				ss10 << netwMod->getReceivedHwicvpa();
				ss11 << netwMod->getReceivedBwicvpa();
				ss12 << netwMod->getReceivedAwicvpa();
				if ((netwMod->getReceivedHwicvpa() != 0) ||(netwMod->getReceivedBwicvpa() != 0)||(netwMod->getReceivedAwicvpa() != 0)){
					cout << "debug found" << endl;
				}
				str= str+":"+ss10.str()+":"+ss11.str()+":"+ss12.str();

				if (getParentModule()->findSubmodule("nic")!=-1){
					cModule* nicMod = this->getParentModule()->getSubmodule("nic");
					if ((nicMod != NULL) && (nicMod->findSubmodule("mac80211p") != -1)){
						Mac80211p* macMod = FindModule<Mac80211p*>::findSubModule(nicMod);
						str= str+":"+getL2Stats((cModule*) macMod);
					}else{
						opp_error("Unable to found mac module to collect L2 Stats");
					}
				}

				netwMod->resetStatPerVPA();
				hadDistZero = false;
			}
			emit(sectorChange,str.c_str());
		}else {
			oldSector = currentSector;
		}
	}
}

void GeoTraCIMobility::reComputeMETDAfterReRoute(){
	Enter_Method_Silent("reComputeMETDAfterReRoute");
	this->initialRoute = commandGetEdgesOfRoute(commandGetRouteId());
	std::list<std::string> newInitialRoute = commandGetEdgesOfVehRoute();
	addRouteToEdgeBTTIndex(initialRoute);
	this->remainingRoute = initialRoute;
	this->indexLastRoadId = -1;
	this->indexRoadId = roadIndexInRoute(road_id,initialRoute, this->indexLastRoadId);

	if (!computeNPMsg->isScheduled()){
		scheduleAt(simTime(), computeNPMsg);
	}
}


int GeoTraCIMobility::roadIndexInRoute(std::string roadId, std::list<std::string> route, int indexLastRoadId, bool canEquals)
{
	bool found = false;
	int index = -1, i = -1;
	std::list<int> indexes;
	if (!route.empty()){
		for (std::list<std::string>::iterator it = route.begin(); it != route.end(); it++){
			i+=1;
			if (*it == roadId){
				if ((canEquals) && (i>= indexLastRoadId)){
					   found = true;
					   index = i;
					   break;
				}else if (i> indexLastRoadId){
					   found = true;
					   index = i;
					   break;
				}
			}
		}
		if (!found){
			index = -1;
		}
	}
	return index;
}

bool GeoTraCIMobility::roadInRoute(std::string roadId, std::list<std::string> route)
{
	bool found = false;
	if (!route.empty()){
		for (std::list<std::string>::iterator it = route.begin(); it != route.end(); it++){
			if (*it == roadId){
				found = true;
				break;
			}
		}
	}
	return found;
}

void GeoTraCIMobility::updateRemainingRoute()
{
	if ((indexLastRoadId != -1)&&(indexRoadId != -1)){
		int nbrPushes = indexRoadId - indexLastRoadId;
		for (int i = 1; i <= nbrPushes; ++i) {
			remainingRoute.pop_front();
		}
	}
}


void GeoTraCIMobility::updateMETD(bool reComputeETA_NP_VPA)
{
	double currentETA_NP = maxDbl;
	double currentRmgRoadBestTT = maxDbl; // RmgRoadBestTT: Remaining Road Best Travel Time

	if (currentNP.isInitialized()){
		if (currentNP.isValid()){
			currentRmgRoadBestTT = calculateRmgRoadBestTT();
			currentETA_NP = calculateETA_NP(false);
			if (currentETA_NP != -1){
				double currentETA_NP_VPA = maxDbl;
				if (!reComputeETA_NP_VPA){
					// It is only an update of METD, with ETA_NP_VPA previously calculated
					if (currentNP.getEtaNpVpa() == maxDbl){
						// but if previously with get a maxDbl value for it, then we recalculate it
						currentETA_NP_VPA = calculateETA_NP_VPA();
						if (currentETA_NP_VPA != maxDbl){
							currentNP.setEtaNpVpa(calculateETA_NP_VPA());
						}
//						opp_error("Calling updateMETD without previous calculation of ETA_NP_VPA");
					}
				}else {
					// It is a full compute of METD, with calculation of both ETA_NP and ETA_NP_VPA
					currentETA_NP_VPA = calculateETA_NP_VPA();
					if (currentETA_NP_VPA != maxDbl){
						currentNP.setEtaNpVpa(calculateETA_NP_VPA());
					}
				}
				currentNP.setMetd(currentRmgRoadBestTT+currentETA_NP+currentNP.getEtaNpVpa());
				if ((currentNP.getMetd() == maxDbl) && (currentNP.isValid())){
					opp_error("Unable to calculate METD when having already a valid NP_Node");
				}
			}
		}else{
			return;
		}
	}else {
		return;
	}



//	int indexNP_edgeTo = roadIndexInRoute(currentNP.getNpEdgeTo(), initialRoute);
//	if ((indexNP_edgeTo != -1) && (indexRoadId == -1)) {
//		// We don't updateMETD, instead we consider it as valid, until the next compute
//		return;
//	}
//
//	double currentETA_NP = maxDbl;
//	double currentRmgRoadBestTT = maxDbl; // RmgRoadBestTT: Remaining Road Best Travel Time
//	if (!reComputeETA_NP_VPA){
//		// It is only an update of METD, with ETA_NP_VPA previously calculated
//		if (currentETA_NP_VPA == maxDbl){
//			opp_error("Calling updateMETD without previous calculation of ETA_NP_VPA");
//		}
//	}else {
//		// It is a full compute of METD, with calculation of both ETA_NP and ETA_NP_VPA
//		currentETA_NP_VPA = calculateETA_NP_VPA();
//	}
//
//	currentRmgRoadBestTT = calculateRmgRoadBestTT();
//	currentETA_NP = calculateETA_NP(false);
//
//	if ((!currentNP.isValid()) || (currentETA_NP_VPA == maxDbl) ||
//			(currentETA_NP == maxDbl) || (currentRmgRoadBestTT == maxDbl)	){
//		currentMETD = maxDbl;
//	}else{
//		currentMETD = currentRmgRoadBestTT + currentETA_NP + currentETA_NP_VPA;
//	}
//
//	if ((currentMETD == maxDbl) && (currentNP.isValid())){
//		opp_error("Unable to calculate METD when having already a valid NP_Node");
//	}
}

double GeoTraCIMobility::calculateRmgRoadBestTT()
{
	double rmgRoadBestTT = maxDbl;
	std::string laneId = commandGetLaneId();
	if (laneId == ""){
		std::string error_str = "Unable to retrieve LaneId for EdgeId: "+road_id;
		opp_error(error_str.c_str());
	}else{
		double laneTotalLength = cmdGetLaneLength(laneId);
		double currentVehLanePos = cmdGetVehiclelanePosition();
		double laneMaxSpeed = cmdGetLaneMaxSpeed(laneId);
		std::ostringstream iss1, iss2;
		iss1 << currentVehLanePos;
		iss2 << laneTotalLength;
		if (laneTotalLength < currentVehLanePos){
			std::string warning_str = "LanePos for Veh "+this->external_id+" is higher than max length of LaneId "+laneId+"("+iss1.str()+" > "+iss2.str()+")";
			opp_warning(warning_str.c_str());
			opp_warning("Assuming that vehicle has traversed the current lane");
			rmgRoadBestTT = 0;
		}
		if (laneMaxSpeed == 0.0){
			std::string warning_str = "Max Speed for LaneIdPos "+laneId+" equals 0.0";
			opp_warning(warning_str.c_str());
		}else{
			rmgRoadBestTT = (laneTotalLength-currentVehLanePos) / laneMaxSpeed;
		}
	}
	return rmgRoadBestTT;
}

double GeoTraCIMobility::calculateETA_NP(bool includeCurrentRoad)
{
	double ETA_NP = maxDbl;
	if (currentNP.isValid()){
		int indexNP_edgeTo = roadIndexInRoute(currentNP.getNpEdgeTo(), initialRoute, this->indexLastRoadId, true);
		if (indexNP_edgeTo != -1){
			if (indexRoadId > indexNP_edgeTo){
				ETA_NP = -1;
			}else if(indexRoadId == -1){
				ETA_NP = -1;
			}else if ((indexRoadId <= indexNP_edgeTo)){
				ETA_NP = 0.0;
				bool foundNP_edgeTo = false;
				for (std::list<std::string>::iterator it = remainingRoute.begin(); it != remainingRoute.end(); it++){
					if ((!includeCurrentRoad) && (*it == this->road_id)){
						if (*it == currentNP.getNpEdgeTo()){
							foundNP_edgeTo = true;
							break;
						}else{
							continue;
						}
					}else{
						ETA_NP += getEdgeBestTravelTime(*it);
						if (*it == currentNP.getNpEdgeTo()){
							foundNP_edgeTo = true;
							break;
						}
					}
				}
				if (!foundNP_edgeTo){
					opp_error("Unable to find NP_edgeTo");
				}
			}else {
				opp_error("Invalid IndexRoadId");
			}
		}else{
			ETA_NP = -1;
//			opp_error("Valid NP but unable to get NP_EdgeTo Index");
		}
	}
	return ETA_NP;


//	int indexNP_edgeTo = roadIndexInRoute(currentNP.getNpEdgeTo(), initialRoute);
//	if (indexNP_edgeTo == -1){
//		std::string warning_str = "Unable to find index of NP_edgeTo "+currentNP.NP_edgeTo+" for VehId "+this->external_id;
//		opp_warning(warning_str.c_str());
//	}
//	if ((indexNP_edgeTo != -1) && (indexRoadId != -1) && (indexNP_edgeTo >= indexRoadId)){
//		ETA_NP = 0.0;
//		bool foundNP_edgeTo = false;
//		for (std::list<std::string>::iterator it = remainingRoute.begin(); it != remainingRoute.end(); it++){
//			if (*it == currentNP.getNpEdgeTo()){
//				foundNP_edgeTo = true;
//				break;
//			}
//
//			if ((!includeCurrentRoad) && (*it == this->road_id)){
//				continue;
//			}else{
//				ETA_NP += getEdgeBestTravelTime(*it);
//			}
//		}
//		if (!foundNP_edgeTo){
//			opp_error("Unable to find NP_edgeTo");
//		}
//	}
//	return ETA_NP;
}

double GeoTraCIMobility::calculateETA_NP_VPA()
{
	double ETA_NP_VPA = maxDbl;
	if (currentNP.isValid()){
		if ((currentNP.getRouteNpVpa().empty()) && (currentNP.getDistanceNpVpa() == 0.0)){
			ETA_NP_VPA = 0.0;
		}else{
			ETA_NP_VPA = 0.0;
			std::list<std::string> route = currentNP.getRouteNpVpa();
			for (std::list<std::string>::iterator it = route.begin(); it != route.end(); it++){
				ETA_NP_VPA += getEdgeBestTravelTime(*it);
			}
		}
	}else {
//		std::cout << "is not valid " << endl;
	}
//
//
//	if ((currentNP.getRouteNpVpa().empty()) && (currentNP.getDistanceNpVpa() == 0.0)){
//		ETA_NP_VPA = 0.0;
//	}else if (currentNP.isValid()){
//		ETA_NP_VPA = 0.0;
//		for (std::list<std::string>::iterator it = currentNP.getRouteNpVpa().begin(); it != currentNP.getRouteNpVpa().end(); it++){
//			ETA_NP_VPA += getEdgeBestTravelTime(*it);
//		}
//	}
	return ETA_NP_VPA;
}

double GeoTraCIMobility::getEdgeBestTravelTime(std::string edgeId)
{
	double travelTime = maxDbl;

	if (edgesBTTIndex.find(edgeId) != edgesBTTIndex.end()){
		travelTime = edgesBTTIndex[edgeId];
	}else{
		// Creating the command
		int msg_type = MY_CONST::QUERY;
		int cmd = MY_CONST::CMD_EDGE_BEST_TRAVEL_TIME;

		int arg1 = MY_CONST::EDGE_ID;
		std::string arg1_value = edgeId;

		std::string msg = "";
		msg+= MY_CONST::convertToStr(msg_type)+":"+MY_CONST::convertToStr(cmd)+";";
		msg+= MY_CONST::convertToStr(arg1)+":"+arg1_value+"?";

		// Sending request to server
		std::string queryRep = sendRequestToPyServer(msg);

		// Tokenizing request result
		std::vector<std::string> reponse_tokens = MY_CONST::tokenizeMSG(queryRep);

		// Checking if received data are correct
		if (reponse_tokens.empty() || (reponse_tokens.size() != 4)){
			std::string errorMsg = "RESPONSE_EDGE_BEST_TRAVEL_TIME not correctly computed: "+queryRep+" Sent msg"+msg;
			opp_error(errorMsg.c_str());
		}
		bool abort = false;
		if (reponse_tokens[0] != MY_CONST::convertToStr(MY_CONST::RESPONSE)) {abort = true;}
		if (reponse_tokens[1] != MY_CONST::convertToStr(MY_CONST::RESPONSE_EDGE_BEST_TRAVEL_TIME)) {abort = true;}
		if (reponse_tokens[2] != MY_CONST::convertToStr(MY_CONST::EDGE_BEST_TRAVEL_TIME)) {abort = true;}

		if (abort){
			opp_warning("Bad format for reponse_EDGE_BEST_TRAVEL_TIME");
		}else{
			travelTime = MY_CONST::convertToDbl(reponse_tokens[3]);
		}
	}

	return travelTime;
}



NearestPoint GeoTraCIMobility::getNearestPoint(int vpaSectorId, std::list<std::string> currentRoute)
{
	NearestPoint returnedNP = NearestPoint();
	int msg_type = MY_CONST::QUERY;
	int cmd = MY_CONST::CMD_NP_ALL;

	int arg1 = MY_CONST::VPA_SECTOR_ID;
	int arg1_value = vpaSectorId;

	int arg2 = MY_CONST::ROUTE_CURRENT;
	std::string arg2_value = "";
	for (std::list<std::string>::iterator it = currentRoute.begin(); it != currentRoute.end(); it++){
		if (arg2_value == ""){
			arg2_value = *it;
		}else{
			arg2_value = arg2_value+" "+*it;
		}
	}
	int arg3 = MY_CONST::HOTSPOT_MODE;
	std::string arg3_value = "";
	if (withHotSpotMode){
		arg3_value = "True";
	}else{
		arg3_value = "False";
	}

	std::string msg = "";
	msg+= MY_CONST::convertToStr(msg_type)+":"+MY_CONST::convertToStr(cmd)+";";
	msg+= MY_CONST::convertToStr(arg1)+":"+MY_CONST::convertToStr(arg1_value)+";";
	msg+= MY_CONST::convertToStr(arg2)+":"+arg2_value+";";
	msg+= MY_CONST::convertToStr(arg3)+":"+arg3_value+"?";

	// Sending request to server
	std::string queryRep = sendRequestToPyServer(msg);

	// Tokenizing request result
	std::vector<std::string> reponse_tokens = MY_CONST::tokenizeMSG(queryRep);

	// Checking if received data are correct
	if (reponse_tokens.empty() || (reponse_tokens.size() != 14)){
		std::string errorMsg = "RESPONSE_NP_ALL not correctly computed: "+queryRep+" Sent msg"+msg;
		opp_error(errorMsg.c_str());
	}

	// Checking if received data are correct
	bool abort = false;
	if (reponse_tokens[0] != MY_CONST::convertToStr(MY_CONST::RESPONSE)) {abort = true;}
	if (reponse_tokens[1] != MY_CONST::convertToStr(MY_CONST::RESPONSE_NP_ALL)) {abort = true;}
	if (reponse_tokens[2] != MY_CONST::convertToStr(MY_CONST::EDGE_TO_NP)) {abort = true;}
	if (reponse_tokens[4] != MY_CONST::convertToStr(MY_CONST::NODE_NP)) {abort = true;}
	if (reponse_tokens[6] != MY_CONST::convertToStr(MY_CONST::EDGE_FROM_NP)) {abort = true;}
	if (reponse_tokens[8] != MY_CONST::convertToStr(MY_CONST::NODE_VPA_MAPPING)) {abort = true;}
	if (reponse_tokens[10] != MY_CONST::convertToStr(MY_CONST::ROUTE_NP_VPA)) {abort = true;}
	if (reponse_tokens[12] != MY_CONST::convertToStr(MY_CONST::ROUTE_LENGTH_NP_VPA)) {abort = true;}

	if (abort){
		opp_warning("Bad format for reponse_NP_ALL");
	}else{

		double distance = maxDbl;
		std::list<std::string> route;
		char* edges = strtok(strdup(reponse_tokens[11].c_str())," ");
		std::list<std::string> keysValues;
		while (edges != NULL){
				   keysValues.push_back(std::string(edges));
				edges = strtok(NULL," ");
		}
	   for (std::list<std::string>::iterator it = keysValues.begin(); it != keysValues.end(); it++){
			   std::string tmp = *it;
			   std::pair<std::string, double> pair = addEntryToEdgeBTTIndex(tmp);
			   route.push_back(pair.first);
        }
		distance = MY_CONST::convertToDbl(reponse_tokens[13]);

		if (distance == 0){
			hadDistZero = true;
		}

		returnedNP = NearestPoint(reponse_tokens[3], reponse_tokens[5], reponse_tokens[7], reponse_tokens[9], route, distance);
	}

	return returnedNP;
}

std::pair<std::string, double>  GeoTraCIMobility::addEntryToEdgeBTTIndex(std::string edgeAndValue)
{
	char* edge =  strtok(strdup(edgeAndValue.c_str()),"=");
	std::string edgeAsStr = std::string(edge);
	char* value = strtok(NULL,"=");
	double valueAsFloat = maxDbl;
	if ((value != NULL) && (std::string(value) != "")){
		if (edgeAsStr != "ND"){
			valueAsFloat = MY_CONST::convertToDbl(value);
		}
		if (edgesBTTIndex.find(edgeAsStr) == edgesBTTIndex.end()){
			edgesBTTIndex.insert(std::pair<std::string,double>(edgeAsStr,valueAsFloat));
		}
	}
	return std::pair<std::string, double>(edgeAsStr,valueAsFloat);
}

void GeoTraCIMobility::addRouteToEdgeBTTIndex(std::list<std::string> route)
{
	double travelTime;
	std::string edge ="";
	for (std::list<std::string>::iterator it = route.begin(); it != route.end(); it++){
		 travelTime = maxDbl;
		 edge = *it;
		 if (edge == "ND"){
			 edgesBTTIndex.insert(std::pair<std::string,double>(edge,travelTime));
		 }else{
			 if (edgesBTTIndex.find(edge) == edgesBTTIndex.end()){
				 travelTime = getEdgeBestTravelTime(edge);
				 edgesBTTIndex.insert(std::pair<std::string,double>(edge,travelTime));
			 }
		 }
	}
}

std::string GeoTraCIMobility::sendRequestToPyServer(std::string buf)
{
//	std::string rep;
//
//	std::string HOST = "127.0.0.1";
//	int PORT = 19999;
//	int MAX_BUFFER = 4096;
//
//	int connectionFd, rc, index = 0, limit = MAX_BUFFER;
////	struct sockaddr_in servAddr, localAddr;
//	char buffer[MAX_BUFFER+1];
//
//
////	memset(&servAddr, 0, sizeof(servAddr));
////	servAddr.sin_family = AF_INET;
////	servAddr.sin_port = htons(PORT);
////	servAddr.sin_addr.s_addr = inet_addr(HOST.c_str());
//
//	// Create socket
//	connectionFd = socket(AF_INET, SOCK_STREAM, 0);
//
////	/* bind any port number */
////	memset(&localAddr, 0, sizeof(localAddr));
////	localAddr.sin_family = AF_INET;
////	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
////	localAddr.sin_port = htons(0);
//
//	int returnCode = 0;
//
//	returnCode = bind(connectionFd,
//	  (struct sockaddr *) &localAddr, sizeof(localAddr));
//	if (returnCode == -1){
//		std::stringstream ss;
//		ss << errno;
//		std::string errorMsg = "(BIND) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
//		opp_error(errorMsg.c_str());
//	}
//
//
//	// Connect to Server
//	returnCode = connect(connectionFd,(struct sockaddr *)&servAddr, sizeof(servAddr));
//	if (returnCode == -1){
//		std::stringstream ss;
//		ss << errno;
//		std::string errorMsg = "(CONNECT) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
//		opp_error(errorMsg.c_str());
//	}
//
//	// Sending request
//	sprintf( buffer, "%s", buf.c_str() );
//	returnCode = ::send(connectionFd, buffer, strlen(buffer), 0 );
////	printf("Client send to Server %s\n", buffer);
//	if (returnCode == -1){
//		std::stringstream ss;
//		ss << errno;
//		std::string errorMsg = "(SEND) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
//		opp_error(errorMsg.c_str());
//	}
//
//	// Receiving request
//	memset(&buffer[0], 0, sizeof(buffer));
//	returnCode = recv(connectionFd, buffer, MAX_BUFFER, 0);
////	printf("Client read from Server %s\n", buffer);
//
//	if (returnCode == -1){
//		std::stringstream ss;
//		ss << errno;
//		std::string errorMsg = "(RECV) Socket error returned: "+std::string(strerror(errno))+ " Code: "+ss.str();
//		opp_error(errorMsg.c_str());
//	}
//	rep = std::string(buffer);
//
//	// Closing connection to Server
//	close(connectionFd);


	return pyManager->sendRequestToPyServer(buf);

}

void GeoTraCIMobility::updateDisplayString() {
	ASSERT(-M_PI <= angle);
	ASSERT(angle < M_PI);

	getParentModule()->getDisplayString().setTagArg("b", 2, "rect");
	getParentModule()->getDisplayString().setTagArg("b", 3, "red");
	getParentModule()->getDisplayString().setTagArg("b", 4, "red");
	getParentModule()->getDisplayString().setTagArg("b", 5, "0");

	if (angle < -M_PI + 0.5 * M_PI_4 * 1) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2190");
		getParentModule()->getDisplayString().setTagArg("b", 0, "4");
		getParentModule()->getDisplayString().setTagArg("b", 1, "2");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 3) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2199");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 5) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2193");
		getParentModule()->getDisplayString().setTagArg("b", 0, "2");
		getParentModule()->getDisplayString().setTagArg("b", 1, "4");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 7) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2198");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 9) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2192");
		getParentModule()->getDisplayString().setTagArg("b", 0, "4");
		getParentModule()->getDisplayString().setTagArg("b", 1, "2");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 11) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2197");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 13) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2191");
		getParentModule()->getDisplayString().setTagArg("b", 0, "2");
		getParentModule()->getDisplayString().setTagArg("b", 1, "4");
	}
	else if (angle < -M_PI + 0.5 * M_PI_4 * 15) {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2196");
		getParentModule()->getDisplayString().setTagArg("b", 0, "3");
		getParentModule()->getDisplayString().setTagArg("b", 1, "3");
	}
	else {
		getParentModule()->getDisplayString().setTagArg("t", 0, "\u2190");
		getParentModule()->getDisplayString().setTagArg("b", 0, "4");
		getParentModule()->getDisplayString().setTagArg("b", 1, "2");
	}
}

std::string GeoTraCIMobility::getL2Stats(cModule* macMod)
{
	std::string str;
	Mac80211p* mac = check_and_cast<Mac80211p*>(macMod);
	str = str+":"+mac->getLastStatsSent();
	str = str+":"+mac->getLastStatsDropped();
	str = str+":"+mac->getLastStatsReceivedBroadcast();
	str = str+":"+mac->getLastStatsLost();
	str = str+":"+mac->getLastStatsNumBackoffs();
	str = str+":"+mac->getLastStatsBackoffDuration();
	return str;
}



//int GeoTraCIMobility::cmd_NP_ALL(){
//	int dist = 0;
//	updateCurrentSector();
//	int vpaIndex = currentSector;
//	//	using namespace SOCKET;
//	std::string HOST = "127.0.0.1";
//	int PORT = 19999;
//	int MAX_BUFFER = 4096;
//
//	int msg_type = MY_CONST::QUERY;
//	int cmd = MY_CONST::CMD_NP_ALL;
//
//	int arg1 = MY_CONST::VPA_SECTOR_ID;
//	int arg1_value = vpaIndex;
//
//	int arg2 = MY_CONST::ROUTE_CURRENT;
//	std::string arg2_value = "";
//	for (std::list<std::string>::iterator it = remainingRoute.begin(); it != remainingRoute.end(); it++){
//		if (arg2_value == ""){
//			arg2_value = *it;
//		}else{
//			arg2_value = arg2_value+" "+*it;
//		}
//	}
//
//	std::string msg = "";
//	msg+= MY_CONST::convertToStr(msg_type)+":"+MY_CONST::convertToStr(cmd)+";";
//	msg+= MY_CONST::convertToStr(arg1)+":"+MY_CONST::convertToStr(arg1_value)+";";
//	msg+= MY_CONST::convertToStr(arg2)+":"+arg2_value+"#";
//
//	int connectionFd, rc, index = 0, limit = MAX_BUFFER;
//	struct sockaddr_in servAddr, localAddr;
//	char buffer[MAX_BUFFER+1];
//
//
//	memset(&servAddr, 0, sizeof(servAddr));
//	servAddr.sin_family = AF_INET;
//	servAddr.sin_port = htons(PORT);
//	servAddr.sin_addr.s_addr = inet_addr(HOST.c_str());
//
//	// Create socket
//	connectionFd = socket(AF_INET, SOCK_STREAM, 0);
//
//	/* bind any port number */
//	localAddr.sin_family = AF_INET;
//	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
//	localAddr.sin_port = htons(0);
//
//	rc = bind(connectionFd,
//	  (struct sockaddr *) &localAddr, sizeof(localAddr));
//
//	// Connect to Server
//	connect(connectionFd,
//	  (struct sockaddr *)&servAddr, sizeof(servAddr));
//
//	// Send request to Server
////	std::string tmp = "";
////	for (std::list<std::string>::iterator it = remainingRoute.begin(); it != remainingRoute.end(); it++){
////		if (tmp == ""){
////			tmp = *it;
////		}else{
////			tmp = tmp+" "+*it;
////		}
////	}
////	std::stringstream ss;
////	ss << "ShortestPath_" << vpaIndex << "_";
////	std::string req= ss.str()+tmp+";";
//	sprintf( buffer, "%s", msg.c_str() );
//	::send(connectionFd, buffer, strlen(buffer), 0 );
//	printf("Client send to Server %s\n", buffer);
//
////	  printf("Client sent to sever %s\n", buffer);
////
////	  sprintf( buffer, "%s", " " );
////	  send( connectionFd, buffer, strlen(buffer), 0 );
////
////	  printf("Client sent to sever %s\n", buffer);
////
////	  sprintf( buffer, "%f", coordY );
////	  send( connectionFd, buffer, strlen(buffer), 0 );
////
////	   printf("Client sent to sever %s\n", buffer);
//
//	  // Receive data from Server
//	  // sprintf( buffer, "%s", "" );
//
//	memset(&buffer[0], 0, sizeof(buffer));
//	recv(connectionFd, buffer, MAX_BUFFER, 0);
//	printf("Client read from Server %s\n", buffer);
//
//
////	std::list<std::string> response;
////	char* tmp_str = strtok(buffer,";");
////	while (tmp_str != NULL){
////		printf("%s\n", tmp_str);
////		response.push_front(std::string(tmp_str));
////		tmp_str = strtok(NULL,";");
////	}
//
//	rep_NP_ALL(buffer);
//
//	close(connectionFd);
//
////	printf("Client closed.\n");
//	return dist;
//}
//
//void GeoTraCIMobility::rep_NP_ALL(char *data)
//{
//	std::vector<std::string> reponse_tokens = MY_CONST::tokenizeMSG(data);
//
//	// Checking if received data are correct
//	bool abort = false;
//	if (reponse_tokens[0] != MY_CONST::convertToStr(MY_CONST::RESPONSE)) {abort = true;}
//	if (reponse_tokens[1] != MY_CONST::convertToStr(MY_CONST::RESPONSE_NP_ALL)) {abort = true;}
//	if (reponse_tokens[2] != MY_CONST::convertToStr(MY_CONST::EDGE_TO_NP)) {abort = true;}
//	if (reponse_tokens[4] != MY_CONST::convertToStr(MY_CONST::NODE_NP)) {abort = true;}
//	if (reponse_tokens[6] != MY_CONST::convertToStr(MY_CONST::EDGE_FROM_NP)) {abort = true;}
//	if (reponse_tokens[8] != MY_CONST::convertToStr(MY_CONST::NODE_VPA_MAPPING)) {abort = true;}
//	if (reponse_tokens[10] != MY_CONST::convertToStr(MY_CONST::ROUTE_NP_VPA)) {abort = true;}
//	if (reponse_tokens[12] != MY_CONST::convertToStr(MY_CONST::ROUTE_LENGTH_NP_VPA)) {abort = true;}
//
//	if (abort){
//		opp_warning("Bad format for reponse_NP_ALL");
//	}else{
//
//		currentNP.NP_edgeTo = reponse_tokens[3];
//		currentNP.NP_node = reponse_tokens[5];
//		currentNP.NP_edgefrom = reponse_tokens[7];
//		currentNP.VPA_node = reponse_tokens[9];
//		char* edges = strtok(strdup(reponse_tokens[11].c_str())," ");
//		while (edges != NULL){
//			currentNP.Route_NP_VPA.push_back(std::string(edges));
//			edges = strtok(NULL," ");
//		}
//		currentNP.Distance_NP_VPA = MY_CONST::convertToDbl(reponse_tokens[13]);
//	}
//}

//void GeoTraCIMobility::requestingFromPyServer()
//{
////	using namespace SOCKET;
//	std::string HOST = "127.0.0.1";
//	int PORT = 19999;
//	int MAX_BUFFER = 4096;
//
//	  int connectionFd, rc, index = 0, limit = MAX_BUFFER;
//	  struct sockaddr_in servAddr, localAddr;
//	  char buffer[MAX_BUFFER+1];
//
//
//	  memset(&servAddr, 0, sizeof(servAddr));
//	  servAddr.sin_family = AF_INET;
//	  servAddr.sin_port = htons(PORT);
//	  servAddr.sin_addr.s_addr = inet_addr(HOST.c_str());
//
//	  // Create socket
//	  connectionFd = socket(AF_INET, SOCK_STREAM, 0);
//
//	  /* bind any port number */
//	  localAddr.sin_family = AF_INET;
//	  localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
//	  localAddr.sin_port = htons(0);
//
//	  rc = bind(connectionFd,
//	      (struct sockaddr *) &localAddr, sizeof(localAddr));
//
//	  // Connect to Server
//	  connect(connectionFd,
//	      (struct sockaddr *)&servAddr, sizeof(servAddr));
//
//	  // Send request to Server
//	  std::string req= "ShortestPath_297162704_82660198";
//	  sprintf( buffer, "%s", req.c_str() );
//	  ::send(connectionFd, buffer, strlen(buffer), 0 );
//
////	  printf("Client sent to sever %s\n", buffer);
////
////	  sprintf( buffer, "%s", " " );
////	  send( connectionFd, buffer, strlen(buffer), 0 );
////
////	  printf("Client sent to sever %s\n", buffer);
////
////	  sprintf( buffer, "%f", coordY );
////	  send( connectionFd, buffer, strlen(buffer), 0 );
////
////	   printf("Client sent to sever %s\n", buffer);
//
//	  // Receive data from Server
//	  // sprintf( buffer, "%s", "" );
//	  recv(connectionFd, buffer, MAX_BUFFER, 0);
//	  printf("Client read from Server %s\n", buffer);
//
//	  close(connectionFd);
//
//	  printf("Client closed.\n");
//}

//double GeoTraCIMobility::calculateMETD()
//{
//	double ETA_NP = calculateETA_NP();
//	double ETA_NP_VPA = calculateETA_NP_VPA();
//	double newMETD = 0.0;
//
//	if ((ETA_NP == std::numeric_limits<double>::max()) || (ETA_NP_VPA == std::numeric_limits<double>::max())){
//		newMETD = std::numeric_limits<double>::max();
//	}else{
//		newMETD = ETA_NP + ETA_NP_VPA;
//	}
//	return newMETD;
//}

//void GeoTraCIMobility::updateMETD(double lastETA_NP_VPA)
//{
//	double ETA_NP = calculateETA_NP();
//	double newMETD = 0.0;
//
//	if (! ((ETA_NP == std::numeric_limits<double>::max()) || (lastETA_NP_VPA == std::numeric_limits<double>::max()))){
//		newMETD = ETA_NP + lastETA_NP_VPA;
//		currentMETD = newMETD;
//	}
//}





