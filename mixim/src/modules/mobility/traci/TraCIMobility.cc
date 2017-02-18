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

#include "mobility/traci/TraCIMobility.h"

Define_Module(TraCIMobility);

namespace {
	const double MY_INFINITY = (std::numeric_limits<double>::has_infinity ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::max());

	double roadIdAsDouble(std::string road_id) {
		std::istringstream iss(road_id);
		double d;
		if (!(iss >> d)) return MY_INFINITY;
		return d;
	}
}

void TraCIMobility::Statistics::initialize()
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

void TraCIMobility::Statistics::watch(cSimpleModule& )
{
	WATCH(totalTime);
	WATCH(minSpeed);
	WATCH(maxSpeed);
	WATCH(totalDistance);
}

void TraCIMobility::Statistics::recordScalars(cSimpleModule& module)
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

void TraCIMobility::initialize(int stage)
{
	if (stage == 0)
	{
		BaseMobility::initialize(stage);

		debug = par("debug");
		accidentCount = par("accidentCount");

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
		updateCurrentSector();
	}
	else
	{
		BaseMobility::initialize(stage);
	}

}

void TraCIMobility::finish()
{
	statistics.stopTime = simTime();

	statistics.recordScalars(*this);

	cancelAndDelete(startAccidentMsg);
	cancelAndDelete(stopAccidentMsg);

	isPreInitialized = false;
}

void TraCIMobility::handleSelfMsg(cMessage *msg)
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
	}
}

void TraCIMobility::preInitialize(std::string external_id, const Coord& position, std::string road_id, double speed, double angle)
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

	isPreInitialized = true;
}

void TraCIMobility::nextPosition(const Coord& position, std::string road_id, double speed, double angle)
{
	if (debug) EV << "nextPosition " << position.x << " " << position.y << " " << road_id << " " << speed << " " << angle << std::endl;
	isPreInitialized = false;
	nextPos = position;
	this->road_id = road_id;
	this->speed = speed;
	this->angle = angle;
	changePosition();
	updateCurrentSector();
}

void TraCIMobility::changePosition()
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

void TraCIMobility::updateDisplayString() {
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

void TraCIMobility::fixIfHostGetsOutside()
{
	Coord dummy = move.getStartPos();
	double dum;

	handleIfOutside( RAISEERROR, dummy, dummy, dummy, dum);
}

void TraCIMobility::simulateAccident()
{
	Enter_Method_Silent("simulateAccident()");
	if (accidentCount > 0){
		startAccidentMsg = new cMessage("scheduledAccident");
		stopAccidentMsg = new cMessage("scheduledAccidentResolved");
		scheduleAt(simTime(), startAccidentMsg);
	}
}

double TraCIMobility::calculateCO2emission(double v, double a) const {
	// Calculate CO2 emission parameters according to:
	// Cappiello, A. and Chabini, I. and Nam, E.K. and Lue, A. and Abou Zeid, M., "A statistical model of vehicle emissions and fuel consumption," IEEE 5th International Conference on Intelligent Transportation Systems (IEEE ITSC), pp. 801-809, 2002

	double A = 1000 * 0.1326; // W/m/s
	double B = 1000 * 2.7384e-03; // W/(m/s)^2
	double C = 1000 * 1.0843e-03; // W/(m/s)^3
	double M = 1325.0; // kg

	// power in W
	double P_tract = A*v + B*v*v + C*v*v*v + M*a*v; // for sloped roads: +M*g*sin_theta*v

	/*
	// "Category 7 vehicle" (e.g. a '92 Suzuki Swift)
	double alpha = 1.01;
	double beta = 0.0162;
	double delta = 1.90e-06;
	double zeta = 0.252;
	double alpha1 = 0.985;
	*/

	// "Category 9 vehicle" (e.g. a '94 Dodge Spirit)
	double alpha = 1.11;
	double beta = 0.0134;
	double delta = 1.98e-06;
	double zeta = 0.241;
	double alpha1 = 0.973;

	if (P_tract <= 0) return alpha1;
	return alpha + beta*v*3.6 + delta*v*v*v*(3.6*3.6*3.6) + zeta*a*v;
}

void TraCIMobility::initScenarioType() {

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
}

int TraCIMobility::getCurrentSector() const
{
    return currentSector;
}

void TraCIMobility::updateCurrentSector()
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

		//if I noticed a change of sectorID reset everything. Also gives the current counter o'course.
		//Note: this counter is complemented with the counter in received messages by VPAs or Vehicles.
		if (inSector != currentSector){
			currentSector= inSector; //update Veh sector transit
		}else {
			oldSector = currentSector;
		}
	}
}

