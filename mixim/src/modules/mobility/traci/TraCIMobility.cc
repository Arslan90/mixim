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

		c_probability = par("c_probability").doubleValue();
		g_probability = par("g_probability").doubleValue();

		my_community ="";
		target_community ="";
		target_edgeID= "";

		g_community = par("g_community").stringValue();
		nbrCommunities = par("nbrCommunities");

		targetFileName = par("targetFileName").stringValue();

		std::ifstream infile(targetFileName.c_str());
		std::string line;

		int nbrLine = 0;
		while (std::getline(infile, line))
		{
			nbrLine++;
		    std::vector<std::string> edges;
		    char* edgeChar = (char*) line.c_str();
		    std::string edgeString ="";
		    char* token = strtok(edgeChar,";");
		    while (token) {
		    	edgeString = std::string(token);
			    edges.push_back(edgeString);
		        token = strtok(NULL, ";");
		    }
		    targetEdgesByCommunity.push_back(edges);
		}

		int x = targetEdgesByCommunity.size();
		if (x != nbrCommunities){
			opp_error("Error nbr communities is not equal to targetFile");
		}
	}
	else if (stage == 1)
	{
		// don't call BaseMobility::initialize(stage) -- our parent will take care to call changePosition later
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

	int my_community_rank = atoi(std::string(my_community).erase(0,1).c_str());
	recordScalar("Community", my_community_rank);
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


	std::list<std::string> edgesOfRoute = commandGetEdgesOfRoute(commandGetRouteId());

	// On starting we are in our community and we will move to the gathering place
	if (my_community == ""){
		my_community = commandGetVehicleTypeId();
	}
	if (target_community == ""){
		target_community =g_community; // Gathering place is the community 7
	}
	// We initialize this value the first time that we insert the vehicle
	if (target_edgeID == ""){
		target_edgeID = edgesOfRoute.back();
	}

	// check if we arrived to the edge before the targeted
	bool arrivedToTarget = false;
	std::list<std::string>::iterator iter = edgesOfRoute.end();
	std::advance(iter, -2);
	std::string lastEdgeBeforeTarget = *(iter);
	if (road_id == lastEdgeBeforeTarget){
		arrivedToTarget = true;
	}

	if (arrivedToTarget){
		// we have to change our target
		double probability = dblrand();

		std::string g = std::string(g_community).erase(0,1);
		std::string t = std::string(target_community).erase(0,1);
		std::string c = std::string(my_community).erase(0,1);
		int g_rank = atoi(g.c_str());
		int t_rank = atoi(t.c_str());
		int c_rank = atoi(c.c_str());
		int newT_rank = -1;

		if (target_community == my_community){
			// if our target is our community we have to choose a different target
			// between Gathering place and other places
			if (probability <= g_probability ){
				// we have choosed to go to Gathering place
				newT_rank = g_rank;
			}else {
				while ((newT_rank == -1) || (newT_rank == t_rank) || (newT_rank == g_rank)){
					newT_rank = intuniform(0,nbrCommunities-1);
//					newT_rank = rand() % nbrCommunities;
				}
			}
		}else {
			// if our target is not our community we have to choose a different target
			// between our community and other places
			if (probability <= c_probability ){
				// we have choosed to go to our community
				newT_rank = c_rank;
			}else {
				while ((newT_rank == -1) || (newT_rank == t_rank) || (newT_rank == g_rank)){
					newT_rank = intuniform(0,nbrCommunities-1);
//					newT_rank = rand() % nbrCommunities;
				}
			}
		}


		std::ostringstream stream;
		stream << "C" << newT_rank;
		target_community = stream.str();

		target_edgeID = targetEdgesByCommunity[t_rank][newT_rank];
		commandChangeTarget(target_edgeID);
	}

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

