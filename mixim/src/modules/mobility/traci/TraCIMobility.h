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

#ifndef MOBILITY_TRACI_TRACIMOBILITY_H
#define MOBILITY_TRACI_TRACIMOBILITY_H

#include <string>
#include <fstream>
#include <list>
#include <stdexcept>

#include <BaseMobility.h>
#include "FindModule.h"
#include "mobility/traci/TraCIScenarioManager.h"

/**
 * @brief
 * TraCIMobility is a mobility module for hosts controlled by TraCIScenarioManager.
 * It receives position and state updates from an external module and updates
 * the parent module accordingly.
 * See NED file for more info.
 *
 * @ingroup mobility
 * @author Christoph Sommer
 */
class TraCIMobility : public BaseMobility
{
	public:
		class Statistics {
			public:
				double firstRoadNumber; /**< for statistics: number of first road we encountered (if road id can be expressed as a number) */
				simtime_t startTime; /**< for statistics: start time */
				simtime_t totalTime; /**< for statistics: total time travelled */
				simtime_t stopTime; /**< for statistics: stop time */
				double minSpeed; /**< for statistics: minimum value of currentSpeed */
				double maxSpeed; /**< for statistics: maximum value of currentSpeed */
				double totalDistance; /**< for statistics: total distance travelled */
				double totalCO2Emission; /**< for statistics: total CO2 emission */

				void initialize();
				void watch(cSimpleModule& module);
				void recordScalars(cSimpleModule& module);
		};

		TraCIMobility() : BaseMobility(), isPreInitialized(false) {}
		virtual void initialize(int);
		virtual void finish();

		virtual void simulateAccident();

		virtual void handleSelfMsg(cMessage *msg);
		virtual void preInitialize(std::string external_id, const Coord& position, std::string road_id = "", double speed = -1, double angle = -1);
		virtual void nextPosition(const Coord& position, std::string road_id = "", double speed = -1, double angle = -1);
		virtual void changePosition();
		virtual void updateDisplayString();
		virtual void setExternalId(std::string external_id) {
			this->external_id = external_id;
		}
		virtual std::string getExternalId() const {
			if (external_id == "") throw cRuntimeError("TraCIMobility::getExternalId called with no external_id set yet");
			return external_id;
		}
		virtual Coord getPositionAt(const simtime_t& t) const {
			return move.getPositionAt(t) ;
		}
		virtual std::string getRoadId() const {
			if (road_id == "") throw cRuntimeError("TraCIMobility::getRoadId called with no road_id set yet");
			return road_id;
		}
		virtual double getSpeed() const {
			if (speed == -1) throw cRuntimeError("TraCIMobility::getSpeed called with no speed set yet");
			return speed;
		}
		/**
		 * returns angle in rads, 0 being east, with -M_PI <= angle < M_PI.
		 */
		virtual double getAngleRad() const {
			if (angle == M_PI) throw cRuntimeError("TraCIMobility::getAngleRad called with no angle set yet");
			return angle;
		}
		virtual TraCIScenarioManager* getManager() const {
			if (!manager) manager = TraCIScenarioManagerAccess().get();
			return manager;
		}
		void commandSetSpeedMode(int32_t bitset) {
			getManager()->commandSetSpeedMode(getExternalId(), bitset);
		}
		void commandSetSpeed(double speed) {
			getManager()->commandSetSpeed(getExternalId(), speed);
		}
		void commandChangeRoute(std::string roadId, double travelTime) {
			getManager()->commandChangeRoute(getExternalId(), roadId, travelTime);
		}
		double commandDistanceRequest(Coord position1, Coord position2, bool returnDrivingDistance) {
			return getManager()->commandDistanceRequest(position1, position2, returnDrivingDistance);
		}
		void commandStopNode(std::string roadId, double pos, uint8_t laneid, double radius, double waittime) {
			return getManager()->commandStopNode(getExternalId(), roadId, pos, laneid, radius, waittime);
		}
		std::list<std::string> commandGetPolygonIds() {
			return getManager()->commandGetPolygonIds();
		}
		std::string commandGetPolygonTypeId(std::string polyId) {
			return getManager()->commandGetPolygonTypeId(polyId);
		}
		std::list<Coord> commandGetPolygonShape(std::string polyId) {
			return getManager()->commandGetPolygonShape(polyId);
		}
		void commandSetPolygonShape(std::string polyId, std::list<Coord> points) {
			getManager()->commandSetPolygonShape(polyId, points);
		}
		bool commandAddVehicle(std::string vehicleId, std::string vehicleTypeId, std::string routeId, std::string laneId, double emitPosition, double emitSpeed) {
			return getManager()->commandAddVehicle(vehicleId, vehicleTypeId, routeId, laneId, emitPosition, emitSpeed);
		}

		//ARTURO ICI mes fonctions/COMMANDS TraCI que j'ai besoin.
		std::list<std::string> commandGetVehiclesIds() {
			return getManager()->commandGetVehiclesIds();  // the getExternalId() retrieves the vehicleId
		}
		std::list<std::string> commandGetArcIds() {
			return getManager()->commandGetArcIds();
		}
		double commandGetArcCO2(std::string polyId) {
			return getManager()->commandGetArcCO2(polyId);
		}
		double commandGetArcOccupancy(std::string polyId) {
			return getManager()->commandGetArcOccupancy(polyId);
		}
		std::list<std::string> commandGetSingleVehicleRoutes() {
			return getManager()->commandGetSingleVehicleRoutes(getExternalId()); // the getExternalId() retrieves the vehicleId
		}
		double cmdGetVehiclelanePosition() {
			return getManager()->cmdGetVehiclelanePosition(getExternalId());
		}
		double cmdGetLaneLength(std::string laneId) {
			return getManager()->cmdGetLaneLength(laneId);
		}

		/* ARTURO jusqu'ici mes functions qui travaillent très très bien.  ok I'm doing progress ...  ;)*/

		// My Function
		void commandChangeRouteById(std::string routeId) {
				getManager()->commandChangeRouteById(getExternalId(), routeId);
		}
		std::list<std::string> commandGetRouteIds() {
			return getManager()->commandGetRouteIds();
		}

		std::string commandGetRouteId() {
			return getManager()->commandGetRouteId(getExternalId());
		}

		std::list<std::string> commandGetEdgesOfRoute(std::string routeId) {
			return getManager()->commandGetEdgesOfRoute(routeId);
		}
	    virtual int getCurrentSector() const;
		virtual void updateCurrentSector();

	protected:
		bool debug; /**< whether to emit debug messages */
		int accidentCount; /**< number of accidents */

		cOutVector currentPosXVec; /**< vector plotting posx */
		cOutVector currentPosYVec; /**< vector plotting posy */
		cOutVector currentSpeedVec; /**< vector plotting speed */
		cOutVector currentAccelerationVec; /**< vector plotting acceleration */
		cOutVector currentCO2EmissionVec; /**< vector plotting current CO2 emission */

		Statistics statistics; /**< everything statistics-related */

		bool isPreInitialized; /**< true if preInitialize() has been called immediately before initialize() */

		std::string external_id; /**< updated by setExternalId() */

		simtime_t lastUpdate; /**< updated by nextPosition() */
		Coord nextPos; /**< updated by nextPosition() */
		std::string road_id; /**< updated by nextPosition() */
		double speed; /**< updated by nextPosition() */
		double angle; /**< updated by nextPosition() */

		cMessage* startAccidentMsg;
		cMessage* stopAccidentMsg;
		mutable TraCIScenarioManager* manager;
		double last_speed;

	    /**
		 * Scenario Model Type (Cologne -Koln- like) or Free
		 */
	    enum t_scenarioType{ Free = 1, Sector = 2};
	    /*
		 * Variable for specifying Scenario Model Type
		 */
	    t_scenarioType scenarioModel;
	    // Parameters related to Sector Mode
	    int oldSector;
	    int currentSector;
	    int rowSectorGrid;
	    int colSectorGrid;
	    double sectorSizeX;
	    double sectorSizeY;
	    bool useNegativeValues; // Allow us the use of negative values for Offset - Coord(0.0) of sector[0]
	    double sectorOffsetX; // Traci Coord X for - Coord(0.0) of sector[0]
	    double sectorOffsetY; // Traci Coord Y for - Coord(0.0) of sector[0]

		virtual void initScenarioType();

		virtual void fixIfHostGetsOutside(); /**< called after each read to check for (and handle) invalid positions */

		/**
		 * Returns the amount of CO2 emissions in grams/second, calculated for an average Car
		 * @param v speed in m/s
		 * @param a acceleration in m/s^2
		 * @returns emission in g/s
		 */
		double calculateCO2emission(double v, double a) const;
};

class TraCIMobilityAccess
{
	public:
		TraCIMobility* get(cModule* host) {
			TraCIMobility* traci = FindModule<TraCIMobility*>::findSubModule(host);
			ASSERT(traci);
			return traci;
		};
};


#endif

