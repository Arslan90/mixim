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

#ifndef MOBILITY_GEOTRACI_GEOTRACIMOBILITY_H
#define MOBILITY_GEOTRACI_GEOTRACIMOBILITY_H

#include <string>
#include <fstream>
#include <list>
#include <stdexcept>

#include <BaseMobility.h>
#include "mobility/traci/TraCIMobility.h"
#include "FindModule.h"
#include "mobility/traci/TraCIScenarioManager.h"
#include "NearestPoint.h"
//#include <sys/socket.h>
//#include <arpa/inet.h>
//#include "myConstants.h"
#include "PyGraphServerManager.h"

//const double maxDbl = std::numeric_limits<double>::max();

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
class GeoTraCIMobility : public TraCIMobility
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

	    virtual int numInitStages() const {
			return 3;
		}

		GeoTraCIMobility() : TraCIMobility(), isPreInitialized(false) {}
		virtual void initialize(int);
		virtual void finish();

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

		/* ARTURO jusqu'ici mes functions qui travaillent très très bien.  ok I'm doing progress ...  ;)*/

		double commandGetEdgeTravelTime(std::string roadId){
			return getManager()->commandGetEdgeCurrentTravelTime(roadId);
		}

		std::string commandGetLaneId(){
			return getManager()->commandGetLaneId(this->external_id);
		}

		double cmdGetLaneMaxSpeed(std::string laneId){
			return getManager()->cmdGetLaneMaxSpeed(laneId);
		}

    double getCurrentMetd() const;
    NearestPoint getCurrentNp() const;
    void setCurrentMetd(double currentMetd);
	virtual void updateCurrentSector();
	void reComputeMETDAfterReRoute();



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
		cMessage* computeNPMsg;
		mutable TraCIScenarioManager* manager;
		double last_speed;

		PyGraphServerManager* pyManager;


		std::string lastRoadId; /**< updated by nextPosition() */
		std::list<std::string> initialRoute;  /** Edge of the initial route */
		std::list<std::string> remainingRoute;  /** Edge of the current/remaining route */

		int indexRoadId; /** Index of current RoadID in the initial route */
		int indexLastRoadId; /** Index of last RoadID in the initial route */

		NearestPoint currentNP;

		/**
		 * Returns -1 if @param roadId is not found in route @param route, otherwise return its index
		 */
		int roadIndexInRoute(std::string roadId, std::list<std::string> route, int indexLastRoadId, bool canEquals = false);

		bool roadInRoute(std::string roadId, std::list<std::string> route);

		void updateRemainingRoute();

//		double calculateMETD();

//		void updateMETD(double lastETA_NP_VPA);

		void updateMETD(bool reComputeETA_NP_VPA);

		double calculateRmgRoadBestTT();

		double calculateETA_NP(bool includeCurrentRoad);

		double calculateETA_NP_VPA();

		double getEdgeBestTravelTime(std::string edgeId);

		NearestPoint getNearestPoint(int vpaSectorId, std::list<std::string> currentRoute);

		std::string sendRequestToPyServer(std::string buf);

		double currentMETD;

		double currentETA_NP_VPA;

		struct sockaddr_in servAddr, localAddr;

		/*
		 * Map for storing Edge Best Travel Time (BTT) and
		 * by the way avoiding sending multiples requests
		 * to the server
		 */
		std::map<std::string, double> edgesBTTIndex;

		void addRouteToEdgeBTTIndex(std::list<std::string> route);

		std::pair<std::string, double> addEntryToEdgeBTTIndex(std::string edgeAndValue);

		bool withHotSpotMode;

};

class GeoTraCIMobilityAccess
{
	public:
		GeoTraCIMobility* get(cModule* host) {
			GeoTraCIMobility* traci = FindModule<GeoTraCIMobility*>::findSubModule(host);
			ASSERT(traci);
			return traci;
		};
};


#endif

