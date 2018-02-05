/*
 * NetwRoute.h
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#ifndef NETWROUTE_H_
#define NETWROUTE_H_

#include "SimpleAddress.h"
#include "Coord.h"

class NetwRoute {
protected:
	LAddress::L3Type destAddr;
	double destMETD;
	double destDist;
	simtime_t timestamp;
	bool status;
	int nodeType;
	Coord currentPos;
public:
	NetwRoute():destAddr(LAddress::L3NULL), destMETD(0.0), destDist(0.0), timestamp(0), status(false), nodeType(-1), currentPos(Coord()){};
//	NetwRoute(LAddress::L3Type addr, double currentMETD, double currentDist, simtime_t creationTime, bool currentStatus):
//		destAddr(addr), destMETD(currentMETD), destDist(currentDist), timestamp(creationTime), status(currentStatus){};
	NetwRoute(LAddress::L3Type addr, double currentMETD, double currentDist, simtime_t creationTime, bool currentStatus, int nodType, Coord pos);
	virtual ~NetwRoute();

    LAddress::L3Type getDestAddr() const;
    double getDestDist() const;
    double getDestMetd() const;
    simtime_t getTimestamp() const;
    bool isStatus() const;
    void setDestAddr(LAddress::L3Type destAddr);
    void setDestDist(double destDist);
    void setDestMetd(double destMetd);
    void setStatus(bool status);
    void setTimestamp(simtime_t timestamp);
    int getNodeType() const;
    void setNodeType(int nodeType);
    Coord getCurrentPos() const;
    void setCurrentPos(Coord currentPos);
};

#endif /* NETWROUTE_H_ */
