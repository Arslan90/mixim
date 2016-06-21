/*
 * NetwRoute.h
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#ifndef NETWROUTE_H_
#define NETWROUTE_H_

#include "SimpleAddress.h"

class NetwRoute {
protected:
	LAddress::L3Type destAddr;
	double destMETD;
	double destDist;
	simtime_t timestamp;
	bool status;
	int nodeType;
public:
	NetwRoute():destAddr(LAddress::L3NULL), destMETD(0.0), destDist(0.0), timestamp(0), status(false), nodeType(-1){};
//	NetwRoute(LAddress::L3Type addr, double currentMETD, double currentDist, simtime_t creationTime, bool currentStatus):
//		destAddr(addr), destMETD(currentMETD), destDist(currentDist), timestamp(creationTime), status(currentStatus){};
	NetwRoute(LAddress::L3Type addr, double currentMETD, double currentDist, simtime_t creationTime, bool currentStatus, int nodType);
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
};

#endif /* NETWROUTE_H_ */
