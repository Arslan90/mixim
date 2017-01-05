/*
 * NetwRoute.cc
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#include "NetwRoute.h"

//NetwRoute::NetwRoute() {
//	// TODO Auto-generated constructor stub
//
//}

NetwRoute::NetwRoute(LAddress::L3Type addr, double currentMETD, double currentDist, simtime_t creationTime, bool currentStatus, int nodType, Coord pos)
{
	destAddr = addr;
	destMETD = currentMETD;
	destDist = currentDist;
	timestamp = creationTime;
	status = currentStatus;
	if (destMETD < 0){
		opp_error("METD for this route is negative");
	}

	if (destDist < 0){
		opp_error("Dist_NP_VPA for this route is negative");
	}
	nodeType = nodType;
	currentPos = pos;
}

LAddress::L3Type NetwRoute::getDestAddr() const
{
    return destAddr;
}

double NetwRoute::getDestDist() const
{
    return destDist;
}

double NetwRoute::getDestMetd() const
{
    return destMETD;
}

simtime_t NetwRoute::getTimestamp() const
{
    return timestamp;
}

bool NetwRoute::isStatus() const
{
    return status;
}

void NetwRoute::setDestAddr(LAddress::L3Type destAddr)
{
    this->destAddr = destAddr;
}

void NetwRoute::setDestDist(double destDist)
{
    this->destDist = destDist;
}

void NetwRoute::setDestMetd(double destMetd)
{
    destMETD = destMetd;
}

void NetwRoute::setStatus(bool status)
{
    this->status = status;
}

int NetwRoute::getNodeType() const
{
    return nodeType;
}

Coord NetwRoute::getCurrentPos() const
{
    return currentPos;
}

void NetwRoute::setCurrentPos(Coord currentPos)
{
    this->currentPos = currentPos;
}

void NetwRoute::setNodeType(int nodeType)
{
    this->nodeType = nodeType;
}

void NetwRoute::setTimestamp(simtime_t timestamp)
{
    this->timestamp = timestamp;
}

NetwRoute::~NetwRoute() {
	// TODO Auto-generated destructor stub
}

