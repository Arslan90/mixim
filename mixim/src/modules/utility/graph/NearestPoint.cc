/*
 * NearestPoint.cc
 *
 *  Created on: May 29, 2016
 *      Author: arslan
 */

#include "NearestPoint.h"
#include "limits"

NearestPoint::NearestPoint() {
	// TODO Auto-generated constructor stub
	NP_edgeTo = " ";
	NP_node = " ";
	NP_edgefrom = " ";
	VPA_node = " ";
	Route_NP_VPA = std::list<std::string>();
	Distance_NP_VPA = std::numeric_limits<double>::max();
	METD = std::numeric_limits<double>::max();
	ETA_NP_VPA = std::numeric_limits<double>::max();
	initialized = false;
	valid = false;
}

NearestPoint::NearestPoint(std::string edgeToNP, std::string nodeNP, std::string edgeFromNP, std::string nodeVPA, std::list<std::string> Route, double dist)
{
	NP_edgeTo = edgeToNP;
	NP_node = nodeNP;
	NP_edgefrom = edgeFromNP;
	VPA_node = nodeVPA;
	Route_NP_VPA = Route;
	Distance_NP_VPA = dist;
	METD = std::numeric_limits<double>::max();
	ETA_NP_VPA = std::numeric_limits<double>::max();
	initialized = true;
	if ((NP_node != " ") and (NP_node != "ND")){
		valid = true;
	}else{
		valid = false;
	}

}

double NearestPoint::getDistanceNpVpa() const
{
    return Distance_NP_VPA;
}

double NearestPoint::getMetd() const
{
    return METD;
}

std::string NearestPoint::getNpEdgeTo() const
{
    return NP_edgeTo;
}

std::string NearestPoint::getNpEdgefrom() const
{
    return NP_edgefrom;
}

std::string NearestPoint::getNpNode() const
{
    return NP_node;
}

std::list<std::string> NearestPoint::getRouteNpVpa() const
{
    return Route_NP_VPA;
}

std::string NearestPoint::getVpaNode() const
{
    return VPA_node;
}

bool NearestPoint::isInitialized() const
{
    return initialized;
}

bool NearestPoint::isValid() const
{
	if ((initialized) && (NP_node != " ")){
		return true;
	}else{
		return false;
	}
}

void NearestPoint::setDistanceNpVpa(double distanceNpVpa)
{
    Distance_NP_VPA = distanceNpVpa;
}

void NearestPoint::setInitialized(bool initialized)
{
    this->initialized = initialized;
}

void NearestPoint::setMetd(double metd)
{
    METD = metd;
}

void NearestPoint::setNpEdgeTo(std::string npEdgeTo)
{
    NP_edgeTo = npEdgeTo;
}

void NearestPoint::setNpEdgefrom(std::string npEdgefrom)
{
    NP_edgefrom = npEdgefrom;
}

void NearestPoint::setNpNode(std::string npNode)
{
    NP_node = npNode;
}

void NearestPoint::setRouteNpVpa(std::list<std::string> routeNpVpa)
{
    Route_NP_VPA = routeNpVpa;
}

void NearestPoint::setValid(bool valid)
{
    this->valid = valid;
}

double NearestPoint::getEtaNpVpa() const
{
    return ETA_NP_VPA;
}

void NearestPoint::setEtaNpVpa(double etaNpVpa)
{
    ETA_NP_VPA = etaNpVpa;
}

void NearestPoint::setVpaNode(std::string vpaNode)
{
    VPA_node = vpaNode;
}

NearestPoint::~NearestPoint() {
	// TODO Auto-generated destructor stub
}

