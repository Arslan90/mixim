/*
 * NearestPoint.h
 *
 *  Created on: May 29, 2016
 *      Author: arslan
 */

#ifndef NEARESTPOINT_H_
#define NEARESTPOINT_H_

#include "string"
#include "list"

class NearestPoint {
protected:
    std::string NP_edgeTo;
    std::string NP_node;
    std::string NP_edgefrom;
    std::string VPA_node;
    std::list<std::string> Route_NP_VPA;
    double Distance_NP_VPA;
    double METD;
    double ETA_NP_VPA;
    bool initialized;
    bool valid;
public:
	NearestPoint();
	NearestPoint(std::string edgeToNP, std::string nodeNP, std::string edgeFromNP, std::string nodeVPA, std::list<std::string> Route, double dist );
	virtual ~NearestPoint();
    double getDistanceNpVpa() const;
    double getMetd() const;
    std::string getNpEdgeTo() const;
    std::string getNpEdgefrom() const;
    std::string getNpNode() const;
    std::list<std::string> getRouteNpVpa() const;
    std::string getVpaNode() const;
    bool isInitialized() const;
    bool isValid() const;
    void setDistanceNpVpa(double distanceNpVpa);
    void setInitialized(bool initialized);
    void setMetd(double metd);
    void setNpEdgeTo(std::string npEdgeTo);
    void setNpEdgefrom(std::string npEdgefrom);
    void setNpNode(std::string npNode);
    void setRouteNpVpa(std::list<std::string> routeNpVpa);
    void setValid(bool valid);
    void setVpaNode(std::string vpaNode);
    double getEtaNpVpa() const;
    void setEtaNpVpa(double etaNpVpa);
};

#endif /* NEARESTPOINT_H_ */
