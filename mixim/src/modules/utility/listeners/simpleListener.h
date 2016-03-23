/*
 * simpleListener.h
 *
 *  Created on: Mar 22, 2016
 *      Author: arslan
 */

#ifndef SIMPLELISTENER_H_
#define SIMPLELISTENER_H_

#include <clistener.h>

class simpleListener: public cListener {
public:
	simpleListener();
	virtual ~simpleListener();
//    double getDistance() const
//    {
//        return distance;
//    }
//
//    void setDistance(double distance)
//    {
//        this->distance = distance;
//    }

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, long l);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, double d);

    virtual void finish(cComponent *component, simsignal_t signalID);

    virtual void subscribedTo(cComponent *component, simsignal_t signalID);

    virtual void unsubscribedFrom(cComponent *component, simsignal_t signalID);
protected:
//	double distance;
};

#endif /* SIMPLELISTENER_H_ */
