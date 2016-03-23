/*
 * simpleListener.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: arslan
 */

#include "simpleListener.h"

simpleListener::simpleListener() {
//	distance = 0;

}

simpleListener::~simpleListener() {
	// TODO Auto-generated destructor stub
}

void simpleListener::receiveSignal(cComponent *source, simsignal_t signalID, long  l)
{
}



void simpleListener::receiveSignal(cComponent *source, simsignal_t signalID, double d)
{
//	distance = d;
}

void simpleListener::finish(cComponent *component, simsignal_t signalID)
{
}



void simpleListener::subscribedTo(cComponent *component, simsignal_t signalID)
{
}



void simpleListener::unsubscribedFrom(cComponent *component, simsignal_t signalID)
{
}





