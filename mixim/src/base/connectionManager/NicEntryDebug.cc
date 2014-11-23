/* -*- mode:c++ -*- ********************************************************
 * file:        NicEntryDebug.cc
 *
 * author:      Daniel Willkomm
 *
 * copyright:   (C) 2005 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 * description: Class to store information about a nic for the
 *              ConnectionManager module
 **************************************************************************/

#include "NicEntryDebug.h"

#include <cassert>

#include "ChannelAccess.h"
#include "FindModule.h"
#include "BaseLayer.h"
#include "BaseNetwLayer.h"
//#include <../../modules/mac/opprouting/Mac1609_4_opp.h>
//#include "../../modules/netw/opprouting/ProphetV2.h"



#ifndef nicEV
#define nicEV (ev.isDisabled()||!coreDebug) ? ev : ev << "NicEntry: "
#endif

using std::endl;

void NicEntryDebug::connectTo(NicEntry* other) {
	// if no older neighbor, then this is a first connection
	if (!existingNeighborhood()){
		// ProphetV2::NEWLY_CONNECTED = 24500
		prepareControlMsg(24500);
	}
//	if (!existingNeighborhood()){
//		if (nicPtr->findSubmodule("mac1609_4_opp")!=-1){
//			cModule* module = nicPtr->getSubmodule("mac1609_4_opp");
//			if (module !=NULL){
//				Mac1609_4_opp *mac = check_and_cast<Mac1609_4_opp*>(module);
//				mac->neighborhoodNotifier("connection");
//			}
//		}
//	}

	nicEV<<"connecting nic #"<<nicId<< " and #"<<other->nicId<<endl;

	NicEntryDebug* otherNic = (NicEntryDebug*) other;
//	cModule* netwModule =

	cGate *localoutgate = requestOutGate();
	localoutgate->connectTo(otherNic->requestInGate());
	outConns[other] = localoutgate->getPathStartGate();

	cModule* otherNode = other->nicPtr->getParentModule();
	int destAddr =0;
	if (otherNode->findSubmodule("netw")!=-1){
		cModule* module = otherNode->getSubmodule("netw");
		BaseNetwLayer *netw = check_and_cast<BaseNetwLayer*>(module);
		destAddr = netw->getMyNetwAddr();
	}
	// for each new neighbor, we have to start a new prophet information exchange
	// ProphetV2::NEW_NEIGHBOR = 24510
	prepareControlMsg(24510, destAddr);
}


void NicEntryDebug::disconnectFrom(NicEntry* other) {
	nicEV<<"disconnecting nic #"<<nicId<< " and #"<<other->nicId<<endl;

	NicEntryDebug* otherNic = (NicEntryDebug*) other;

	cModule* otherNode = other->nicPtr->getParentModule();

	int destAddr =0;
	if (otherNode->findSubmodule("netw")!=-1){
		cModule* module = otherNode->getSubmodule("netw");
		BaseNetwLayer *netw = check_and_cast<BaseNetwLayer*>(module);
		destAddr = netw->getMyNetwAddr();
	}
	// for each neighbor, we have to send a control message when disconnecting
	// ProphetV2::NEW_NEIGHBOR_GONE = 24530
	prepareControlMsg(24530, destAddr);

	//search the connection in the outConns list
	GateList::iterator p = outConns.find(other);
	//no need to check whether entry is valid; is already check by ConnectionManager isConnected
	//get the hostGate
	//order is phyGate->nicGate->hostGate
	cGate* hostGate = p->second->getNextGate()->getNextGate();

	// release local out gate
	freeOutGates.push_back(hostGate);

	// release remote in gate
	otherNic->freeInGates.push_back(hostGate->getNextGate());

	//reset gates
	//hostGate->getNextGate()->connectTo(0);
	hostGate->disconnect();

	//delete the connection
	outConns.erase(p);

//	if (!existingNeighborhood()){
//		if (nicPtr->findSubmodule("mac1609_4_opp")!=-1){
//			cModule* module = nicPtr->getSubmodule("mac1609_4_opp");
//			if (module !=NULL){
//				Mac1609_4_opp *mac = check_and_cast<Mac1609_4_opp*>(module);
//				mac->neighborhoodNotifier("disconnection");
//			}
//		}
//	}

	/*
	 * *after deleting the connection, we check if there is no current neighbor
	 * if it the case we have to send a control message to notify the newt layer
	 */
	if (!existingNeighborhood()){
		// ProphetV2::NO_NEIGHBOR_AND_DISCONNECTED = 24520
		prepareControlMsg(24520);
	}

}

int NicEntryDebug::collectGates(const char* pattern, GateStack& gates)
{
	cModule* host = nicPtr->getParentModule();
	int i = 1;
	char gateName[20];
	//create the unique name for the gate (composed of the nic module id and a counter)
	sprintf(gateName, pattern, nicId, i);
	while(host->hasGate(gateName))
	{
		cGate* hostGate = host->gate(gateName);
		if(hostGate->isConnectedOutside()) {
			opp_error("Gate %s is still connected but not registered with this "
					  "NicEntry. Either the last NicEntry for this NIC did not "
					  "clean up correctly or another gate creation module is "
					  "interfering with this one!", gateName);
		}
		assert(hostGate->isConnectedInside());
		gates.push_back(hostGate);

		++i;
		sprintf(gateName, pattern, nicId, i);
	}

	return i - 1;
}

void NicEntryDebug::collectFreeGates()
{
	if(!checkFreeGates)
		return;

	inCnt = collectGates("in%d-%d", freeInGates);
	nicEV << "found " << inCnt << " already existing usable in-gates." << endl;


	outCnt = collectGates("out%d-%d", freeOutGates);
	nicEV << "found " << inCnt << " already existing usable out-gates." << endl;

	checkFreeGates = false;
}


cGate* NicEntryDebug::requestInGate(void) {
	collectFreeGates();

	// gate of the host
	cGate *hostGate;

	if (!freeInGates.empty()) {
		hostGate = freeInGates.back();
		freeInGates.pop_back();
	} else {
		char gateName[20];

		// we will have one more in gate
		++inCnt;

		//get a unique name for the gate (composed of the nic module id and a counter)
		sprintf(gateName, "in%d-%d", nicId, inCnt);

		// create a new gate for the host module
		nicPtr->getParentModule()->addGate(gateName, cGate::INPUT);
		hostGate = nicPtr->getParentModule()->gate(gateName);

		// gate of the nic
		cGate *nicGate;

		// create a new gate for the nic module
		nicPtr->addGate(gateName, cGate::INPUT);
		nicGate = nicPtr->gate(gateName);

		// connect the hist gate with the nic gate
		hostGate->connectTo(nicGate);

		// pointer to the phy module
		ChannelAccess* phyModule;
		// gate of the phy module
		cGate *phyGate;

		// to avoid unnecessary dynamic_casting we check for a "phy"-named submodule first
		if ((phyModule = dynamic_cast<ChannelAccess *> (nicPtr->getSubmodule("phy"))) == NULL)
			phyModule = FindModule<ChannelAccess*>::findSubModule(nicPtr);
		assert(phyModule != 0);

		// create a new gate for the phy module
		phyModule->addGate(gateName, cGate::INPUT);
		phyGate = phyModule->gate(gateName);

		// connect the nic gate (the gate of the compound module) to
		// a "real" gate -- the gate of the phy module
		nicGate->connectTo(phyGate);
	}

	return hostGate;
}

cGate* NicEntryDebug::requestOutGate(void) {
	collectFreeGates();

	// gate of the host
	cGate *hostGate;

	if (!freeOutGates.empty()) {
		hostGate = freeOutGates.back();
		freeOutGates.pop_back();
	} else {
		char gateName[20];

		// we will have one more out gate
		++outCnt;

		//get a unique name for the gate (composed of the nic module id and a counter)
		sprintf(gateName, "out%d-%d", nicId, outCnt);

		// create a new gate for the host module
		nicPtr->getParentModule()->addGate(gateName, cGate::OUTPUT);
		hostGate = nicPtr->getParentModule()->gate(gateName);

		// gate of the nic
		cGate *nicGate;
		// create a new gate for the nic module
		nicPtr->addGate(gateName, cGate::OUTPUT);
		nicGate = nicPtr->gate(gateName);

		// connect the hist gate with the nic gate
		nicGate->connectTo(hostGate);

		// pointer to the phy module
		ChannelAccess* phyModule;
		// gate of the phy module
		cGate *phyGate;

		// to avoid unnecessary dynamic_casting we check for a "phy"-named submodule first
		if ((phyModule = dynamic_cast<ChannelAccess *> (nicPtr->getSubmodule("phy"))) == NULL)
			phyModule = FindModule<ChannelAccess*>::findSubModule(nicPtr);
		assert(phyModule != 0);

		// create a new gate for the phy module
		phyModule->addGate(gateName, cGate::OUTPUT);
		phyGate = phyModule->gate(gateName);

		// connect the nic gate (the gate of the compound module) to
		// a "real" gate -- the gate of the phy module
		phyGate->connectTo(nicGate);
	}

	return hostGate;
}

void NicEntryDebug::prepareControlMsg(short controlKind, int destAddr)
{
	if (nicPtr->findSubmodule("mac1609_4_opp")!=-1){
		cModule* module = nicPtr->getSubmodule("mac1609_4_opp");
//		if (module !=NULL){
//			Mac1609_4_opp *mac = check_and_cast<Mac1609_4_opp*>(module);
//			mac->neighborhoodNotifier(controlKind);
//		}
		if (module !=NULL){
			BaseLayer *mac = check_and_cast<BaseLayer*>(module);
			mac->enForceExecution(controlKind, destAddr);
//			mac->sendControlUp(msg);
//			mac->neighborhoodNotifier(controlKind);
		}
//		cMessage *msg = new cMessage();
//		msg->setKind(controlKind);
//		sendControlUp(msg);
	}
}
