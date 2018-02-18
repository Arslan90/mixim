/*
 * NetwSession.cpp
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#include "NetwSession.h"

NetwSession::NetwSession() {
	// TODO Auto-generated constructor stub
	init();
}

NetwSession::NetwSession(LAddress::L3Type addr, unsigned long  sessionId)
{
	init();
	this->destAddr = addr;
	this->sessionId = sessionId;
}

void NetwSession::init(){
	this->destAddr = LAddress::L3NULL;
	this->sessionId = 0;
	this->storedBundles = std::set<unsigned long>();
	this->storedAcks = std::map<unsigned long, double>();
	this->storedCustody = std::map<unsigned long, double>();
}


NetwSession::~NetwSession() {
	// TODO Auto-generated destructor stub
}

void NetwSession::updateStoredBundle(std::set<unsigned long > bundleToStore)
{
	for (std::set<unsigned long >::iterator it = bundleToStore.begin(); it != bundleToStore.end(); it++){
		this->storedBundles.insert(*it);
	}
}

void NetwSession::updateStoredAck(std::map<unsigned long ,double> ackToStore)
{
	for (std::map<unsigned long ,double>::iterator it = ackToStore.begin(); it != ackToStore.end(); it++){
		unsigned long serial = it->first;
		double expTime = it->second;
		std::map<unsigned long ,double>::iterator it2 = this->storedAcks.find(serial);
		if (it2 == storedAcks.end()){
			storedAcks.insert(std::pair<unsigned long ,double>(serial, expTime));
		}else{
			double previousExpTime = it2->second;
			if (expTime > previousExpTime){
				storedAcks[serial] = expTime;
			}
		}
	}
}

void NetwSession::updateStoredCustody(std::map<unsigned long ,double> custodyToStore)
{
	for (std::map<unsigned long ,double>::iterator it = custodyToStore.begin(); it != custodyToStore.end(); it++){
		unsigned long serial = it->first;
		double expTime = it->second;
		std::map<unsigned long ,double>::iterator it2 = this->storedCustody.find(serial);
		if (it2 == storedCustody.end()){
			storedCustody.insert(std::pair<unsigned long ,double>(serial, expTime));
		}else{
			double previousExpTime = it2->second;
			if (expTime > previousExpTime){
				storedCustody[serial] = expTime;
			}
		}
	}
}

bool NetwSession::existInStoredBundle(unsigned long  serial)
{
	bool found = false;
	std::set<unsigned long>::iterator it = storedBundles.find(serial);
	if (it != storedBundles.end()){
		found = true;
	}
	return found;
}



bool NetwSession::existInStoredAck(unsigned long  serial)
{
	bool found = false;
	std::map<unsigned long,double>::iterator it = storedAcks.find(serial);
	if (it != storedAcks.end()){
		found = true;
	}
	return found;
}



bool NetwSession::existInStoredCustody(unsigned long  serial)
{
	bool found = false;
	std::map<unsigned long,double>::iterator it = storedCustody.find(serial);
	if (it != storedCustody.end()){
		found = true;
	}
	return found;
}



bool NetwSession::existInStoredBundleOrAck(unsigned long  serial)
{
	return (existInStoredBundle(serial) || existInStoredAck(serial));
}





//void NetwSession::insertInDelivredToBndl(unsigned long  delivredSerial)
//{
//	this->delivredToBndl.insert(delivredSerial);
//}
//
//void NetwSession::insertInDelivredToVpaBndl(unsigned long  delivredToVpaSerial)
//{
//	this->delivredToVPABndl.insert(delivredToVpaSerial);
//}






