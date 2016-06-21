/*
 * NetwSession.cpp
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#include "NetwSession.h"

NetwSession::NetwSession() {
	// TODO Auto-generated constructor stub
	this->destAddr = LAddress::L3NULL;
	this->sessionId = 0;
	this->storedBndl = std::set<unsigned long>();
	this->delivredToBndl = std::set<unsigned long>();
	this->delivredToVPABndl = std::set<unsigned long>();
}

NetwSession::NetwSession(LAddress::L3Type addr, unsigned long  sessionId)
{
	NetwSession();
	this->destAddr = addr;
	this->sessionId = sessionId;
}

std::set<unsigned long > NetwSession::getDelivredToBndl() const
{
    return delivredToBndl;
}

std::set<unsigned long > NetwSession::getDelivredToVpaBndl() const
{
    return delivredToVPABndl;
}

std::set<unsigned long > NetwSession::getStoredBndl() const
{
    return storedBndl;
}

void NetwSession::setDelivredToBndl(std::set<unsigned long > delivredToBndl)
{
    this->delivredToBndl = delivredToBndl;
}

void NetwSession::setDelivredToVpaBndl(std::set<unsigned long > delivredToVpaBndl)
{
    delivredToVPABndl = delivredToVpaBndl;
}

void NetwSession::setStoredBndl(std::set<unsigned long > storedBndl)
{
    this->storedBndl = storedBndl;
}

NetwSession::~NetwSession() {
	// TODO Auto-generated destructor stub
}

void NetwSession::insertInDelivredToBndl(unsigned long  delivredSerial)
{
	this->delivredToBndl.insert(delivredSerial);
}

void NetwSession::insertInDelivredToVpaBndl(unsigned long  delivredToVpaSerial)
{
	this->delivredToVPABndl.insert(delivredToVpaSerial);
}

void NetwSession::insertInStoredBndl(unsigned long  storedSerial)
{
	this->storedBndl.insert(storedSerial);
}





