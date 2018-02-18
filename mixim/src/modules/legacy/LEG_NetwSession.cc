/*
 * LEG_NetwSession.cpp
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#include "LEG_NetwSession.h"

LEG_NetwSession::LEG_NetwSession() {
	// TODO Auto-generated constructor stub
	this->destAddr = LAddress::L3NULL;
	this->sessionId = 0;
	this->storedBndl = std::set<unsigned long>();
	this->delivredToBndl = std::set<unsigned long>();
	this->delivredToVPABndl = std::set<unsigned long>();
}

LEG_NetwSession::LEG_NetwSession(LAddress::L3Type addr, unsigned long  sessionId)
{
	LEG_NetwSession();
	this->destAddr = addr;
	this->sessionId = sessionId;
}

std::set<unsigned long > LEG_NetwSession::getDelivredToBndl() const
{
    return delivredToBndl;
}

std::set<unsigned long > LEG_NetwSession::getDelivredToVpaBndl() const
{
    return delivredToVPABndl;
}

std::set<unsigned long > LEG_NetwSession::getStoredBndl() const
{
    return storedBndl;
}

void LEG_NetwSession::setDelivredToBndl(std::set<unsigned long > delivredToBndl)
{
    this->delivredToBndl = delivredToBndl;
}

void LEG_NetwSession::setDelivredToVpaBndl(std::set<unsigned long > delivredToVpaBndl)
{
    delivredToVPABndl = delivredToVpaBndl;
}

void LEG_NetwSession::setStoredBndl(std::set<unsigned long > storedBndl)
{
    this->storedBndl = storedBndl;
}

LEG_NetwSession::~LEG_NetwSession() {
	// TODO Auto-generated destructor stub
}

void LEG_NetwSession::insertInDelivredToBndl(unsigned long  delivredSerial)
{
	this->delivredToBndl.insert(delivredSerial);
}

void LEG_NetwSession::insertInDelivredToVpaBndl(unsigned long  delivredToVpaSerial)
{
	this->delivredToVPABndl.insert(delivredToVpaSerial);
}

void LEG_NetwSession::insertInStoredBndl(unsigned long  storedSerial)
{
	this->storedBndl.insert(storedSerial);
}





