/*
 * NetwSession.h
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#ifndef NETWSESSION_H_
#define NETWSESSION_H_

#include "SimpleAddress.h"

class NetwSession {
protected:
	LAddress::L3Type destAddr;
	unsigned long sessionId;
	std::set<unsigned long> storedBndl;
	std::set<unsigned long> delivredToBndl;
	std::set<unsigned long> delivredToVPABndl;

public:
	NetwSession();
	NetwSession(LAddress::L3Type addr, unsigned long sessionId);
	virtual ~NetwSession();
    std::set<unsigned long > getDelivredToBndl() const;
    std::set<unsigned long > getDelivredToVpaBndl() const;
    std::set<unsigned long > getStoredBndl() const;
    void setDelivredToBndl(std::set<unsigned long > delivredToBndl);
    void setDelivredToVpaBndl(std::set<unsigned long > delivredToVpaBndl);
    void setStoredBndl(std::set<unsigned long > storedBndl);

    void insertInDelivredToBndl(unsigned long delivredSerial);
    void insertInDelivredToVpaBndl(unsigned long delivredToVpaSerial);
    void insertInStoredBndl(unsigned long storedSerial);

};

#endif /* NETWSESSION_H_ */
