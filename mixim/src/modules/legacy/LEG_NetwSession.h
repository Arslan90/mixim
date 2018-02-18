/*
 * LEG_NetwSession.h
 *
 *  Created on: Jun 1, 2016
 *      Author: arslan
 */

#ifndef LEG_NETWSESSION_H_
#define LEG_NETWSESSION_H_

#include "SimpleAddress.h"

class LEG_NetwSession {
protected:
	LAddress::L3Type destAddr;
	unsigned long sessionId;
	std::set<unsigned long> storedBndl;
	std::set<unsigned long> delivredToBndl;
	std::set<unsigned long> delivredToVPABndl;

public:
	LEG_NetwSession();
	LEG_NetwSession(LAddress::L3Type addr, unsigned long sessionId);
	virtual ~LEG_NetwSession();
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

#endif /* LEG_NETWSESSION_H_ */
