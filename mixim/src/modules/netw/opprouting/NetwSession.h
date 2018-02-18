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
	std::set<unsigned long> storedBundles;
	std::map<unsigned long, double > storedAcks;
	std::map<unsigned long, double > storedCustody;

public:
	NetwSession();
	NetwSession(LAddress::L3Type addr, unsigned long sessionId);
	virtual ~NetwSession();

	virtual void init();

//    void insertInDelivredToBndl(unsigned long delivredSerial);
//    void insertInDelivredToVpaBndl(unsigned long delivredToVpaSerial);
    void updateStoredBundle(std::set<unsigned long> bundleToStore);
    void updateStoredAck(std::map<unsigned long, double > ackToStore);
    void updateStoredCustody(std::map<unsigned long, double > custodyToStore);

    bool existInStoredBundle(unsigned long serial);
    bool existInStoredAck(unsigned long serial);
    bool existInStoredCustody(unsigned long serial);
    bool existInStoredBundleOrAck(unsigned long serial);

};

#endif /* NETWSESSION_H_ */
