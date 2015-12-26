/*
 * ProphetSession.h
 *
 *  Created on: Dec 23, 2015
 *      Author: arslan
 */

#ifndef PROPHETSESSION_H_
#define PROPHETSESSION_H_

#include "vector"
#include "map"
#include "list"
#include "BundleMeta.h"
#include "WaveShortMessage_m.h"


class ProphetSession {

private:
	int senderAddress;
	int recipientAddress;
	simtime_t timestamp;
	unsigned long serial;
	unsigned long contactID;
	short kind;

	short totalFragment;
	short remainFragment;

	std::vector<std::map<int, double> > predsFragment;
	std::vector<std::list<BundleMeta> > bndlMetaFragment;
	std::vector<WaveShortMessage* > bundleFragment;

public:
	ProphetSession(): senderAddress(-1), recipientAddress(-1), timestamp(0), serial(0), contactID(0), kind(0), totalFragment(0), remainFragment(0){}
	ProphetSession(int sender, int recipient, simtime_t time, unsigned long serial, unsigned long contactID, short kind, short totalFragment);
	void addPredsFrag(short fragNumber, std::map<int, double> fragment);
	void addBndlMetaFrag(short fragNumber, std::list<BundleMeta> fragment);
	void addBundleFrag(short fragNumber, WaveShortMessage* fragment);
	virtual ~ProphetSession();
};

#endif /* PROPHETSESSION_H_ */
