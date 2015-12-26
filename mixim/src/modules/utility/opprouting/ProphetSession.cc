/*
 * ProphetSession.cc
 *
 *  Created on: Dec 23, 2015
 *      Author: arslan
 */

#include "ProphetSession.h"


ProphetSession::ProphetSession(int sender, int recipient, simtime_t time, unsigned long  serial, unsigned long  contactID, short  kind, short  totalFragment)
{
	ProphetSession();
	this->senderAddress = sender;
	this->recipientAddress = recipient;
	this->timestamp = time;
	this->serial = serial;
	this->contactID = contactID;
	this->kind = kind;
	this->totalFragment = totalFragment;
	this->remainFragment+= totalFragment;
}



void ProphetSession::addPredsFrag(short  fragNumber, std::map<int,double> fragment)
{
	this->remainFragment--;
	this->predsFragment[fragNumber] = fragment;
}



void ProphetSession::addBndlMetaFrag(short  fragNumber, std::list<BundleMeta> fragment)
{
	this->remainFragment--;
	this->bndlMetaFragment[fragNumber] = fragment;
}



void ProphetSession::addBundleFrag(short  fragNumber, WaveShortMessage *fragment)
{
	this->remainFragment--;
	this->bundleFragment[fragNumber] = fragment;
}



ProphetSession::~ProphetSession()
{
}

