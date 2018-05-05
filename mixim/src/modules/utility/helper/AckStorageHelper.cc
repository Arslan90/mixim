//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "AckStorageHelper.h"

#include "vector"
#include "omnetpp.h"

AckStorageHelper::AckStorageHelper() {
	// TODO Auto-generated constructor stub
	init();
}

AckStorageHelper::AckStorageHelper(unsigned int sizeOfStorage, bool withAck, bool withTTL)
{
	init();
	this->storageSize = sizeOfStorage;
	this->withAck = withAck;
	this->withTTL = withTTL;
}

void AckStorageHelper::init()
{
	this->ackSerials = std::list<unsigned long>();
	this->ackSerialsWithExpireTime = std::map<unsigned long,double>();

	nbrDeletedAcksByTTL = 0;
	nbrDeletedAcksByFIFO = 0;
	nbrUpdatesForAckExpireTime = 0;
}

AckStorageHelper::~AckStorageHelper() {
	// TODO Auto-generated destructor stub
}

bool AckStorageHelper::storeAck(unsigned long  serial, double expireTime)
{
	bool stored = false;
	// step 1 : check if withAck and if ack has not expired
	if (withAck && (!hasExpired(expireTime))){
		// step 2.Alternative1 : check if the Ack is already stored and update it if so
		if (existAck(serial)){
			// step 2 : Get old expire time
			double previousExpireTime = ackSerialsWithExpireTime[serial];

			// step 3 : update ackSerialsWithExpireTime with newer expire time
			if (expireTime > previousExpireTime){
				ackSerialsWithExpireTime[serial] = expireTime;

				nbrUpdatesForAckExpireTime++;

				stored = true;
			  	integrityChecker();
			}
		// step 2.Alternative2 :if not store ack and add it to relative indexes
		}else {
			// step 2 : check if there is enough place to store ack
			deleteFirstAck();

			// step 3 : add the ack to stored ack and to all relative indexes
			ackSerials.push_back(serial);
			ackSerialsWithExpireTime.insert(std::pair<unsigned long, double>(serial,expireTime));

		  	stored = true;
		  	integrityChecker();
		}
	}

	return stored;
}

bool AckStorageHelper::deleteAck(unsigned long  serial)
{
	bool deleted = false;
	// step 1 : check if the ack is already stored
	if (withAck && existAck(serial)){
		// step 2 : delete it from ackSerials
		ackSerials.remove(serial);

		// step 3 : delete it from ackSerialsWithExpireTime
		ackSerialsWithExpireTime.erase(serial);

		deleted = true;

		integrityChecker();
	}

	return deleted;
}

bool AckStorageHelper::deleteAckUponTTL(unsigned long  serial)
{
	bool deleted = deleteAck(serial);
	if (deleted){
		nbrDeletedAcksByTTL++;
	}
	return deleted;
}

bool AckStorageHelper::existAck(unsigned long  serial)
{
	bool found = false;
	if (withAck){
		std::map<unsigned long,double>::iterator it = ackSerialsWithExpireTime.find(serial);
		if (it != ackSerialsWithExpireTime.end()){
			found = true;
		}
	}

	return found;
}

void AckStorageHelper::deleteFirstAck()
{
	if (ackSerials.size() == storageSize){
		unsigned long serialToDelete = ackSerials.front();
		if (deleteAck(serialToDelete)){
			nbrDeletedAcksByFIFO++;
		}
	}else if (ackSerials.size() > storageSize){
		opp_error("AckSerials::deleteFirstAck --- Acks storage structure exceed its maximum size");
	}
}

void AckStorageHelper::deleteExpiredAcks()
{
	if(withTTL){
		std::vector<unsigned long > oldACK;
		for (std::map<unsigned long,double>::iterator it = ackSerialsWithExpireTime.begin(); it != ackSerialsWithExpireTime.end(); it++){
			// step 1 : Check if the current ack is up to date and has not expired
			unsigned long serial = it->first;
			double expireTime = it->second;
			if (hasExpired(expireTime)){
				// step 2 : If the current ack has expired, add it to acks to delete
				oldACK.push_back(serial);
			}
		}

		// step 3 : Delete Expired Acks
		for (std::vector<unsigned long >::iterator it = oldACK.begin(); it != oldACK.end(); it++){
			deleteAckUponTTL(*it);
		}
	}
}

unsigned int AckStorageHelper::getNbrStoredAcks()
{
	integrityChecker();
	if (withAck && (!ackSerials.empty())){
		return ackSerials.size();
	}else{
		return 0;
	}
}

bool AckStorageHelper::hasExpired(double expireTime)
{
	bool hasExpired = false;
	double currentTime = simTime().dbl();

	if(withTTL && ( currentTime > expireTime)){
		hasExpired = true;
	}

	return hasExpired;
}

std::map<unsigned long ,double> AckStorageHelper::getAckSerialsWithExpTime(std::set<unsigned long > serials)
{
	std::map<unsigned long ,double> serialsWithExpTime;
	for(std::set<unsigned long>::iterator it = serials.begin(); it != serials.end(); it++){
		unsigned long serial = *it;
		if (existAck(serial) && (!hasExpired(serial))){
			double expireTime = ackSerialsWithExpireTime[serial];
			serialsWithExpTime.insert(std::pair<unsigned long ,double>(serial,expireTime));
		}
	}

	return serialsWithExpTime;
}

void AckStorageHelper::integrityChecker()
{
	// If not all Data Structures are equal in size, there is an error and we must throw an error message
	if (! ((ackSerials.size() == ackSerialsWithExpireTime.size())) ){
		opp_error("AckStorageHelper::integrityChecker --- Data Structure dedicated to storage are not equal");
	}
}

