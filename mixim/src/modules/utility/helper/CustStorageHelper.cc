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

#include "CustStorageHelper.h"

#include "vector"
#include "omnetpp.h"

CustStorageHelper::CustStorageHelper() {
	// TODO Auto-generated constructor stub
	init();
}

CustStorageHelper::CustStorageHelper(unsigned int sizeOfStorage, bool withCustodyList, bool withTTL)
{
	init();
	this->storageSize = sizeOfStorage;
	this->withCustodyList = withCustodyList;
	this->withTTL = withTTL;
}

void CustStorageHelper::init()
{
	this->custodySerials = std::list<unsigned long>();
	this->custodySerialsWithExpireTime = std::map<unsigned long,double>();

	nbrDeletedCustByTTL = 0;
	nbrDeletedCustByFIFO = 0;
	nbrDeletedCustByAck = 0;
	nbrUpdatesForCustExpireTime = 0;
}

CustStorageHelper::~CustStorageHelper() {
	// TODO Auto-generated destructor stub
}

bool CustStorageHelper::storeCustody(unsigned long  serial, double expireTime)
{
	bool stored = false;
	// step 1 : check if withCustodyList and if custody has not expired
	if (withCustodyList && (!hasExpired(expireTime))){
		// step 2.Alternative1 : check if the Custody is already stored and update it if so
		if (existCustody(serial)){
			// step 2 : Get old expire time
			double previousExpireTime = custodySerialsWithExpireTime[serial];

			// step 3 : update custodySerialsWithExpireTime with newer expire time
			if (expireTime > previousExpireTime){
				custodySerialsWithExpireTime[serial] = expireTime;

				nbrUpdatesForCustExpireTime++;

				stored = true;
			  	integrityChecker();
			}
		// step 2.Alternative2 :if not store custody and add it to relative indexes
		}else {
			// step 2 : check if there is enough place to store custody
			deleteFirstCustody();

			// step 3 : add the custody to stored custody and to all relative indexes
			custodySerials.push_back(serial);
			custodySerialsWithExpireTime.insert(std::pair<unsigned long, double>(serial,expireTime));

		  	stored = true;
		  	integrityChecker();
		}
	}

	return stored;
}



bool CustStorageHelper::deleteCustody(unsigned long  serial)
{
	bool deleted = false;
	// step 1 : check if the custody is already stored
	if (withCustodyList && existCustody(serial)){
		// step 2 : delete it from custodySerials
		custodySerials.remove(serial);

		// step 3 : delete it from custodySerialsWithExpireTime
		custodySerialsWithExpireTime.erase(serial);

		deleted = true;

		integrityChecker();
	}

	return deleted;
}



bool CustStorageHelper::deleteCustodyUponTTL(unsigned long  serial)
{
	bool deleted = deleteCustody(serial);
	if (deleted){
		nbrDeletedCustByTTL++;
	}
	return deleted;
}

bool CustStorageHelper::deleteCustodyUponACK(unsigned long  serial)
{
	bool deleted = deleteCustody(serial);
	if (deleted){
		nbrDeletedCustByAck++;
	}
	return deleted;
}



bool CustStorageHelper::existCustody(unsigned long  serial)
{
	bool found = false;
	if (withCustodyList){
		std::map<unsigned long,double>::iterator it = custodySerialsWithExpireTime.find(serial);
		if (it != custodySerialsWithExpireTime.end()){
			found = true;
		}
	}

	return found;
}



void CustStorageHelper::deleteFirstCustody()
{
	if (custodySerials.size() == storageSize){
		unsigned long serialToDelete = custodySerials.front();
		if (deleteCustody(serialToDelete)){
			nbrDeletedCustByFIFO++;
		}
	}else if (custodySerials.size() > storageSize){
		opp_error("CustStorageHelper::deleteFirstCustody --- Custody storage structure exceed its maximum size");
	}
}



void CustStorageHelper::deleteExpiredCustodys()
{
	if(withTTL){
		std::vector<unsigned long > oldCustodys;
		for (std::map<unsigned long,double>::iterator it = custodySerialsWithExpireTime.begin(); it != custodySerialsWithExpireTime.end(); it++){
			// step 1 : Check if the current custody is up to date and has not expired
			unsigned long serial = it->first;
			double expireTime = it->second;
			if (hasExpired(expireTime)){
				// step 2 : If the current custody has expired, add it to custodys to delete
				oldCustodys.push_back(serial);
			}
		}

		// step 3 : Delete Expired Custodys
		for (std::vector<unsigned long >::iterator it = oldCustodys.begin(); it != oldCustodys.end(); it++){
			deleteCustodyUponTTL(*it);
		}
	}
}



unsigned int CustStorageHelper::getNbrStoredCustodys()
{
	integrityChecker();
	if (withCustodyList && (!custodySerials.empty())){
		return custodySerials.size();
	}else{
		return 0;
	}
}



bool CustStorageHelper::hasExpired(double expireTime)
{
	bool hasExpired = false;
	double currentTime = simTime().dbl();

	if(withTTL && ( currentTime > expireTime)){
		hasExpired = true;
	}

	return hasExpired;
}

void CustStorageHelper::integrityChecker()
{
	// If not all Data Structures are equal in size, there is an error and we must throw an error message
	if (! ((custodySerials.size() == custodySerialsWithExpireTime.size())) ){
		opp_error("CustStorageHelper::integrityChecker --- Data Structure dedicated to storage are not equal");
	}
}





