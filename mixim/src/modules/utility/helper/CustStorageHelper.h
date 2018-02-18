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

#ifndef CUSTSTORAGEHELPER_H_
#define CUSTSTORAGEHELPER_H_

#include "map"
#include "set"
#include "list"
#include "limits"

namespace NS_CustStorageHelper {
const double maxDbl = std::numeric_limits<double>::max();
};

class CustStorageHelper {
protected:
    /** Size of the Custody Storage structure */
    unsigned int storageSize;

    /** Fifo structure for Custody Storage*/
    std::list<unsigned long> custodySerials;

    /** Specific map with K as a Custody Serial &
  	 * V as a expire time of Custody.
  	 * */
    std::map<unsigned long,double> custodySerialsWithExpireTime;

	/** Boolean for using Custody list or not */
    bool withCustodyList;

	/** Boolean for using Custody Time To Live */
    bool withTTL;

	/** Stats related to Custody deleted due to TTL */
	int nbrDeletedCustByTTL;

	/** Stats related to Custody deleted due to StorageLimit */
	int nbrDeletedCustByFIFO;

	/** Stats related to Custody deleted due to Acl */
	int nbrDeletedCustByAck;

	/** Stats related to Custody for which we update Expire Time */
	int nbrUpdatesForCustExpireTime;

public:
	CustStorageHelper();
	CustStorageHelper(unsigned int sizeOfStorage, bool withCustodyList, bool withTTL);
	virtual ~CustStorageHelper();

	virtual void init();

	bool storeCustody(unsigned long serial, double expireTime = NS_CustStorageHelper::maxDbl);

	bool deleteCustody(unsigned long serial);
	bool deleteCustodyUponTTL(unsigned long serial);
	bool deleteCustodyUponACK(unsigned long serial);

	bool existCustody(unsigned long  serial);

	void deleteFirstCustody();

	void deleteExpiredCustodys();

	unsigned int getNbrStoredCustodys();

	bool hasExpired(double expireTime);
    int getNbrDeletedCustByAck() const
    {
        return nbrDeletedCustByAck;
    }

    int getNbrDeletedCustByFifo() const
    {
        return nbrDeletedCustByFIFO;
    }

    int getNbrDeletedCustByTtl() const
    {
        return nbrDeletedCustByTTL;
    }

    int getNbrUpdatesForCustExpireTime() const
    {
        return nbrUpdatesForCustExpireTime;
    }

    void setNbrDeletedCustByAck(int nbrDeletedCustByAck)
    {
        this->nbrDeletedCustByAck = nbrDeletedCustByAck;
    }

    void setNbrDeletedCustByFifo(int nbrDeletedCustByFifo)
    {
        nbrDeletedCustByFIFO = nbrDeletedCustByFifo;
    }

    void setNbrDeletedCustByTtl(int nbrDeletedCustByTtl)
    {
        nbrDeletedCustByTTL = nbrDeletedCustByTtl;
    }

    void setNbrUpdatesForCustExpireTime(int nbrUpdatesForCustExpireTime)
    {
        this->nbrUpdatesForCustExpireTime = nbrUpdatesForCustExpireTime;
    }

	std::map<unsigned long ,double> getCustodySerialsWithExpTime() const
	{
	    return std::map<unsigned long,double>(custodySerialsWithExpireTime);
	}

protected:
	void integrityChecker();
};

#endif /* CUSTSTORAGEHELPER_H_ */
