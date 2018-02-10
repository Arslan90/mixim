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

#ifndef ACKSTORAGEHELPER_H_
#define ACKSTORAGEHELPER_H_

#include "map"
#include "set"
#include "list"
#include "limits"

namespace NS_AckStorageHelper {
const double maxDbl = std::numeric_limits<double>::max();
};

class AckStorageHelper {
protected:
    /** Size of the WMS Storage structure */
    unsigned int storageSize;

    /** Fifo structure for WMS Storage*/
    std::list<unsigned long> ackSerials;

    /** Specific map with K as a Ack Serial &
  	 * V as a expire time of Ack.
  	 * */
    std::map<unsigned long,double> ackSerialsWithExpireTime;

	/** Boolean for using Ack or not */
    bool withAck;

	/** Boolean for using Ack Time To Live */
    bool withTTL;

	/** Stats related to Acks deleted due to TTL */
	int nbrDeletedAcksByTTL;

	/** Stats related to Acks deleted due to StorageLimit */
	int nbrDeletedAcksByFIFO;

	/** Stats related to Acks for which we update Expire Time */
	int nbrUpdatesForAckExpireTime;

public:
	AckStorageHelper();
	AckStorageHelper(unsigned int sizeOfBundleStorage, bool withAck, bool withTTL);
	virtual ~AckStorageHelper();

	bool storeAck(unsigned long serial, double expireTime = NS_AckStorageHelper::maxDbl);

	bool deleteAck(unsigned long serial);
	bool deleteAckUponTTL(unsigned long serial);

	bool existAck(unsigned long  serial);

	void deleteFirstAck();

	void deleteExpiredAcks();

	unsigned int getNbrStoredAcks();

	bool hasExpired(double expireTime);

	std::map<unsigned long,double> getAckSerialsWithExpTime(std::set <unsigned long> serials);

	std::map<unsigned long ,double> getAckSerialsWithExpTime() const
	{
	    return std::map<unsigned long,double>(ackSerialsWithExpireTime);
	}

	int getNbrDeletedAcksByFifo() const
	{
	    return nbrDeletedAcksByFIFO;
	}

	int getNbrDeletedAcksByTtl() const
	{
	    return nbrDeletedAcksByTTL;
	}

    int getNbrUpdatesForAckExpireTime() const
    {
        return nbrUpdatesForAckExpireTime;
    }

protected:
	void integrityChecker();
};

#endif /* ACKSTORAGEHELPER_H_ */
