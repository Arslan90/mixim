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

#ifndef BUNDLEMETA_H_
#define BUNDLEMETA_H_

#include "Prophet_Enum.h"
#include "WaveShortMessage_m.h"

class BundleMeta {
private:
	int senderAddress;
	int recipientAddress;
	unsigned long serial;
	simtime_t timestamp;
	Prophet_Enum::bndlFlags bFlags;
public:
//	BundleMeta();
//	BundleMeta(BundleMeta* bndl_meta);
//	BundleMeta(BundleMeta* bndl_meta, Prophet_Enum::bndlFlags flag);
//	BundleMeta(WaveShortMessage* wsm, Prophet_Enum::bndlFlags flag);

	BundleMeta(): senderAddress(-1), recipientAddress(-1), serial(0), timestamp(0), bFlags(Prophet_Enum::Bndl_Accepted) {}
	BundleMeta(const BundleMeta &mt ): senderAddress(mt.getSenderAddress()), recipientAddress(mt.getRecipientAddress()), serial(mt.getSerial()), timestamp(mt.getTimestamp()), bFlags(mt.getFlags()) {}
	BundleMeta(const BundleMeta &mt, Prophet_Enum::bndlFlags flag): senderAddress(mt.getSenderAddress()), recipientAddress(mt.getRecipientAddress()), serial(mt.getSerial()), timestamp(mt.getTimestamp()), bFlags(flag) {}
	BundleMeta(WaveShortMessage* wsm, Prophet_Enum::bndlFlags flag): senderAddress(wsm->getSenderAddress()), recipientAddress(wsm->getRecipientAddress()), serial(wsm->getSerial()), timestamp(wsm->getTimestamp()), bFlags(flag) {};
	virtual ~BundleMeta();

	BundleMeta& operator=( const BundleMeta& other ) {
	      senderAddress = other.getSenderAddress();
	      recipientAddress = other.getRecipientAddress();
	      serial = other.getSerial();
	      timestamp = other.getTimestamp();
	      bFlags = other.getFlags();
	      return *this;
	 }

    Prophet_Enum::bndlFlags getFlags() const
    {
        return bFlags;
    }

    int getRecipientAddress() const
    {
        return recipientAddress;
    }

    int getSenderAddress() const
    {
        return senderAddress;
    }

    unsigned long getSerial() const
    {
        return serial;
    }

    simtime_t getTimestamp() const
    {
        return timestamp;
    }
};

#endif /* BUNDLEMETA_H_ */
