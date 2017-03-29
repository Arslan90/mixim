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

#ifndef __MIXIM_DDELIVERYNETWLAYER_H_
#define __MIXIM_DDELIVERYNETWLAYER_H_

#include <omnetpp.h>
#include "DtnNetwLayer.h"
#include "GeoDtnNetwPkt_m.h"
#include "set"

/**
 * TODO - Generated class
 */
class DDeliveryNetwLayer : public DtnNetwLayer
{
/*******************************************************************
** 							Variables section
********************************************************************/
  protected:
	int NBHAddressNbrInsert;
	int NBHAddressNbrDelete;

	int totalBundlesReceived;

	bool meetVPA;

	// E2E Acks serial
	std::set<unsigned long > ackSerial;

	int bndlSentToVPA;

	bool firstSentToVPA;

	int totalBndlSentToVPA;

	/**
	 * Comparator used to sort Bundles to sent when using RC Asc strategy
	 */
	struct comparatorRCAsc {
		bool operator() (std::pair<WaveShortMessage*, int> i,std::pair<WaveShortMessage*, int> j)
		{
		  if (i.second != j.second){
			  return (i.second<j.second);
		  }else {
			  return (i.first->getTimestamp()>j.first->getTimestamp());
		  }

		}
	} comparatorRCAscObject;

	// comparison, not case sensitive.
	struct comparatorRLAsc {
		bool operator() (WaveShortMessage* first_TTL, WaveShortMessage* second_TTL)
		{
			simtime_t i = first_TTL->getTimestamp();
			simtime_t j = second_TTL->getTimestamp();
			return ( i <= j );
		}
	} comparatorRLAscObject;
/*******************************************************************
** 							Methods section
********************************************************************/
  public:
 	virtual int numInitStages() const {
    	return 3;
    }
    virtual void initialize(int stage);
    virtual void finish();

  protected:
  	/** @brief Handle messages from upper layer */
  	virtual void handleUpperMsg(cMessage* msg);

  	/** @brief Handle messages from lower layer */
  	virtual void handleLowerMsg(cMessage* msg);

  	/** @brief Handle self messages */
	virtual void handleSelfMsg(cMessage* msg);

  	/** @brief Handle control messages from lower layer */
  	virtual void handleLowerControl(cMessage* msg);
//
//  	/** @brief Handle control messages from lower layer */
//  	virtual void handleUpperControl(cMessage* msg);

  	void sendingHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleMsg(LAddress::L3Type destAddr);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

	void sendingBundleAckMsg(LAddress::L3Type destAddr, std::list<unsigned long > wsmFinalDeliverd);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

	////////////////////////// Others Methods //////////////////////

  	/** Function for preparing GeoDtnNetwPkt */
  	GeoDtnNetwPkt* prepareNetwPkt(short kind, LAddress::L3Type srcAddr, int srcType, LAddress::L3Type destAddr, int vpaSectorId, LAddress::L3Type vpaAddr);

  	void storeAckSerial(unsigned long serial);

  	void storeAckSerial(std::set<unsigned long> setOfSerials);

  	bool erase(unsigned long serial);

  	bool exist(unsigned long serial);
};

#endif