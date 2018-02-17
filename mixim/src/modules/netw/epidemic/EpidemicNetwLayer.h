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

#ifndef __MIXIM_EPIDEMICNETWLAYER_H_
#define __MIXIM_EPIDEMICNETWLAYER_H_

#include "DtnNetwLayer.h"
#include "GeoDtnNetwPkt_m.h"

/**
 * TODO - Generated class
 */
class EpidemicNetwLayer : public DtnNetwLayer
{
/*******************************************************************
** 							Variables section
********************************************************************/
  protected:

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

  	/** @brief Handle messages from lower layer */
  	virtual void handleLowerMsg(cMessage* msg);

  	virtual void sendingHelloMsg();

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBndlResponseMsg( LAddress::L3Type nodeAddr, std::set<unsigned long> wsmResponseBndl);

  	void handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleMsg(LAddress::L3Type destAddr, int destType, std::vector<WaveShortMessage* > wsmToSend);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

	void sendingBundleAckMsg(LAddress::L3Type destAddr, std::set<unsigned long > wsmFinalDeliverd);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

	////////////////////////// Others Methods //////////////////////
};

#endif
