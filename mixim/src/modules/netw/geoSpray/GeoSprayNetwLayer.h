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

#ifndef __MIXIM_GEOSPRAYNETWLAYER_H_
#define __MIXIM_GEOSPRAYNETWLAYER_H_

#include "DtnNetwLayer.h"
#include "GeoDtnNetwPkt_m.h"
#include "GeoTraCIMobility.h"

/**
 * TODO - Generated class
 */
class GeoSprayNetwLayer : public DtnNetwLayer
{
/*******************************************************************
** 							Variables section
********************************************************************/
  protected:
	GeoTraCIMobility* geoTraci;

	int nbrReplica;

//	std::map<unsigned long, int> bundlesRmgReplica;

	/*
	 * Bool withEMethod standing for erroneous method, in order to evaluate the impact of the previous error
	 */
	bool withExplicitE2EAck;

	bool withExplicitH2HAck;
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

  	virtual void sendingHelloMsg();

  	void handleHelloMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleOfferMsg(LAddress::L3Type destAddr);

  	void handleBundleOfferMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleResponseMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmResponseBndl);

  	void handleBundleResponseMsg(GeoDtnNetwPkt *netwPkt);

  	void sendingBundleMsg(LAddress::L3Type destAddr, std::vector<WaveShortMessage* >  wsmToSend);

  	void sendingBundleMsgToVPA(LAddress::L3Type vpaAddr);

  	void handleBundleMsg(GeoDtnNetwPkt *netwPkt);

	void sendingBundleE2EAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmFinalDeliverd);

	void sendingBundleH2HAckMsg(LAddress::L3Type destAddr, std::set<unsigned long> wsmDeliverd, int nbrReplica, bool custodyTransfer);

  	void handleBundleAckMsg(GeoDtnNetwPkt *netwPkt);

	////////////////////////// Others Methods //////////////////////

//  	bool erase(unsigned long serial);

  	GeoTraCIMobility* getGeoTraci();

  	double getCurrentMETD();
};

#endif
