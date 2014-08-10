/* -*- mode:c++ -*- ********************************************************
 * file:        BurstApplLayer.h
 *
 * author:      Daniel Willkomm
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.

 ***************************************************************************
 * Ok THIS IS MI VPA APPLICATION
 *
 *
 * LAst revision: Thu Mar 15 13:21:20 CET 2012
 **************************************************************************/


#ifndef BURST_APPL_LAYER_H
#define BURST_APPL_LAYER_H

#include "MiXiMDefs.h"
#include <TestApplLayer.h>



/**
 * @brief Application layer to test lower layer implementations
 *
 * This application layer does exactly the same as the
 * TestApplLayer. The only difference is that is sends a burst of
 * broadcast messages instead of just one. The burst size is specified
 * in burstSize and can be set in omnetpp.ini
 *
 * @sa TestApplALyer
 * @ingroup applLayer
 * @author Daniel Willkomm
 **/
class MIXIM_API BurstApplLayer : public TestApplLayer
{
 public:
  /** @brief Initialize module parameters*/
  virtual void initialize(int);

 protected:
  /** @brief Handle self messages such as timer... */
  virtual void handleSelfMsg(cMessage*);

  /** @brief Handle messages from lower layer */
  virtual void handleLowerMsg(cMessage*);

  /** @brief Number of messages to send in a burst*/
  int  burstSize;
  /** @brief If true, send a unicast BROADCAST_REPLY message to each
   * received BROADCAST message. */
    bool bSendReply;

  /*
   * I Think this is only the stuff of my program..
  int messageSequence;//sequence of the VPA message number sent
  void sendVPABroadcast(int messageSequence); //own function to send message
  double CW; //Contention Window value.
  double T; //Timer to send periodic VPA WMS Broadcast. NOTE:This value must be de the same for VPA/vehicles beacons.
  */
};

#endif

