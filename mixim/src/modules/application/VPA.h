/* -*- mode:c++ -*- ********************************************************
 * Myfile:        VAP.h based on the file: BurstApplLayer.h
 *
 * Ok THIS IS MI VPA APPLICATION
 *
 *
 * LAst revision: Thu Mar 15 13:21:20 CET 2012
 **************************************************************************/


#ifndef BURST_APPL_LAYER_H
#define BURST_APPL_LAYER_H

#include "MiXiMDefs.h"
#include <TestApplLayer.h>


class MIXIM_API VPA : public TestApplLayer
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
  int messageSequence;//sequence of the VPA message number sent
  double CW; //Contention Window value.
  double T; //Timer to send periodic VPA WMS Broadcast. NOTE:This value must be de the same for VPA/vehicles beacons.
  void sendVPABroadcast(int messageSequence); //own function to send message
};

#endif

