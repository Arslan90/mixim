/* -*- mode:c++ -*- ********************************************************
 * Myfile:        TestingBurstApplLayer.h
 * Ok this modify file  is to review the broadcast problem.
 *
 ***************************************************************************/


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
class MIXIM_API TestingBurstApplLayer : public TestApplLayer
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
  //Arturo My functions and Variables.
  void sendVPABroadcast(int messageSequence); //own function to send message
  int messageSequence;//sequence of the VPA message number sent
  double CW; //Contention Window value.
  double T; //Timer to send periodic VPA WMS Broadcast. NOTE:This value must be de the same for VPA/vehicles beacons.
  bool appCW;//Using or not appCW
  double appMinCW; // what value
};

#endif

