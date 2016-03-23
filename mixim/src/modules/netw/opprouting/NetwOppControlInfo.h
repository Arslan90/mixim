/*
 * NetwOppControlInfo.h
 *
 *  Created on: Dec 3, 2015
 *      Author: arslan
 */

#ifndef NetwOppCONTROLINFO_H_
#define NetwOppCONTROLINFO_H_

class NetwOppControlInfo : public cObject
{
  protected:
    /** @brief netw control kind */
    int controlKind;

  public:
    /** @brief Default constructor*/
    NetwOppControlInfo(int kind) : controlKind(kind) {};
    /** @brief Destructor*/
    virtual ~NetwOppControlInfo(){};

    int getControlKind() const
    {
        return controlKind;
    }

    void setControlKind(int controlKind)
    {
        this->controlKind = controlKind;
    }

    //
    //    /*
    //	 * @brief Attaches a "control info" structure (object) to the message pMsg.
    //	 *
    //	 * This is most useful when passing packets between protocol layers
    //	 * of a protocol stack, the control info will contain the destination MAC address.
    //	 *
    //	 * The "control info" object will be deleted when the message is deleted.
    //	 * Only one "control info" structure can be attached (the second
    //	 * setL3ToL2ControlInfo() call throws an error).
    //	 *
    //	 * @param pMsg	The message where the "control info" shall be attached.
    //	 * @param pAddr	The network address of to save.
    //	 */
    //	static cObject *const setControlInfo(cMessage *const pMsg, const LAddress::L3Type & pAddr)
    //	{
    //		NetwControlInfo *const cCtrlInfo = new NetwControlInfo(pAddr);
    //		pMsg->setControlInfo(cCtrlInfo);
    //		return cCtrlInfo;
    //	}
    //
    //	/**
    //	 * @brief extracts the address from "control info".
    //	 */
    //	static const LAddress::L3Type & getAddressFromControlInfo(cObject *const pCtrlInfo)
    //	{
    //		NetwControlInfo *const cCtrlInfo = dynamic_cast<NetwControlInfo*const >(pCtrlInfo);
    //		if(cCtrlInfo)
    //			return cCtrlInfo->getNetwAddr();
    //
    //		return LAddress::L3NULL;
    //	}
};


#endif /* NetwOppCONTROLINFO_H_ */
