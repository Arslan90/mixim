/*
 * ApplOppControlInfo.h
 *
 *  Created on: Dec 3, 2015
 *      Author: arslan
 */

#ifndef APPLOPPCONTROLINFO_H_
#define APPLOPPCONTROLINFO_H_

class ApplOppControlInfo : public cObject
{
  protected:
    /** @brief netw address of the previous traversed sector*/
    LAddress::L3Type oldSectorNetwAddr;

    /** @brief netw address of the actual traversed sector*/
	LAddress::L3Type newSectorNetwAddr;

  public:
    /** @brief Default constructor*/
    ApplOppControlInfo(const LAddress::L3Type& oldAddr, const LAddress::L3Type& newAddr) : oldSectorNetwAddr(oldAddr), newSectorNetwAddr(newAddr) {};
    /** @brief Destructor*/
    virtual ~ApplOppControlInfo(){};

	LAddress::L3Type getNewSectorNetwAddr() const
    {
        return newSectorNetwAddr;
    }

    LAddress::L3Type getOldSectorNetwAddr() const
    {
        return oldSectorNetwAddr;
    }

    void setNewSectorNetwAddr(LAddress::L3Type newSectorNetwAddr)
    {
        this->newSectorNetwAddr = newSectorNetwAddr;
    }

    void setOldSectorNetwAddr(LAddress::L3Type oldSectorNetwAddr)
    {
        this->oldSectorNetwAddr = oldSectorNetwAddr;
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


#endif /* APPLOPPCONTROLINFO_H_ */
