///*
// * Prophetv2Kinds.h
// *
// *  Created on: Apr 5, 2015
// *      Author: arslan
// */
//
//#ifndef PROPHETV2KINDS_H_
//#define PROPHETV2KINDS_H_
//
//#include "BaseNetwLayer.h"
//	/**
//	 * @brief The message kinds used by PRoPHETv2 as implemented.
//	 * Defined as described in RFC 6693,
//	 * the only difference is the Bundle Enum that is not defined in RFC
//	 * it value is taken from Experimental values (bounded between 0xD0 & 0xFF)
//	 */
//	enum TestProphetv2MessageKinds {
//		HELLO = 0x00,
//		ERROR = 0x01,
//		RIBD  = 0xA0,
//		RIB   = 0xA1,
//		Bundle_Offer = 0xA4,
//		Bundle_Response = 0xA5,
//		Bundle_Ack = 0xD0,
//		Bundle = 0xFF,
//	};
//	/**
//	 * @brief Prophet Control Kinds used when notified by the lower layer (i.e Mac1609_4_Opp & NicEntryDebug)
//	 */
//	enum TestprophetNetwControlKinds {
//		NEWLY_CONNECTED = BaseNetwLayer::LAST_BASE_NETW_CONTROL_KIND,
//		NEW_NEIGHBOR = NEWLY_CONNECTED + 10,
//		NO_NEIGHBOR_AND_DISCONNECTED = NEWLY_CONNECTED + 20,
//		NEW_NEIGHBOR_GONE = NEWLY_CONNECTED + 30,
//	};
//
//
//
//#endif /* PROPHETV2KINDS_H_ */
