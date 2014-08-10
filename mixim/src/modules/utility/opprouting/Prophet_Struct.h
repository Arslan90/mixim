/*
 * Prophet_Struct.h
 *
 *  Created on: Apr 22, 2014
 *      Author: arslan
 */

#ifndef PROPHET_STRUCT_H_
#define PROPHET_STRUCT_H_

#include "simtime.h"
#include "Prophet_Enum.h"
#include <iostream>
#include <bitset>
#include <list>
#include "TlvData.h"

//using namespace std;
/*
 * structure of data used to transmit data related
 * to bundle (like with Bundle Offer/Resp TLV)
 *
 * Used in phase 0 with Prophet.msg & in phase 1 with bundle Offer/Resp TLV
 */
class Prophet_Struct {
public :
	struct bndl_meta {
		int senderAddress;
		int recipientAddress;
		int serial;
		simtime_t timestamp;
		Prophet_Enum::bndlFlags bFlags;
	};

	struct prophet_header {
		unsigned char result_field;
		unsigned char codes_field;
		int transaction_ID;
	};

	struct testing {
		std::vector<bndl_meta> tests ;
	};

	struct prophet_tlv {
		unsigned char tlv_Type;
		std::bitset<8> tlv_Flags;
	//	std::list<Tlv_Data> tlv_Data;
	};
};


//// test area
//typedef std::map<LAddress::L3Type, double> preds;

#endif /* PROPHET_MSG_STRUCT_H_ */
