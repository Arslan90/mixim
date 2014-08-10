/*
 * TlvData.h
 *
 *  Created on: Apr 23, 2014
 *      Author: arslan
 */

#ifndef TLVDATA_H_
#define TLVDATA_H_

#include <bitset>
#include "simtime.h"
#include <omnetpp.h>
#include <map>
#include "SimpleAddress.h"
#include "Prophet_Enum.h"

using namespace std;

//class Tlv_Data {
//public :
//	virtual string tlv_data_type ()=0;
//
//	virtual ~Tlv_Data();
//};
//class Tlv_Hello : public Tlv_Data {
//
//public :
//	Tlv_Hello();
//	virtual ~Tlv_Hello();
//    unsigned int getTimer() const
//    {
//        return timer;
//    }
//
//    void setTimer(unsigned int timer)
//    {
//        this->timer = timer;
//    }
//	string tlv_data_type(){
//		return t_tlv_type::HELLO;
//	}
//
//private :
//	unsigned int timer;
//};
//
//class Tlv_Error : public Tlv_Data {
//public :
//	Tlv_Error();
//	virtual ~Tlv_Error();
//	string Tlv_Data::tlv_data_type(){
//		return t_tlv_type::ERROR;
//	}
//};
//
//class Tlv_RIBD : public Tlv_Data {
//public :
//	Tlv_RIBD();
//	virtual ~Tlv_RIBD();
//	string Tlv_Data::tlv_data_type(){
//		return t_tlv_type::RIBD;
//	}
//};
//
//class Tlv_RIB : public Tlv_Data {
//public :
//	Tlv_RIB();
//	virtual ~Tlv_RIB();
//	string Tlv_Data::tlv_data_type(){
//		return t_tlv_type::RIB;
//	}
//
//private :
//	std::map<LAddress::L3Type, double > transmited_Preds;
//};
//
//class Tlv_Bndl_OfferResp : public Tlv_Data {
//public :
//	Tlv_Bndl_OfferResp();
//	virtual ~Tlv_Bndl_OfferResp();
//
//protected :
//	std::bitset<8> bndl_Flags;
//	int senderAddress_var;
//    int recipientAddress_var;
//	int serial_var;
//	simtime_t timestamp_var;
//};
//
//class Tlv_Bndl_Offer : public Tlv_Bndl_OfferResp {
//public :
//	string Tlv_Data::tlv_data_type(){
//		return t_tlv_type::Bundle_Offer;
//	}
//};
//
//class Tlv_Bndl_Resp : public Tlv_Bndl_OfferResp {
//public :
//	string Tlv_Data::tlv_data_type(){
//		return t_tlv_type::Bundle_Response;
//	}
//};
///*
// * Initially build to define Bundle that are a WSMessage + TTL,
// * but i preferred to extend WSMessage with TTL variable
// */
//
//class Tlv_Bundle : public Tlv_Data {
//public :
//	Tlv_Bundle();
//	virtual ~Tlv_Bundle();
//};


#endif /* TLVDATA_H_ */
