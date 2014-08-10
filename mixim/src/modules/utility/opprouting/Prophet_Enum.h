/*
 * Prophet_Enum.h
 *
 *  Created on: Apr 22, 2014
 *      Author: arslan
 */

#ifndef PROPHET_ENUM_H_
#define PROPHET_ENUM_H_

/*
 * Defined as described in RFC 6693,
 * the only difference is the Bundle Enum that is not defined in RFC
 * it value is taken from Experimental values (bounded between 0xD0 & 0xFF)
 */

class Prophet_Enum{
public:
	//	namespace tlvType {
	//		typedef enum {
	//			HELLO = 0x00,
	//			ERROR = 0x01,
	//			RIBD  = 0x02,
	//			RIB   = 0x03,
	//			Bundle_Offer = 0xA4,
	//			Bundle_Response = 0xA5,
	//			Bundle = 0xFF,
	//		}t_tlv_type;
	//	};


	enum t_flags_hello {
		SYN = 0b000,
		SYNACK = 0b001,
		ACK    = 0b010,
		RSTACK = 0b100,
	};

	enum t_flags_error {
		Dictionary_Conflict = 0x00,
		Bad_String_ID 		= 0x01,
	};

	enum t_flags_RIBD {
		Sent_by_Initiator = 00,
		Sent_by_Listener = 01,
	};

	//enum t_flags_RIB {
	//	More_RIB_TLV = 0,
	//};
	//
	//enum t_flags_Bundle_OfferResp {
	//	More_OfferResp_TLV = 0,
	//};
	/*
	 * Replacement for RIB & Bundle Offer/Responses Flag, for more detail Consult : https://www.dropbox.com/s/yy4j9zf8o7v9rg4/D%C3%A9tails%20sur%20l%27impl%C3%A9mentation%20des%20TLV%20Type%20et%20FLags.pdf
	 */
	enum t_flags_More_TLV {
		No_More_TLV = 00,
		At_Least_One_More_TLV = 01,
	};

//	namespace bndlFlags {
//		typedef enum {
//			Bndl_Accepted = 0,
//			Bndl_is_a_Fragment = 1,
//			Bndl_Payload_Length_Included = 2,
//			PRoPHET_ACK = 7,
//		}t_b_flags;
//	};

	enum bndlFlags {
			Bndl_Accepted = 0,
			Bndl_is_a_Fragment = 1,
			Bndl_Payload_Length_Included = 2,
			PRoPHET_ACK = 7,
	};


	enum t_result_field {
		NoSuccessACK = 0x01,
		ACKAll,
		Success,
		Failure,
		ReturnReceipt,
	};

	enum t_SuccessCodes_field {
		Generic_Success = 0x01,
		Submessage_Received,
	};

	enum t_FailureCodes_field {
		Unspecified_Failure = 0x02,
		Error_TLV_in_message = 0xFF,
	};
};

#endif /* PROPHET_ENUM_H_ */
