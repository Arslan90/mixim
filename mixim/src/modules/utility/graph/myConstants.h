/*
 * myConstants.h
 *
 *  Created on= May 29; 2016
 *      Author= arslan
 */

//ifndef MYCONSTANTS_H_
//define MYCONSTANTS_H_
#include "sstream"
#include "list"
#include "simutil.h"
#include "vector"


namespace MY_CONST {

// ****************************************
// VERSION
// ****************************************
const int VERSION = 1;

// ****************************************
// COMMANDS
// ****************************************
// Basics= 0x0X
const int NBR_ARGUMENTS = 0x01;
// command; query; response
const int QUERY = 0x02;
const int RESPONSE = 0x03;

// ****************************************
// COMMANDS VALUES
// ****************************************
// Commands values= 0x1X
const int CMD_NP_ALL = 0x11;
const int RESPONSE_NP_ALL = 0x12;

const int CMD_EDGE_BEST_TRAVEL_TIME = 0x13;
const int RESPONSE_EDGE_BEST_TRAVEL_TIME = 0x14;

// ****************************************
// NP RELATED
// ****************************************
// DATA Related to Nearest point node (NP)= 0x2X
const int EDGE_TO_NP = 0x26;
const int NODE_NP = 0x27;
const int EDGE_FROM_NP = 0x28;

// ****************************************
// VPA RELATED
// ****************************************
// DATA Related to VPA= 0x3X
const int VPA_SECTOR_ID = 0x31;
const int VPA_MAPPING_TYPE = 0x32;
const int VPA_MAPPING_NBR = 0x32;

const int EDGE_TO_VPA_MAPPING = 0x36  ;
const int NODE_VPA_MAPPING = 0x37     ;
const int EDGE_FROM_VPA_MAPPING = 0x38;

// ****************************************
// ROUTE RELATED
// ****************************************
// ROUTE RELATED = 0x5X
const int ROUTE_CURRENT = 0x51; // as a list of edges; edged route
const int ROUTE_LENGTH_CURRENT = 0x52;
const int ROUTE_NP_VPA = 0x53;
const int ROUTE_LENGTH_NP_VPA = 0x54;

// ****************************************
// EDGE RELATED
// ****************************************
// EDGE RELATED = 0x6X
const int EDGE_ID = 0x61;
const int EDGE_BEST_TRAVEL_TIME = 0x62;

// ****************************************
// SPECIAL COMMAND RELATED
// ****************************************
// SPECIAL COMMAND = 0x7X
const int HOTSPOT_MODE = 0x71;

// ****************************************
// COMMAND Arguments type
// ****************************************
// COMMAND Arguments type= 0x8x
const int BOOL = 0x81       ;
const int INT = 0x82        ;
const int FLOAT = 0x83      ;
const int STRING = 0x84     ;
const int LIST_STRING = 0x85;
const int MAPPING_VPA = 0x86;
const char* T_NODE = "NODE";
const char* T_EDGE = "EDGE";
const char* T_NONE = "NONE";

// ****************************************
// DICO FOR Arguments type
// ****************************************
const int   T_NBR_ARGUMENTS = INT;
const int   T_EDGE_TO_NP = STRING;
const int   T_NODE_NP = STRING;
const int   T_EDGE_FROM_NP  = STRING;
const int   T_VPA_SECTOR_ID  = INT;
const int   T_VPA_MAPPING_TYPE  = MAPPING_VPA;
const int   T_VPA_MAPPING_NBR  = INT;
const int   T_EDGE_TO_VPA_MAPPING  = STRING;
const int   T_NODE_VPA_MAPPING  = STRING;
const int   T_EDGE_FROM_VPA_MAPPING  = STRING;
const int   T_ROUTE_CURRENT  = LIST_STRING;
const int   T_ROUTE_LENGTH_CURRENT  = FLOAT;
const int   T_ROUTE_NP_VPA  = LIST_STRING;
const int   T_ROUTE_LENGTH_NP_VPA  = FLOAT;
const int   T_EDGE_ID  = STRING;
const int   T_EDGE_BEST_TRAVEL_TIME  = FLOAT;
const int   T_HOTSPOT_MODE  = STRING;

std::string convertToStr(int intToConvert){
	std::stringstream ss;
	ss << intToConvert;
	std::string str = std::string(ss.str());
	return str;
}

double convertToDbl(std::string strToConvert){
	double convertedDbl = 0;
	if (strToConvert != ""){
		std::istringstream iss(strToConvert);
		iss >> convertedDbl;
	}else {
		opp_error("Unable to convert str to double");
	}
	return convertedDbl;
}

std::vector<std::string> tokenizeMSG(char* msg_data){
	std::vector<std::string> tokens, one_entryTokens, one_tupleTokens;
	char* copyOfMsgData = strdup((const char*) msg_data);
	char* one_entry= strtok (copyOfMsgData,"?");
	while (one_entry != NULL)
	{
		one_entryTokens.push_back(std::string(one_entry));
		one_entry = strtok (NULL, "?");
	}

	for (std::vector<std::string>::iterator it = one_entryTokens.begin(); it != one_entryTokens.end(); it++){
		char* one_tuple= strtok (strdup((*it).c_str()),";");
		while (one_tuple != NULL)
		{
			one_tupleTokens.push_back(std::string(one_tuple));
			one_tuple = strtok (NULL, ";");
		}
		free (one_tuple);
	}

	for (std::vector<std::string>::iterator it = one_tupleTokens.begin(); it != one_tupleTokens.end(); it++){
		char* one_arg= strtok (strdup((*it).c_str()),":");
		while (one_arg != NULL)
		{
			tokens.push_back(std::string(one_arg));
			one_arg = strtok (NULL, ":");
		}
		free (one_arg);
	}

	free(one_entry);

	return tokens;
}

std::vector<std::string> tokenizeMSG(std::string msg_data){
	std::vector<std::string> tokens, one_entryTokens, one_tupleTokens;
	char* copyOfMsgData = strdup(msg_data.c_str());
	char* one_entry= strtok (copyOfMsgData,"?");
	while (one_entry != NULL)
	{
		one_entryTokens.push_back(std::string(one_entry));
		one_entry = strtok (NULL, "?");
	}

	for (std::vector<std::string>::iterator it = one_entryTokens.begin(); it != one_entryTokens.end(); it++){
		char* one_tuple= strtok (strdup((*it).c_str()),";");
		while (one_tuple != NULL)
		{
			one_tupleTokens.push_back(std::string(one_tuple));
			one_tuple = strtok (NULL, ";");
		}
		free (one_tuple);
	}

	for (std::vector<std::string>::iterator it = one_tupleTokens.begin(); it != one_tupleTokens.end(); it++){
		char* one_arg= strtok (strdup((*it).c_str()),":");
		while (one_arg != NULL)
		{
			tokens.push_back(std::string(one_arg));
			one_arg = strtok (NULL, ":");
		}
		free (one_arg);
	}

	free(one_entry);

	return tokens;
}

/* Structure describing return data for CMD_NP_ALL.  */
//class NP_ALL
//{
//public:
//    std::string NP_edgeTo;
//    std::string NP_node;
//    std::string NP_edgefrom;
//    std::string VPA_node;
//    std::list<std::string> Route_NP_VPA;
//    float Distance_NP_VPA;
//    NP_ALL(){
//    	NP_edgeTo = "";
//    	NP_node = "";
//    	NP_edgefrom = "";
//    	VPA_node = "";
//    	Route_NP_VPA = std::list<std::string>();
//    	Distance_NP_VPA = 0.0;
//    };
//    virtual ~NP_ALL();
//
//};

}  // namespace MY_CONST






//endif /* MYCONSTANTS_H_ */
