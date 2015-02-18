//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "ProphetV2.h"



Define_Module(ProphetV2);

void ProphetV2::initialize(int stage)
{
	BaseNetwLayer::initialize(stage);
	if (stage==0){
		/*
		 * L3Address will be initialized by BaseNetwLayer::initialize(1);
		 */
//		myNetwAddr = LAddress::L3Type( getId() );

		/*
		 * Initialization of ProphetV2 parameters
		 */
		PEncMax = par("PEncMax").doubleValue();
		PFirstContact = par("PFirstContact").doubleValue();
		PMinThreshold = par("PMinThreshold").doubleValue();
		I_TYP = par("I_TYP").doubleValue();
		Beta = par("Beta").doubleValue();
		GAMMA = par("GAMMA").doubleValue();
		secondsInTimeUnit = par("secondsInTimeUnit");
		/*
		 * Initialization of the forward strategy &
		 * the queuing strategy
		 */
		int fwd = par("fwdStrategy");
		fwdStrategy = static_cast<t_prophet_forward>(fwd);
		int q = par("qStrategy");
		qStrategy = static_cast<t_prophet_queuing>(q);
		/*
		 * Initialization of the used structures
		 */
		lastAgeUpdate = 0;

//		preds = std::map<LAddress::L3Type,double>();



//		lastEncouterTime = std::map<LAddress::L3Type,double>();

		bundlesStructureSize = par("storageSize");
		if (bundlesStructureSize<0){
			opp_error("Size of the structure that store bundles can not be negative");
		}
		bundles = std::list<WaveShortMessage*>();


		withAck = par("withAck");
		ackStructureSize = par("ackSize");
		if (ackStructureSize<=0){
			opp_error("Size of the structure that store acks can not be negative");
		}
		acks = std::list<BundleMeta>();
		acksIndex = std::map<int,BundleMeta>();

		/*
		 * Collecting data & metrics
		 */
	    nbrL3Sent = 0;
	    nbrL3Received = 0;

	    nbrPredsVector.setName("Number of predictions");
	    predsMean.setName("Mean of predictions");
	    predsMax.setName("Maximum of predictions");
	    predsMin.setName("Minimum of predictions");
	    predsVariance.setName("Variance of predictions");

	    nbrContacts =0;
	    sumOfContactDur = 0.0;
	    contacts = std::map<LAddress::L3Type,double>();
	    contactDurVector.setName("Evolution of contact duration mean");

	    nbrRecontacts = 0;
	    sumOfInterContactDur = 0.0;
	    intercontactDurVector.setName("Evolution of intercontact duration mean");

	    nbrSuccessfulContact = 0;
        nbrFailedContactBeforeRIB = 0;
        nbrFailedContactAtRIB= 0;
        nbrFailedContactAtBundle_Offer = 0;
        nbrFailedContactAtBundle_Response = 0;
        contactState = std::map<LAddress::L3Type, Prophetv2MessageKinds>();


	}
	else if (stage==1){
		preds.insert(std::pair<LAddress::L3Type,double>(myNetwAddr,1));
	}
}

/*******************************************************************
**
** 							Methods related to predictions
**
********************************************************************/

void ProphetV2::updateDeliveryPredsFor(const LAddress::L3Type BAdress)
{
	double PEnc,lastEncTime, predsForB;
	/*
	 * before calculating predsForB, we must age preds.
	 */
	ageDeliveryPreds();
	double encTime = simTime().dbl();
	predsIterator it= preds.find(BAdress);
	predsIterator it2= lastEncouterTime.find(BAdress);
	if (it==preds.end()){

		/*
		 * if iterator is equal map.end(), it means that there is no entry for BAdress in preds
		 * so lastEncTime equal 0
		 */
		predsForB = PFirstContact;
		lastEncTime = 0;

	}else {

		/*
		 * if iterator is not equal map.end(), it means that there is an entry for BAdress in preds
		 */
		predsForB = it->second;
		if (it2==lastEncouterTime.end()){
			lastEncTime = 0;
		}else {
			lastEncTime = it2->second;
		}

		if (simTime().dbl()-lastEncTime<I_TYP){
			/*
			 * if the node has been encountered recently then don't use PEncMax
			 */
			PEnc = PEncMax * (( simTime().dbl()-lastEncTime)/I_TYP);
		}else {
			/*
			 * if the node has been encountered long ago then use PEncMax
			 */
			PEnc = PEncMax;
		}
		predsForB = predsForB + (1-predsForB)*PEnc;
	}
	preds[BAdress] = predsForB;
	lastEncouterTime[BAdress] = encTime;


//	preds.insert(std::pair<LAddress::L3Type,double>(BAdress,predsForB));
//	lastEncouterTime.insert(std::pair<LAddress::L3Type,double>(BAdress,encTime));

//	std::pair<std::map<LAddress::L3Type,double>::iterator, bool> result;
//	result = preds.insert(std::pair<LAddress::L3Type,double>(BAdress,predsForB));
//	if (result.second == false){
//		noInsert++;
//		WATCH(noInsert);
//		EV << "Impossible to insert (K,V) : " << "(" << BAdress << "," << predsForB << ")" << endl;
//		EV << "Actual pair (K,V) : " << "(" << result.first->first << "," << result.first->second << ")" << endl;
//	}else {
//		EV << "Inserted pair (K,V) : " << "(" << result.first->first << "," << result.first->second << ")" << endl;
//	}
//	std::pair<std::map<LAddress::L3Type,double>::iterator, bool> result2;
//	result2 = lastEncouterTime.insert(std::pair<LAddress::L3Type,double>(BAdress,encTime));
//	if (result2.second == false){
//		noInsert++;
//		WATCH(noInsert);
//		EV << "Impossible to insert (K,V) : " << "(" << BAdress << "," << encTime << ")" << endl;
//		EV << "Actual pair (K,V) : " << "(" << result2.first->first << "," << result2.first->second << ")" << endl;
//	}else {
//		EV << "Inserted pair (K,V) : " << "(" << result2.first->first << "," << result2.first->second << ")" << endl;
//	}
}


void ProphetV2::updateTransitivePreds(const LAddress::L3Type BAdress, std::map<LAddress::L3Type,double> Bpreds)
{

	double predsForB,predsForC,BpredsForC;
	LAddress::L3Type CAdress;
	predsIterator tmp_it;
	/*
	 * before calculating transitive predictions, we must age preds.
	 */
	ageDeliveryPreds();
	for (predsIterator it=Bpreds.begin(); it!=Bpreds.end();it++){

		if (it->first==myNetwAddr)
			continue;

		CAdress = it->first;
		tmp_it= preds.find(CAdress);
		if (tmp_it == preds.end()){
			predsForC = 0;
		} else {
			predsForC=tmp_it->second;
		}

		tmp_it= Bpreds.find(CAdress);
		BpredsForC=tmp_it->second;

		tmp_it= preds.find(BAdress);
		predsForB=tmp_it->second;

		if (predsForC<(predsForB*BpredsForC*Beta)){
			predsForC = predsForB*BpredsForC*Beta;
		}

		preds[CAdress] = predsForC;
//		preds.insert(std::pair<LAddress::L3Type,double>(CAdress,predsForC));
	}
}

void ProphetV2::ageDeliveryPreds()
{
	predsIterator it2;
	double time = simTime().dbl();
//	double timeDiff = (time-lastAgeUpdate)/secondsInTimeUnit;
	int  timeDiff = int (time-lastAgeUpdate)/secondsInTimeUnit;
	if (timeDiff==0){
		return;
	}else {
		double mult = std::pow(GAMMA, timeDiff);
		for (predsIterator it=preds.begin();it!=preds.end();it++){
			it->second = it->second * mult;


			if (it->second < PMinThreshold){
				it2 = it;
				it++;
				preds.erase(it2);
			}
		}
		lastAgeUpdate = simTime().dbl();
	}
}

void ProphetV2::update(Prophet *prophetPkt)
{
	updateDeliveryPredsFor(prophetPkt->getSrcAddr());
	updateTransitivePreds(prophetPkt->getSrcAddr(),prophetPkt->getPreds());
	recordPredsStats();
}

/*******************************************************************
**
** 							Core methods
**
********************************************************************/

NetwPkt* ProphetV2::encapsMsg(cPacket *appPkt)
{
	LAddress::L2Type macAddr;
	LAddress::L3Type netwAddr;

	coreEV <<"in encaps...\n";

	NetwPkt *pkt = new NetwPkt(appPkt->getName(), appPkt->getKind());
	pkt->setBitLength(headerLength);

	cObject* cInfo = appPkt->removeControlInfo();

	if(cInfo == NULL){
	EV << "warning: Application layer did not specifiy a destination L3 address\n"
	   << "\tusing broadcast address instead\n";
		netwAddr = LAddress::L3BROADCAST;
	} else {
	coreEV <<"CInfo removed, netw addr="<< NetwControlInfo::getAddressFromControlInfo( cInfo ) << std::endl;
		netwAddr = NetwControlInfo::getAddressFromControlInfo( cInfo );
	delete cInfo;
	}

	pkt->setSrcAddr(myNetwAddr);
	pkt->setDestAddr(netwAddr);
	coreEV << " netw "<< myNetwAddr << " sending packet" <<std::endl;
	if(LAddress::isL3Broadcast( netwAddr )) {
		coreEV << "sendDown: nHop=L3BROADCAST -> message has to be broadcasted"
		   << " -> set destMac=L2BROADCAST\n";
		macAddr = LAddress::L2BROADCAST;
	}
	else{
		coreEV <<"sendDown: get the MAC address\n";
		macAddr = arp->getMacAddr(netwAddr);
	}

//	setDownControlInfo(pkt, macAddr);

	//encapsulate the application packet
	pkt->encapsulate(appPkt);
	coreEV <<" pkt encapsulated\n";
	return pkt;
}

void ProphetV2::handleLowerMsg(cMessage* msg)
{

    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;

    Prophet *prophetPkt = check_and_cast<Prophet *>(m->decapsulate());
    delete(m);

    int hopcount;

    if ((prophetPkt->getDestAddr()==LAddress::L3BROADCAST)||(prophetPkt->getDestAddr()==myNetwAddr)){

		switch (prophetPkt->getKind()) {
			case HELLO:
				break;
			case ERROR:
				break;
			case RIBD:
				break;
			case RIB:
				{
					// collecting data
					updatingL3Received();

					// first step : updating preds
					executeListenerRole(RIB,prophetPkt);
					// second step : starting of Bundle_Offer phase
					executeListenerRole(Bundle_Offer,prophetPkt);
				}
				break;
			case Bundle_Offer:
				{
					// collecting data
					updatingL3Received();

					executeInitiatorRole(Bundle_Offer,prophetPkt);
					executeInitiatorRole(Bundle_Response,prophetPkt);
				}
				break;
			case Bundle_Response:
				{
					// collecting data
					updatingL3Received();

					executeListenerRole(Bundle_Response,prophetPkt);
					executeListenerRole(Bundle,prophetPkt);
				}
				break;
			case Bundle_Ack:
				{
					// collecting data
					updatingL3Received();

					executeListenerRole(Bundle_Ack,prophetPkt);
				}
				break;
			case Bundle:
				{
					// collecting data
					updatingL3Received();

					executeInitiatorRole(Bundle,prophetPkt);
					if (withAck){
						executeInitiatorRole(Bundle_Ack,prophetPkt);
					}
				}
				break;
			default:
				opp_error("Unknown Prophetv2MessageKinds when calling HandleLowerMsg()");
				break;
		}
    }
}

void ProphetV2::handleUpperMsg(cMessage* msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
	storeBundle(upper_msg);
//	if (canITransmit){
//		sendDown(encapsMsg(upper_msg->dup()));
//	}
}

void ProphetV2::handleLowerControl(cMessage* msg)
{
	switch (msg->getKind()) {
		case NEWLY_CONNECTED:
			canITransmit = true;
			break;
		case NEW_NEIGHBOR:
			{
				if (canITransmit){
					double time = simTime().dbl();

					LAddress::L3Type addr = getAddressFromName(msg->getName());

					/** Repeated contacts stats			*/
					recordRecontactStats(addr,time);

					/** Contacts duration stats				*/
					recordBeginContactStats(addr,time);

					/** Starting IEP Phase					*/
					executeInitiatorRole(RIB,NULL,addr);
				}
			}
			break;
		case NO_NEIGHBOR_AND_DISCONNECTED:
			canITransmit = false;
			break;
		case NEW_NEIGHBOR_GONE:
			{
				double time = simTime().dbl();

				LAddress::L3Type addr = getAddressFromName(msg->getName());

				/** Contacts duration stats				 */
				recordEndContactStats(addr,time);
			}
			break;
	}
}

void ProphetV2::storeBundle(WaveShortMessage *msg)
{
		// step 1 : check if the bundle is already stored
	if (!exist(msg)){

		// step 2 : add the bundle to stored bundles
		switch (qStrategy) {
			case QUEUING_FIFO:
			{
				if (bundles.size()==bundlesStructureSize){
					bundles.pop_front();
				}else if (bundles.size()>bundlesStructureSize){
					opp_error("bundles storage structure exceed its maximum size");
				}
			}
				break;
			case QUEUING_MOFO:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_MOPR:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_LinearMOPR:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_SHLI:
				opp_error("QUEUING Strategy not supported yet");
				break;
			case QUEUING_LEPR:
				opp_error("QUEUING Strategy not supported yet");
				break;
		}
		bundles.push_back(msg);

		// step 3 : adding this bundle to index
		if (bundlesIndex.count(msg->getRecipientAddress())==0){
			// no entry for msg->getRecipientAddress())
			std::map <int, WaveShortMessage*> inner_map;
			inner_map.insert(std::pair<int,WaveShortMessage*>(msg->getSerial(),msg));
			bundlesIndex.insert(std::pair<LAddress::L3Type, innerIndexMap >(msg->getRecipientAddress(),inner_map));
		}else {
			/*
			 * there is a current entry for this address,
			 * we have to add this message to the existing entry
			 */
			bundlesIndexIterator it = bundlesIndex.find(msg->getRecipientAddress());
			it->second.insert(std::pair<int,WaveShortMessage*>(msg->getSerial(),msg));
		}
	}
}

void ProphetV2::executeInitiatorRole(short  kind, Prophet *prophetPkt, LAddress::L3Type destAddr)
{
	switch (kind) {
		case HELLO:
			break;
		case ERROR:
			break;
		case RIBD:
			break;
		case RIB:
		{
			Prophet *ribPkt = new Prophet();

			// aging of predictions that will be sent
			// creating a copy of preds
			std::map<LAddress::L3Type, double> tmp = std::map<LAddress::L3Type, double>();
			ageDeliveryPreds();
			tmp.insert(preds.begin(),preds.end());
			ribPkt = prepareProphet(RIB, myNetwAddr, destAddr, NULL, &tmp);
			ribPkt->setBitLength(headerLength);
//			sendDown(ribPkt);
//			uniform(0.001,1);
			sendDelayed(ribPkt,dblrand(),"lowerLayerOut");

			/*
			 * Collecting data
			 */
			updatingL3Sent();
			ribPkt->setHopCount(ribPkt->getHopCount()+1);
			updatingContactState(destAddr,RIB);
		}
			break;
		case Bundle_Offer:
			/*
			 * nothing to do for now
			 */
			if (withAck){
				std::list<BundleMeta> bundleToAcceptMeta;
				bundleToAcceptMeta = prophetPkt->getBndlmeta();
				for (std::list<BundleMeta>::iterator ackIt = bundleToAcceptMeta.begin(); ackIt !=bundleToAcceptMeta.end(); ++ackIt) {
//					Prophet_Struct::bndl_meta meta;
//					meta.recipientAddress = ackIt->recipientAddress;
//					meta.senderAddress = ackIt->senderAddress;
//					meta.serial = ackIt->serial;
//					meta.timestamp = ackIt->timestamp;
//					meta.bFlags = ackIt->bFlags;


					if (ackIt->getFlags() == Prophet_Enum::PRoPHET_ACK){
						if (acksIndex.find(ackIt->getSerial())==acksIndex.end()){
							/*
							 * 1 step : Adding ack to ack list
							 */

							if (acks.size()==ackStructureSize){
								int serial = acks.front().getSerial();
								acksIndex.erase(serial);
								acks.pop_front();
							}

							acksIndex.insert(std::pair<int, BundleMeta>(ackIt->getSerial(),*ackIt));
							acks.push_back(*ackIt);

							/*
							 * 2 step : Deleting the bundle from the storage
							 */

							existAndErase(*ackIt);
						}
					}
//					if (ackIt->getFlags()==Prophet_Enum::PRoPHET_ACK) {
//						if (acksIndex.find(meta.serial)==acksIndex.end()){
//
//							/*
//							 * 1 step : Adding ack to ack list
//							 */
//
//							if (acks.size()==ackStructureSize){
//								int serial = acks.front().serial;
//								acksIndex.erase(serial);
//								acks.pop_front();
//							}
//
//							acksIndex.insert(std::pair<int, Prophet_Struct::bndl_meta>(meta.serial,meta));
//							acks.push_back(meta);
//
//							/*
//							 * 2 step : Deleting the bundle from the storage
//							 */
//
//							if (exist(meta)){
//								bundlesIndexIterator it = bundlesIndex.find(meta.recipientAddress);
//								if (it != bundlesIndex.end()){
//									innerIndexMap innerMap(it->second);
//									innerIndexIterator it2 = innerMap.find(meta.serial);
//									if (it2 !=innerMap.end()){
//										WaveShortMessage* wsm = it2->second;
//										innerMap.erase(meta.serial);
//										bundles.remove(wsm);
//									}
//								}
//							}
//						}
//					}
				}
			}

			/*
			 * Collecting data
			 */
			updatingContactState(prophetPkt->getSrcAddr(),Bundle_Offer);
			break;
		case Bundle_Response:
		{
			std::list<BundleMeta> bundleToAcceptMeta;
			bundleToAcceptMeta = prophetPkt->getBndlmeta();
			
			/*
			 * step 1 : check if offered bundles are already stored in this node,
			 * in that case delete them from the offered list 
			 */ 
			for (std::list<BundleMeta>::iterator it2 = bundleToAcceptMeta.begin(); it2 !=bundleToAcceptMeta.end(); ++it2) {
				if (exist(*it2)){
					it2 = bundleToAcceptMeta.erase(it2);
				}
			}
			
			/*
			 * step 2 : sending the response
			 */
			
			Prophet *responsePkt = new Prophet();
			responsePkt = prepareProphet(Bundle_Response,myNetwAddr,prophetPkt->getSrcAddr(), &bundleToAcceptMeta);
			responsePkt->setBitLength(headerLength);
			sendDown(responsePkt);
			/*
			 * Collecting data
			 */
			updatingL3Sent();
			responsePkt->setHopCount(responsePkt->getHopCount()+1);
			updatingContactState(prophetPkt->getSrcAddr(),Bundle_Response);
		}
			break;
		case Bundle_Ack:{
			Prophet *ackPkt = new Prophet();

//			std::list<BundleMeta> acksMeta = std::list<BundleMeta>();
//			for (std::list<BundleMeta>::iterator ackIt = acks.begin(); ackIt !=acks.end(); ++ackIt) {
//				BundleMeta meta = new BundleMeta()
//				Prophet_Struct::bndl_meta meta;
//				meta.recipientAddress = ackIt->recipientAddress;
//				meta.senderAddress = ackIt->senderAddress;
//				meta.serial = ackIt->serial;
//				meta.timestamp = ackIt->timestamp;
//				meta.bFlags = ackIt->bFlags;
//				acksMeta.push_back(meta);
//			}

			std::list<BundleMeta> acksMeta (acks.begin(),acks.end());

			ackPkt = prepareProphet(Bundle_Ack, myNetwAddr, destAddr, &acksMeta);
			ackPkt->setBitLength(headerLength);
			sendDown(ackPkt);
			/*
			 * Collecting data
			 */
			updatingL3Sent();
		}
			break;
		case Bundle:
		{
			WaveShortMessage *wsm = check_and_cast<WaveShortMessage*>(prophetPkt->getEncapsulatedPacket());

			// Updating hopCount for WSM Message
			wsm->setHopCount(wsm->getHopCount()+1);

			LAddress::L3Type recipientAddr = wsm->getRecipientAddress();
			if ((recipientAddr==LAddress::L3BROADCAST)||(recipientAddr==myNetwAddr)){
				storeBundle(wsm->dup());
				sendUp(prophetPkt);

				if (withAck){
					if (acksIndex.find(wsm->getSerial())==acksIndex.end()){
						BundleMeta meta (wsm,Prophet_Enum::PRoPHET_ACK);
//						Prophet_Struct::bndl_meta meta;
//						meta.senderAddress = wsm->getSenderAddress();
//						meta.recipientAddress = wsm->getRecipientAddress();
//						meta.serial = wsm->getSerial();
//						meta.timestamp = wsm->getTimestamp();
//						meta.bFlags = Prophet_Enum::PRoPHET_ACK;

						if (acks.size()==ackStructureSize){
							int serial = acks.front().getSerial();
							acksIndex.erase(serial);
							acks.pop_front();
						}

						acksIndex.insert(std::pair<int, BundleMeta>(meta.getSerial(),meta));
						acks.push_back(meta);
					}
				}
			}else {
				wsm = check_and_cast<WaveShortMessage*>(prophetPkt->decapsulate());
				storeBundle(wsm);
			}

			/*
			 * Collecting data
			 */
			updatingContactState(prophetPkt->getSrcAddr(),Bundle);
		}
			break;
		default:
			opp_error("Unknown Prophetv2MessageKinds when calling executeInitiatorRole()");
			break;
	}
}

void ProphetV2::executeListenerRole(short  kind, Prophet *prophetPkt, LAddress::L3Type destAddr)
{
	switch (kind) {
		case HELLO:
			break;
		case ERROR:
			break;
		case RIBD:
			break;
		case RIB:
		{
			update(prophetPkt);
		}
			break;
		case Bundle_Offer:
		{
			defineBundleOffer(prophetPkt);
		}
			break;
		case Bundle_Response:
			/*
			 * nothing to do for now
			 */
			break;
		case Bundle_Ack:
		{
			std::list<BundleMeta> acksMeta;
			acksMeta = prophetPkt->getBndlmeta();

			for (std::list<BundleMeta>::iterator ackIt = acksMeta.begin(); ackIt !=acksMeta.end(); ++ackIt) {
				BundleMeta meta = *ackIt;
//				Prophet_Struct::bndl_meta meta;
//				meta.recipientAddress = ackIt->recipientAddress;
//				meta.senderAddress = ackIt->senderAddress;
//				meta.serial = ackIt->serial;
//				meta.timestamp = ackIt->timestamp;
//				meta.bFlags = ackIt->bFlags;

				if (acksIndex.find(meta.getSerial())==acksIndex.end()){

					/*
					 * 1 step : Adding ack to ack list
					 */

					if (acks.size()==ackStructureSize){
						int serial = acks.front().getSerial();
						acksIndex.erase(serial);
						acks.pop_front();
					}

					acksIndex.insert(std::pair<int, BundleMeta>(meta.getSerial(),meta));
					acks.push_back(meta);

					/*
					 * 2 step : Deleting the bundle from the storage
					 */

					if (exist(meta)){

						bundlesIndexIterator it = bundlesIndex.find(meta.getRecipientAddress());
						if (it != bundlesIndex.end()){
							innerIndexMap innerMap(it->second);
							innerIndexIterator it2 = innerMap.find(meta.getSerial());
							if (it2 !=innerMap.end()){
								WaveShortMessage* wsm = it2->second;
								innerMap.erase(meta.getSerial());
								bundles.remove(wsm);
							}
						}
					}
				}
			}
		}
			break;
		case Bundle:
		{
			std::list<BundleMeta> bundleToSendMeta;
			bundleToSendMeta = prophetPkt->getBndlmeta();
			for (std::list<BundleMeta>::iterator it = bundleToSendMeta.begin(); it !=bundleToSendMeta.end(); ++it) {
				bundlesIndexIterator it2 = bundlesIndex.find(it->getRecipientAddress());
				if (it2!=bundlesIndex.end()){
					innerIndexIterator it3 = it2->second.find(it->getSerial());
					if (it3!=it2->second.end()){
						Prophet *bundlePkt = new Prophet();
						bundlePkt = prepareProphet(Bundle,myNetwAddr,prophetPkt->getSrcAddr(),NULL,NULL,(it3->second)->dup());
						bundlePkt->setBitLength(headerLength);
						if (canITransmit){
							bundlePkt->setHopCount(bundlePkt->getHopCount()+1);
							sendDown(bundlePkt);

							/*
							 * Collecting data
							 */
							updatingL3Sent();
						}
					}
				}
			}
		}
			break;
		default:
			opp_error("Unknown Prophetv2MessageKinds when calling executeListenerRole()");
		break;
	}
}


Prophet *ProphetV2::prepareProphet(short  kind, LAddress::L3Type srcAddr,LAddress::L3Type destAddr, std::list<BundleMeta> *meta, std::map<LAddress::L3Type,double> *preds, WaveShortMessage *msg)
{

	Prophet *prophetMsg = new Prophet();
	prophetMsg->setKind(kind);
	prophetMsg->setSrcAddr(srcAddr);
	prophetMsg->setDestAddr(destAddr);
	if (meta!=NULL){
		prophetMsg->setBndlmeta(*meta);
	}
	if (preds!=NULL){
		prophetMsg->setPreds(*preds);
	}
	if (msg!=NULL){
		prophetMsg->encapsulate(msg);
	}

	return prophetMsg;
}


void ProphetV2::defineBundleOffer(Prophet *prophetPkt)
{
	LAddress::L3Type encounterdNode = prophetPkt->getSrcAddr();
	std::map<LAddress::L3Type, double> concernedPreds = std::map<LAddress::L3Type, double>();
	std::vector<std::pair<LAddress::L3Type, double>	> sortedPreds;
//	std::list<WaveShortMessage*> bundleToOffer = std::list<WaveShortMessage*>();
	std::list<BundleMeta> bundleToOfferMeta = std::list<BundleMeta>();

	// step 1 : check if we have any bundle that are addressed to @encouterdNode

	bundlesIndexIterator it = bundlesIndex.find(encounterdNode);
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2;
		for (it2 = innerMap.begin(); it2 !=innerMap.end(); ++it2){
//			bundleToOffer.push_back(it2->second);
			BundleMeta meta (it2->second, Prophet_Enum::Bndl_Accepted);
//			meta.senderAddress = it2->second->getSenderAddress();
//			meta.recipientAddress = it2->second->getRecipientAddress();
//			meta.serial = it2->second->getSerial();
//			meta.timestamp = it2->second->getTimestamp();
//			meta.bFlags = Prophet_Enum::Bndl_Accepted;
			bundleToOfferMeta.push_back(meta);
		}
	}

	// step 2 : check if we have any bundle that can be offered to @encouterdNode

	ageDeliveryPreds();
	predsIterator itCurrentNode;
	for (predsIterator itEncouterdNode =prophetPkt->getPreds().begin();itEncouterdNode !=prophetPkt->getPreds().end(); ++itEncouterdNode){
		itCurrentNode = preds.find(itEncouterdNode->first);
		if (itCurrentNode != preds.end()){
			// the current node have a prediction for @it->first 
			// the 2 nodes have a predictions for this destination
			if (itEncouterdNode->second > itCurrentNode->second){
				concernedPreds.insert(std::pair<LAddress::L3Type, double>(itEncouterdNode->first,itEncouterdNode->second));
			}
		}
	}

	switch (fwdStrategy) {
		case FWD_GRTR:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GTMX:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GTHR:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GRTRplus:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GTMXplus:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GRTRsort:
			opp_error("Forward Strategy not supported yet");
			break;
		case FWD_GRTRmax:
		{
			sortedPreds = std::vector<std::pair<LAddress::L3Type, double> >(concernedPreds.begin(), concernedPreds.end());
			std::sort(sortedPreds.begin(),sortedPreds.end(),fwdGRTRmax_CompObject);
		}
			break;
		default:
			break;
	}

	for (std::vector<std::pair<LAddress::L3Type, double> >::iterator it= sortedPreds.begin(); it != sortedPreds.end(); ++it){
		bundlesIndexIterator it2 = bundlesIndex.find(it->first);
			if (it2 != bundlesIndex.end()){
				innerIndexMap innerMap(it2->second);
				innerIndexIterator it3;
				for (it3 = innerMap.begin(); it3 !=innerMap.end(); ++it3){
//					bundleToOffer.push_back(it3->second);
					BundleMeta meta (it3->second, Prophet_Enum::Bndl_Accepted);
//					Prophet_Struct::bndl_meta meta;
//					meta.senderAddress = it3->second->getSenderAddress();
//					meta.recipientAddress = it3->second->getRecipientAddress();
//					meta.serial = it3->second->getSerial();
//					meta.timestamp = it3->second->getTimestamp();
//					meta.bFlags = Prophet_Enum::Bndl_Accepted;
					bundleToOfferMeta.push_back(meta);
				}
			}
	}

	if (withAck){
		for (std::list<BundleMeta>::iterator it = acks.begin(); it !=acks.end(); ++it) {
//			Prophet_Struct::bndl_meta meta;
//			meta.senderAddress = it->senderAddress;
//			meta.recipientAddress = it->recipientAddress;
//			meta.serial = it->serial;
//			meta.timestamp = it->timestamp;
//			meta.bFlags = it->bFlags;
			bundleToOfferMeta.push_back(*it);
		}
	}


	// step 3 : sending the ProphetPckt

	Prophet *offerPkt = new Prophet();
	offerPkt->setBitLength(headerLength);
	offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,encounterdNode,&bundleToOfferMeta);
	sendDown(offerPkt);
	/*
	 * Collecting data
	 */
	updatingL3Sent();
	offerPkt->setHopCount(offerPkt->getHopCount()+1);
}

bool ProphetV2::exist(WaveShortMessage *msg)
{
	bool found = false;
	bundlesIndexIterator it = bundlesIndex.find(msg->getRecipientAddress());
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(msg->getSerial());
		if (it2 !=innerMap.end()){
			found = true;
		}
	}
	return found;
}

bool ProphetV2::exist(BundleMeta bndlMeta)
{
	bool found = false;
	bundlesIndexIterator it = bundlesIndex.find(bndlMeta.getRecipientAddress());
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(bndlMeta.getSerial());
		if (it2 !=innerMap.end()){
			found = true;
		}
	}
	return found;
}

bool ProphetV2::existAndErase(BundleMeta bndlMeta)
{
	bool found = false;
	bundlesIndexIterator it = bundlesIndex.find(bndlMeta.getRecipientAddress());
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(bndlMeta.getSerial());
		if (it2 !=innerMap.end()){
			WaveShortMessage* wsm = it2->second;
			innerMap.erase(bndlMeta.getSerial());
			bundles.remove(wsm);
			found = true;
		}
	}
	return found;
}

LAddress::L3Type ProphetV2::getAddressFromName(const char *name)
{
	int addr = 0;
	if (strcmp(name,"")!=0){
		std::istringstream iss(name);
		iss >> addr;
	}
	return addr;
}

void ProphetV2::finish()
{
//	EV << "Sent:     " << nbrL3Sent << endl;
//	EV << "Received: " << nbrL3Received << endl;

	recordScalar("# L3sent", nbrL3Sent);
	recordScalar("# L3received", nbrL3Received);

	recordScalar("# of Contacts", nbrContacts);
	recordScalar("# of InterContacts", nbrRecontacts);

	recordScalar("# failed contacts before RIB", nbrFailedContactBeforeRIB);
	recordScalar("# failed contacts at RIB", nbrFailedContactAtRIB);
	recordScalar("# failed contacts at Bundle_Offer", nbrFailedContactAtBundle_Offer);
	recordScalar("# failed contacts at Bundle_Response", nbrFailedContactAtBundle_Response);
	recordScalar("# successful contacts", nbrSuccessfulContact);

	if (withAck){
		recordScalar("# ACKs", acks.size());
	}
}

/*******************************************************************
**
** 							Methods for collecting  datas & stats
**
********************************************************************/

void ProphetV2::recordPredsStats()
{
	// nbPreds = preds.size()-1, because P(X,X) is counted as a prediction for every node X
	int nbPreds = this->preds.size()-1;
	nbrPredsVector.record(nbPreds);

	if (!(nbPreds < 80)){
		EV << "Actual number of predictions : " << nbPreds << endl;
		for (predsIterator it=preds.begin(); it!=preds.end();it++){
			EV << "(key,value) : " << "(" << it->first <<"," << it->second << ")" << endl;
		}
	}

	double min = DBL_MAX, max = DBL_MIN, mean = 0, sum = 0, variance = 0, varianceSum = 0;

	for (predsIterator it=preds.begin(); it!=preds.end();it++){
		if (it->first != myNetwAddr){
			// Minimum value
			if (it->second < min){
				min = it->second;
			}

			// Maximum value
			if (it->second > max){
				max = it->second;
			}

			// Summing values
			sum+=it->second;
		}
	}

	// Calculating mean and variance
	if (nbPreds <=1){
		// Calculating mean
		mean = 0;

		// Calculating variance
		variance = 0;
	} else {
		// Calculating mean
		mean = sum / double (nbPreds);

		// Calculating variance
		for (predsIterator it=preds.begin(); it!=preds.end();it++){
			if (it->first != myNetwAddr){
				varianceSum += pow((it->second - mean),2);
			}
		}
		variance = varianceSum / double (nbPreds);
	}

	// recording values
	predsMin.record(min);
	predsMax.record(max);
	predsMean.record(mean);
	predsVariance.record(variance);

//	std::pair<LAddress::L3Type, double> predsMin = std::min_element(preds.begin(),preds.end());
//	std::pair<LAddress::L3Type, double> predsMax = std::max_element(preds.begin(),preds.end());

}

void ProphetV2::recordBeginContactStats(LAddress::L3Type addr, double time)
{
	// updating nbr contacts
	nbrContacts++;
	// saving the starting time of the contact
	contacts.insert(std::pair<LAddress::L3Type, double>(addr, time));
}

void ProphetV2::recordEndContactStats(LAddress::L3Type addr, double time)
{
	sumOfContactDur+=time - contacts.find(addr)->second;
	contactDurVector.record(sumOfContactDur/ double (nbrContacts));

	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		switch (it->second) {
		case RIB:
			nbrFailedContactAtRIB++;
			break;
		case Bundle_Offer:
			nbrFailedContactAtBundle_Offer++;
			break;
		case Bundle_Response:
			nbrFailedContactAtBundle_Response++;
			break;
		case Bundle:
			nbrSuccessfulContact++;
			break;
		default:
			nbrFailedContactBeforeRIB++;
			break;
		}

		contacts.erase(addr);
		contactState.erase(addr);
	}
}

void ProphetV2::recordRecontactStats(LAddress::L3Type addr, double time)
{
	std::map<LAddress::L3Type, double>::iterator it = lastEncouterTime.find(addr);
	if (it != lastEncouterTime.end()){
		// updating nbr repeated contacts nodes
		nbrRecontacts++;
		// updating vector stats for recontacted nodes
		sumOfInterContactDur+=time - it->second;
		intercontactDurVector.record(sumOfInterContactDur/ double (nbrRecontacts));
	}

}

void ProphetV2::updatingContactState(LAddress::L3Type addr, Prophetv2MessageKinds kind)
{
	std::map<LAddress::L3Type, Prophetv2MessageKinds>::iterator it = contactState.find(addr);
	if (it != contactState.end()){
		contactState.erase(addr);
	}
	contactState.insert(std::pair<LAddress::L3Type, Prophetv2MessageKinds>(addr,kind));

}




/*******************************************************************
**
** 							Unused functions
**
********************************************************************/
void ProphetV2::handleSelfMsg(cMessage* msg)
{

}

void ProphetV2::handleUpperControl(cMessage* msg)
{

}

cObject *const ProphetV2::setDownControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr)
{
	return BaseNetwLayer::setDownControlInfo(pMsg, pDestAddr);
}

cObject *const ProphetV2::setUpControlInfo(cMessage *const pMsg, const LAddress::L3Type& pSrcAddr)
{
	return BaseNetwLayer::setUpControlInfo(pMsg, pSrcAddr);
}

ProphetV2::~ProphetV2()
{

}
