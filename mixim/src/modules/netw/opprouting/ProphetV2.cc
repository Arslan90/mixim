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

        deletedBundlesWithAck = 0;

        demandedAckedBundle = 0;

        notCorrectlyDeleted = 0;

        bundlesReceived = 0;

        global = ClassifiedContactStats("Global");
        successful = ClassifiedContactStats("Successful");
        failed = ClassifiedContactStats("Failed");

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

    double macArrival = m->getArrivalTime().dbl();
    double macSent = m->getSendingTime().dbl();

    Prophet *prophetPkt = check_and_cast<Prophet *>(m->decapsulate());

    double prophetArrival = prophetPkt->getArrivalTime().dbl();
	double prophetSent = prophetPkt->getSendingTime().dbl();

	double timeT = simTime().dbl();
	double createTime = prophetPkt->getCreationTime().dbl();

	SimpleContactStats contact = getSimpleContactStats(prophetPkt->getDestAddr(),prophetPkt->getCreationTime().dbl());
    contact.setL3Received();

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

					/*
					 * Note : this phase is responsible of triggering the executeInitiatorRole(Bundle_Ack,prophetPkt);
					 */
					executeInitiatorRole(Bundle,prophetPkt);

//					if (withAck){
//						executeInitiatorRole(Bundle_Ack,prophetPkt);
//					}
				}
				break;
			default:
				opp_error("Unknown Prophetv2MessageKinds when calling HandleLowerMsg()");
				break;
		}
    }
//    cancelAndDelete(prophetPkt);
    delete(prophetPkt);
//    delete(m);
    delete(msg);

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

					recordBeginSimplContactStats(addr,time);

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

				recordEndSimpleContactStats(addr,time);
			}
			break;
	}
	delete msg;
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

void ProphetV2::storeACK(BundleMeta meta)
{
	if (acksIndex.find(meta.getSerial())==acksIndex.end()){
		if (acks.size()==ackStructureSize){
			int serial = acks.front().getSerial();
			acksIndex.erase(serial);
			acks.pop_front();
		}

		acksIndex.insert(std::pair<int, BundleMeta>(meta.getSerial(),meta));
		acks.push_back(meta);

		if (existAndErase(meta)) {
			deletedBundlesWithAck++;
		}
	}
}

void ProphetV2::executeInitiatorRole(short  kind, Prophet *prophetPkt, LAddress::L3Type destAddr)
{

	// if destAddr equals 0 then destAddr is  the sender of the previously received prophet packet

	if (destAddr == 0) {
		coreEV << "destAddr equal 0 (null) in Bundle_Ack of Initiator Role. destAddr will be recalculated";
		destAddr = prophetPkt->getSrcAddr();
	}

	SimpleContactStats contact;

//	ASSERT(prophetPkt->getCreationTime().dbl());
//	double time = (prophetPkt->getCreationTime()).dbl();
	if (prophetPkt!=NULL){
		contact = getSimpleContactStats(destAddr,prophetPkt->getCreationTime().dbl() );
		contact.setState(kind);
	}else {
		contact = getSimpleContactStats(destAddr, simTime().dbl());
		contact.setState(kind);
	}


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
			updatingContactState(destAddr,RIB);

			contact.setL3Sent();
			contact.setPredictionsSent(tmp.size());
		}
			break;
		case Bundle_Offer:
		{
			std::list<BundleMeta> bundleToAcceptMeta;
			bundleToAcceptMeta = prophetPkt->getBndlmeta();

			if (bundleToAcceptMeta.size()==0){
				/*
				 * Nothing to do, we have to stop the exchange
				 */
				updatingContactState(destAddr,Bundle);
				contact.setSuccessfulContact(true);
			}else {

				for (std::list<BundleMeta>::iterator it = bundleToAcceptMeta.begin(); it !=bundleToAcceptMeta.end(); ++it) {
					if (it->getFlags() == Prophet_Enum::Bndl_Accepted){
						/*
						 * step 1 : check if offered bundles are already stored in this node,
						 * in that case delete them from the offered list
						 */
						if (exist(*it)){
							it = bundleToAcceptMeta.erase(it);
						}

						/*
						 * step 2 : check if offered bundles are already acked,
						 * in that case delete them from the offered list
						 */
						if (withAck){
							if (acksIndex.find(it->getSerial()) != acksIndex.end()){
								it = bundleToAcceptMeta.erase(it);
								demandedAckedBundle++;
							}
						}
					}
					if (it->getFlags() == Prophet_Enum::PRoPHET_ACK){
						/*
						 * step 3 : check if BundleMeta is ACK,
						 * in that case add it and delete the corresponding bundle from storage
						 */
						storeACK(*it);
						contact.setAckReceived();
						}
				}
				/*
				 * Collecting data
				 */
				updatingContactState(destAddr,Bundle_Offer);
			}


//				if (withAck){
//					std::list<BundleMeta> bundleToAcceptMeta;
//					bundleToAcceptMeta = prophetPkt->getBndlmeta();
//					for (std::list<BundleMeta>::iterator ackIt = bundleToAcceptMeta.begin(); ackIt !=bundleToAcceptMeta.end(); ++ackIt) {
//
//						if (ackIt->getFlags() == Prophet_Enum::PRoPHET_ACK){
//
//							if (acksIndex.find(ackIt->getSerial())==acksIndex.end()){
//								/*
//								 * 1 step : Adding ack to ack list
//								 */
//
//								if (acks.size()==ackStructureSize){
//									int serial = acks.front().getSerial();
//									acksIndex.erase(serial);
//									acks.pop_front();
//								}
//
//								acksIndex.insert(std::pair<int, BundleMeta>(ackIt->getSerial(),*ackIt));
//								acks.push_back(*ackIt);
//
//								/*
//								 * 2 step : Deleting the bundle from the storage
//								 */
//
//								if (existAndErase(*ackIt)) {
//									deletedBundlesWithAck++;
//								}
//							}
//						}
//					}
//				}


		}

			break;
		case Bundle_Response:
		{
//			std::list<BundleMeta> bundleToAcceptMeta;
//			bundleToAcceptMeta = prophetPkt->getBndlmeta();
//
//
//
//			/*
//			 * step 1 : check if offered bundles are already stored in this node,
//			 * in that case delete them from the offered list
//			 */
//			for (std::list<BundleMeta>::iterator it2 = bundleToAcceptMeta.begin(); it2 !=bundleToAcceptMeta.end(); ++it2) {
//				if (exist(*it2)){
//					it2 = bundleToAcceptMeta.erase(it2);
//				}
//				if(withAck){
//					if (acksIndex.find(it2->getSerial())!=acksIndex.end()){
//						it2 = bundleToAcceptMeta.erase(it2);
//						demandedAckedBundle++;
//					}
//				}
//			}
//
//			/*
//			 * step 2 : sending the response
//			 */

			std::list<BundleMeta> bundleToAcceptMeta;
			bundleToAcceptMeta = prophetPkt->getBndlmeta();

			if (bundleToAcceptMeta.size() == 0){
				/*
				 * Nothing to do, we have to stop the exchange
				 */
				updatingContactState(destAddr,Bundle);
				contact.setSuccessfulContact(true);
			}else{

				/*
				 * step 1 : sending the response
				 */

				Prophet *responsePkt = new Prophet();
				responsePkt = prepareProphet(Bundle_Response,myNetwAddr,destAddr, &bundleToAcceptMeta);
				responsePkt->setBitLength(headerLength);
				sendDown(responsePkt);

				/*
				 * Collecting data
				 */
				updatingL3Sent();
				updatingContactState(destAddr,Bundle_Response);

				contact.setL3Sent();
			}
		}
			break;
		case Bundle_Ack:{

			Prophet *ackPkt = new Prophet();
			WaveShortMessage *wsm = check_and_cast<WaveShortMessage*>(prophetPkt->getEncapsulatedPacket());

			/*
			 * Step 1 : Creating the ACK
			 */
			BundleMeta meta (wsm,Prophet_Enum::PRoPHET_ACK);
			storeACK(meta);

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

			/*
			 * No need to send all acks because it's already done in Bundle_Offer, send only ack of the received bundle
			 */
//			std::list<BundleMeta> acksMeta (acks.begin(),acks.end());

			/*
			 * Step 2 : Sending the ACK
			 */

			std::list<BundleMeta> acksMeta;
			if (acksIndex.find(meta.getSerial())!= acksIndex.end()){
				acksMeta.push_back(meta);
			}

			ackPkt = prepareProphet(Bundle_Ack, myNetwAddr, destAddr, &acksMeta);
			ackPkt->setBitLength(headerLength);
			sendDown(ackPkt);
			/*
			 * Collecting data
			 */
			updatingL3Sent();

			contact.setL3Sent();
			contact.setAckSent();
			contact.setSuccessfulContact(true);
		}
			break;
		case Bundle:
		{
//			if (!withAck){
//				contact.setSuccessfulContact(true);
//			}

			contact.setSuccessfulContact(true);

			bool shouldAbort = false;
			WaveShortMessage *wsm;

			if (prophetPkt->getEncapsulatedPacket() == NULL){
				shouldAbort = true;
			}else {

				bundlesReceived++;

				contact.setBundleReceived();

				wsm = check_and_cast<WaveShortMessage*>(prophetPkt->getEncapsulatedPacket());

				if (bundlesIndex.find(wsm->getSerial())!=bundlesIndex.end()){
					shouldAbort = true;
				}

				if ((withAck)&&(acksIndex.find(wsm->getSerial())!=acksIndex.end())){
					shouldAbort = true;
				}
			}

			LAddress::L3Type recipientAddr = wsm->getRecipientAddress();

			if (!shouldAbort){
				// Updating hopCount for WSM Message
				wsm->setHopCount(wsm->getHopCount()+1);


//				if ((recipientAddr==LAddress::L3BROADCAST)||(recipientAddr==myNetwAddr)){
//					if (recipientAddr != myNetwAddr){
//						storeBundle(wsm->dup());
//					}
//					sendUp(prophetPkt);
//					Prophet *toSentUp = prophetPkt->dup();
//					sendUp(toSentUp);
				if (recipientAddr == myNetwAddr){
					sendUp(prophetPkt->dup());
					if (withAck){
						executeInitiatorRole(Bundle_Ack,prophetPkt);
					}
				}else {
					wsm = check_and_cast<WaveShortMessage*>(prophetPkt->decapsulate());
					storeBundle(wsm);
				}
			}

			if ((shouldAbort)&&(recipientAddr == myNetwAddr)){
				sendUp(prophetPkt->dup());
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
	// if destAddr equals 0 then destAddr is  the sender of the previously received prophet packet

	if (destAddr == 0) {
		coreEV << "destAddr equal 0 (null) in Bundle_Ack of Initiator Role. destAddr will be recalculated";
		destAddr = prophetPkt->getSrcAddr();
	}

	SimpleContactStats contact;
	contact = getSimpleContactStats(destAddr, prophetPkt->getCreationTime().dbl());

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
			contact.setPredictionsReceived(prophetPkt->getPreds().size());
		}
			break;
		case Bundle_Offer:
		{
			/*
			 * Step 1 : Calculating Bundle to Offer
			 */
			std::list<BundleMeta> bundleToOfferMeta = defineBundleOffer(prophetPkt);

			/*
			 * Step 2 : Sending the ProphetPckt
			 */

			Prophet *offerPkt = new Prophet();
			offerPkt->setBitLength(headerLength);
			offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,destAddr,&bundleToOfferMeta);
			sendDown(offerPkt);
			/*
			 * Collecting data
			 */
			updatingL3Sent();
			contact.setL3Sent();
		}
			break;
		case Bundle_Response:
		{
			if (withAck){

				/*
				 * 1 step : Check if demanded bundle in bundleResp is currently acked, delete it if it's the case
				 */

				std::list<BundleMeta> bundleToSendMeta;
				bundleToSendMeta = prophetPkt->getBndlmeta();
				for (std::list<BundleMeta>::iterator it = bundleToSendMeta.begin(); it !=bundleToSendMeta.end(); ++it) {
					if (acksIndex.find(it->getSerial()) != acksIndex.end()){
						it = bundleToSendMeta.erase(it);
					}
				}

			}
		}
			break;
		case Bundle_Ack:
		{
			std::list<BundleMeta> acksMeta;
			acksMeta = prophetPkt->getBndlmeta();

			for (std::list<BundleMeta>::iterator ackIt = acksMeta.begin(); ackIt !=acksMeta.end(); ++ackIt) {
				BundleMeta meta = *ackIt;
				storeACK(meta);
				contact.setAckReceived();

//				if (acksIndex.find(meta.getSerial())==acksIndex.end()){
//
//					/*
//					 * 1 step : Adding ack to ack list
//					 */
//
//					if (acks.size()==ackStructureSize){
//						int serial = acks.front().getSerial();
//						acksIndex.erase(serial);
//						acks.pop_front();
//					}
//
//					acksIndex.insert(std::pair<int, BundleMeta>(meta.getSerial(),meta));
//					acks.push_back(meta);
//
//					/*
//					 * 2 step : Deleting the bundle from the storage
//					 */
//
//					if (existAndErase(*ackIt)) {
//						deletedBundlesWithAck++;
//					}
//				}
			}
		}
			break;
		case Bundle:
		{
			std::list<BundleMeta> bundleToSendMeta;
			bundleToSendMeta = prophetPkt->getBndlmeta();

			if (bundleToSendMeta.size() == 0){
				/*
				 * No Bundle to transmit, send a prophet msg with NULL pointer instead of an encapsulated bundle
				 */
				Prophet *bundlePkt = new Prophet();
				bundlePkt = prepareProphet(Bundle,myNetwAddr,destAddr,NULL,NULL,NULL);
				bundlePkt->setBitLength(headerLength);
				if (canITransmit){
					sendDown(bundlePkt);

					/*
					 * Collecting data
					 */
					updatingL3Sent();
					contact.setL3Sent();
				}
			}else{
				/*
				 * 1 step : Send the corresponding bundle
				 */
				for (std::list<BundleMeta>::iterator it = bundleToSendMeta.begin(); it !=bundleToSendMeta.end(); ++it) {
					bundlesIndexIterator it2 = bundlesIndex.find(it->getRecipientAddress());
					if (it2!=bundlesIndex.end()){
						innerIndexIterator it3 = it2->second.find(it->getSerial());
						if (it3!=it2->second.end()){
							Prophet *bundlePkt = new Prophet();
							bundlePkt = prepareProphet(Bundle,myNetwAddr,destAddr,NULL,NULL,(it3->second)->dup());
							bundlePkt->setBitLength(headerLength);
							if (canITransmit){
								sendDown(bundlePkt);

								/*
								 * Collecting data
								 */
								updatingL3Sent();
								contact.setL3Sent();
								contact.setBundleSent();
							}
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


std::list<BundleMeta> ProphetV2::defineBundleOffer(Prophet *prophetPkt)
{
	SimpleContactStats contact = getSimpleContactStats(prophetPkt->getSrcAddr(),prophetPkt->getCreationTime().dbl());

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
			BundleMeta meta (it2->second, Prophet_Enum::Bndl_Accepted);

			if (withAck){
				if (acksIndex.find(it2->second->getSerial())!=acksIndex.end()){
					existAndErase(meta);
					notCorrectlyDeleted++;
					continue;
				}
			}
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
					BundleMeta meta (it3->second, Prophet_Enum::Bndl_Accepted);

					if (withAck){
						if (acksIndex.find(it3->second->getSerial())!=acksIndex.end()){
							existAndErase(meta);
							notCorrectlyDeleted++;
							continue;
						}
					}
					bundleToOfferMeta.push_back(meta);
				}
			}
	}

	// step 3 : check if we have any ack that must be transmitted

	if (withAck){
		for (std::list<BundleMeta>::iterator it = acks.begin(); it !=acks.end(); ++it) {
			if (existAndErase(*it)){
				notCorrectlyDeleted++;
				continue;
			}
			bundleToOfferMeta.push_back(*it);
			contact.setAckSent();
		}
	}


//	// step 3 : sending the ProphetPckt
//
//	Prophet *offerPkt = new Prophet();
//	offerPkt->setBitLength(headerLength);
//	offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,encounterdNode,&bundleToOfferMeta);
//	sendDown(offerPkt);
//	/*
//	 * Collecting data
//	 */
//	updatingL3Sent();
//	offerPkt->setHopCount(offerPkt->getHopCount()+1);

	return bundleToOfferMeta;
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

	recordScalar("# BundlesAtL3", bundlesReceived);

	if (withAck){
		recordScalar("# ACKs", acks.size());
		recordScalar("# DeletedBundles", deletedBundlesWithAck);
		recordScalar("# DemandedAckedBundles", demandedAckedBundle);
		recordScalar("# NotCorrectlyDeleted", notCorrectlyDeleted);
		recordScalar("# Bundles", bundles.size());
	}

	classifyRemaining();
	global.getDurationStats().recordAs(global.getName().c_str());
	successful.getDurationStats().recordAs(successful.getName().c_str());
	failed.getDurationStats().recordAs(failed.getName().c_str());
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

//	simpleContacts.insert(std::pair<LAddress::L3Type, SimpleContactStats>(addr, SimpleContactStats(time)));
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

void ProphetV2::recordBeginSimplContactStats(LAddress::L3Type addr, double time)
{
//	bool repeated = false;
//	std::map<LAddress::L3Type, SimpleContactStats>::iterator it = simpleContacts.find(addr);
//	if ((it != simpleContacts.end())&&(lastEncouterTime.find(addr)!=lastEncouterTime.end())){
//		classify(it->second);
//		repeated = true;
//		simpleContacts.erase(addr);
//	}
//	simpleContacts.insert(std::pair<LAddress::L3Type, SimpleContactStats>(addr,SimpleContactStats(time,repeated)));
	SimpleContactStats contact;
//	std::map<LAddress::L3Type, std::list<SimpleContactStats> >::iterator it = simpleContacts.find(addr);
//	if (it == simpleContacts.end()){
//		contact = getSimpleContactStats(addr,time);
//	}else {
//		contact = it->second.front();
//	}
	contact = getSimpleContactStats(addr,time);


	int x;

	std::map<LAddress::L3Type, std::set<SimpleContactStats> >::iterator it = simpleContacts.find(addr);
	if (it != simpleContacts.end()){
		x = it->second.size();
		it->second.erase(contact);
		contact.setStartTime(time);
		x = it->second.size();
		it->second.insert(contact);
		x = it->second.size();
	}
}

void ProphetV2::recordEndSimpleContactStats(LAddress::L3Type addr, double time)
{
	SimpleContactStats contact;
//	std::map<LAddress::L3Type, std::list<SimpleContactStats> >::iterator it = simpleContacts.find(addr);
//	if (it == simpleContacts.end()){
//		opp_error("recording end of simple contact that doesn't IMPOSSIBLE");
//	}else {
//		contact = it->second.front();
//	}

	std::map<LAddress::L3Type, std::set<SimpleContactStats> >::iterator it = simpleContacts.find(addr);
	std::set<SimpleContactStats>::iterator it2;
	if (it != simpleContacts.end()){
		it2= it->second.end();
		it2--;
		contact = *it2;
		contact.setEndTime(time);
	}



//	SimpleContactStats contact = getSimpleContactStats(addr);
//	contact.setDuration(time - contact.getStartTime());
}

void ProphetV2::classify(SimpleContactStats newContact)
{
	global.update(newContact);

	if (newContact.isSuccessfulContact()){
		successful.update(newContact);
	}else{
		failed.update(newContact);
	}
}

void ProphetV2::classifyRemaining()
{
	for(std::map<LAddress::L3Type, std::set<SimpleContactStats> >::iterator it = simpleContacts.begin(); it != simpleContacts.end(); it++){
		for (std::set<SimpleContactStats>::iterator it2 = it->second.begin(); it2 !=it->second.end(); it2++){
			if (it2->getEndTime()==std::numeric_limits<double>::max()){
				double t = simTime().dbl();
				SimpleContactStats tmp = *it2;
				tmp.setEndTime(t);
			}
			classify(*it2);
		}
	}
}

SimpleContactStats ProphetV2::getSimpleContactStats(LAddress::L3Type addr, double creationTime)
{
	SimpleContactStats contact = NULL;
	std::map<LAddress::L3Type, std::set<SimpleContactStats> >::iterator it = simpleContacts.find(addr);

	SimpleContactStats tempContact2, tempContact3;
	int x;

	if (it==simpleContacts.end()){
		x = it->second.size();
		contact = SimpleContactStats();
		std::set<SimpleContactStats> mySet;
		mySet.insert(contact);
		simpleContacts.insert(std::pair<LAddress::L3Type, std::set<SimpleContactStats> >(addr,mySet));
	}else{
		SimpleContactStats tempContact = SimpleContactStats(creationTime);
		std::set<SimpleContactStats>::iterator it2 = it->second.lower_bound(tempContact);
		std::set<SimpleContactStats>::iterator it3 = it2;
		x = it->second.size();
		tempContact2 = *it2;
		if (it2 != it->second.end()){
			if ((it2 == it->second.begin())||(it2->getStartTime() == creationTime)){
				contact = *it2;
			}else {
				it3--;
				tempContact3 = *it3;
				if (it3->getEndTime() != std::numeric_limits<double>::max()){
					if ((creationTime-it3->getEndTime()) > 1.0){
						contact = *it2;
					}else if ((creationTime-it3->getEndTime()) <= 1.0){
						contact = *it3;
					}
				}
//				if ((it2->getStartTime() == std::numeric_limits<double>::max()-1) &&
//						(creationTime-it3->getEndTime()>1.0)){
//					contact = *it2;
//				}else if (creationTime-it3->getEndTime()<=1){
//					contact = *it3;
//				}
			}
		}else {
			contact = SimpleContactStats();
			std::set<SimpleContactStats> mySet;
			mySet.insert(contact);
			simpleContacts.insert(std::pair<LAddress::L3Type, std::set<SimpleContactStats> >(addr,mySet));
//			opp_error("recording end of simple contact that doesn't exist");
		}
	}

	if (contact == NULL){
		opp_error("recording end of simple contact that doesn't exist");
	}

//		if (it2 == it->second.begin()){
//			if (it2->getStartTime() == std::numeric_limits<double>::max()-1){
//				contact = *it2;
//			}else {
//				contact = *it2;
//				contact = NULL;
////				opp_error("recording end of simple contact that doesn't exist");
//			}
//		}else {
//			std::set<SimpleContactStats>::iterator it3 = it2;
//			it3--;
//			contact = *it3;
//		}
//	}
//		if (it2->getStartTime() == creationTime){
//			contact = *it2;
//		} else {
//			std:set<SimpleContactStats>::iterator it3 = it2;
//			bool valid
//			if (it3 != it->second.begin()){
//				it3--;
//			}
//
////			std::reverse_iterator<SimpleContactStats> revIt(*it2);
//			if (revIt !=it->second.rend()){
//				contact = revIt;
//			}else{
//				opp_error("recording end of simple contact that doesn't exist");
//			}
//		}
//	}


//	if (it == simpleContacts.end()){
//		contact = SimpleContactStats();
//		std::list<SimpleContactStats> list = std::list<SimpleContactStats>();
//		list.push_front(contact);
//		simpleContacts.insert(std::pair<LAddress::L3Type, std::list<SimpleContactStats> >(addr,list));
////		it->second = std::list<SimpleContactStats>();
////		it->second.push_front(contact);
//	}else{
//		/* return the contactStats that belong to the given creationTime*/
//
//		for (std::list<SimpleContactStats>::reverse_iterator it2 = it->second.rbegin(); it2!= it->second.rend(); it2++ ){
//			if (std::distance(it2,it->second.rend())==1){
//				// check if it is the last element of the list
////				opp_error("recording end of simple contact that doesn't exist");
//
//				if (it2->getStartTime()!=-std::numeric_limits<double>::max()){
//					contact = *it2;
//				}else {
//					if (it2->getStartTime()>creationTime){
//						opp_error("recording of simple contact that doesn't exist");
//					}else if (creationTime<it2->getEndTime()){
//						contact = *it2;
//					}else{
//						contact = SimpleContactStats();
//						it->second.push_front(contact);
//					}
//				}
//				break;
//			}
//
//			if ((it2->getEndTime()!=std::numeric_limits<double>::max())&&(creationTime<it2->getEndTime())){
//				contact = SimpleContactStats();
//				it->second.push_front(contact);
//				break;
//			}else {
//				if ((it2->getStartTime()!=-std::numeric_limits<double>::max()) &&
//						(it2->getStartTime()<creationTime) && (creationTime<it2->getEndTime())){
//					contact = *it2;
//					break;
//				}else {
//					continue;
//				}
//			}
//		}
//	}

//			if ((it2->getEndTime()==-1)){
//				if (it2->getStartTime()==-1){
//					contact = *it2;
//					break;
//				}else if (it2->getStartTime()>creationTime){
//					opp_error("recording end of simple contact that doesn't exist");
//					break;
//				}
//			}else if ((it2->getStartTime()<creationTime)&&(it2->getEndTime()>creationTime)){
//				contact = *it2;
//			}
//		}
//	}
//
//	if (lastEncouterTime.find(addr)!=lastEncouterTime.end()){
//		if (it == simpleContacts.end()){
//			opp_error("recording end of simple contact that doesn't exist");
//		}
//	}else{
//		recordBeginSimplContactStats(addr,simTime().dbl());
//	}

//	if ((it == simpleContacts.end())&&(lastEncouterTime.find(addr)!=lastEncouterTime.end())){
//		opp_error("recording end of simple contact that doesn't exist");
//	}
	return contact;
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
