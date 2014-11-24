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

		preds = std::map<LAddress::L3Type,double>();
		preds.insert(std::pair<LAddress::L3Type,double>(myNetwAddr,1));

		lastEncouterTime = std::map<LAddress::L3Type,double>();

		int tmp = par("storageSize");
		if (tmp>=0){
			bundlesStructureSize = tmp;
		}else {
			opp_error("Size of the structure that store bundles can not be negative");
		}
		bundles = std::list<WaveShortMessage*>();

		/*
		 * Collecting data & metrics
		 */
	    numSent = 0;
	    numReceived = 0;
	    WATCH(numSent);
	    WATCH(numReceived);

	    hopCountStats.setName("hopCountStats");
	    hopCountStats.setRangeAutoUpper(0, 10, 1.5);
	    hopCountVector.setName("HopCount");


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

//	    sentStats.setName("Sent Prophet message");
//	    sentStats.setRangeAutoUpper(0);
//	    sentVector.setName("Statistics for sent Prophet Message");
//	    receivedStats.setName("Received Prophet message");
//	    sentStats.setRangeAutoUpper(0);
//	    receivedVector.setName("Statistics for received Prophet Message");
	}
}
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
		predsForB = 0;
		lastEncTime = 0;
		PEnc = PEncMax;

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
		;
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
	}
	predsForB = predsForB + (1-predsForB)*PEnc;
	preds.insert(std::pair<LAddress::L3Type,double>(BAdress,predsForB));
	lastEncouterTime.insert(std::pair<LAddress::L3Type,double>(BAdress,encTime));

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
		predsForC=tmp_it->second;

		tmp_it= Bpreds.find(CAdress);
		BpredsForC=tmp_it->second;

		tmp_it= preds.find(BAdress);
		predsForB=tmp_it->second;

		if (predsForC<(predsForB*BpredsForC*Beta)){
			predsForC = predsForB*BpredsForC*Beta;
		}
		preds.insert(std::pair<LAddress::L3Type,double>(CAdress,predsForC));
	}
}

void ProphetV2::ageDeliveryPreds()
{
	double time = simTime().dbl();
	double timeDiff = (time-lastAgeUpdate)/secondsInTimeUnit;
	if (timeDiff==0){
		return;
	}else {
		double mult = std::pow(GAMMA, timeDiff);
		for (predsIterator it=preds.begin();it!=preds.end();it++){
			it->second = it->second * mult;
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

cMessage* ProphetV2::decapsMsg(NetwPkt *msg)
{
    cMessage *m = msg->decapsulate();
//    setUpControlInfo(m, msg->getSrcAddr());
    // delete the netw packet
//    delete msg;
    return m;
}

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
//	canITransmit();
	return pkt;
}

void ProphetV2::handleLowerMsg(cMessage* msg)
{

//    receivedVector.record(numReceived);
//    receivedStats.collect(numReceived);
//	NetwPkt *m = static_cast<NetwPkt *>(msg);
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;
    NetwPkt *netwpckt = check_and_cast<NetwPkt *>(m->decapsulate());
    Prophet *prophetPkt0 = check_and_cast<Prophet *>(netwpckt);

    int hopcount;

    if ((prophetPkt0->getDestAddr()==LAddress::L3BROADCAST)||(prophetPkt0->getDestAddr()==myNetwAddr)){

		switch (netwpckt->getKind()) {
			case HELLO:
				break;
			case ERROR:
				break;
			case RIBD:
				break;
			case RIB:
				{
					 // collecting data
						numReceived++;
						hopcount = prophetPkt0->getHopCount();
						hopCountVector.record(hopcount);
						hopCountStats.collect(hopcount);

					Prophet *prophetPkt = check_and_cast<Prophet *>(netwpckt);
					// first step : updating preds
					executeListenerRole(RIB,prophetPkt);
					// second step : starting of Bundle_Offer phase
					executeListenerRole(Bundle_Offer,prophetPkt);
				}
				break;
			case Bundle_Offer:
				{
					// collecting data
										numReceived++;
										hopcount = prophetPkt0->getHopCount();
										hopCountVector.record(hopcount);
										hopCountStats.collect(hopcount);

					Prophet *prophetPkt = check_and_cast<Prophet *>(netwpckt);
					executeInitiatorRole(Bundle_Offer,prophetPkt);
					executeInitiatorRole(Bundle_Response,prophetPkt);
				}
				break;
			case Bundle_Response:
				{
					// collecting data
										numReceived++;
										hopcount = prophetPkt0->getHopCount();
										hopCountVector.record(hopcount);
										hopCountStats.collect(hopcount);

					Prophet *prophetPkt = check_and_cast<Prophet *>(netwpckt);
					executeListenerRole(Bundle_Response,prophetPkt);
					executeListenerRole(Bundle,prophetPkt);
				}
				break;
			case Bundle:
				{
					// collecting data
										numReceived++;
										hopcount = prophetPkt0->getHopCount();
										hopCountVector.record(hopcount);
										hopCountStats.collect(hopcount);

					Prophet *prophetPkt = check_and_cast<Prophet *>(netwpckt);
					executeInitiatorRole(Bundle,prophetPkt);
				}
				break;
			default:
	//			WaveShortMessage *tmp = check_and_cast<WaveShortMessage*>(netwpckt->getEncapsulatedPacket());
	//			WaveShortMessage *wsm = tmp->dup();
	//			if (LAddress::isL3Broadcast(m->getDestAddr())){
	//				storeBundle(wsm);
	//			}
	//			sendUp(netwpckt);
				break;
		}
	//    if (netwpckt->getKind()==RIB){
	//    	updateDeliveryPredsFor(netwpckt->getSrcAddr());
	//    }else {
	//    	WaveShortMessage *tmp = check_and_cast<WaveShortMessage*>(netwpckt->getEncapsulatedPacket());
	//		WaveShortMessage *wsm = tmp->dup();
	//		if (LAddress::isL3Broadcast(m->getDestAddr())){
	//			storeBundle(wsm);
	//		}
	//		sendUp(netwpckt);
	//    	//    WaveShortMessage *tmp = static_cast<WaveShortMessage*>(decapsMsg(m));
	//    	//    WaveShortMessage *dup = tmp->dup();
	//    	//    if (LAddress::isL3Broadcast(m->getDestAddr())||(m->getDestAddr()==myNetwAddr)){
	//    	//    	sendUp(decapsMsg(m));
	//    	//    }
	//
	//    	//    WaveShortMessage *wsm = check_and_cast<WaveShortMessage*>(m->getEncapsulatedPacket());
	//    	//    cMessage *tmp =decapsMsg(m);
	//    	//    WaveShortMessage *copy = wsm->dup();
	//    	//	if (LAddress::isL3Broadcast(m->getDestAddr())){
	//    	//		storeBundle(copy);
	//    	//	}
	//    	//    sendUp(tmp);
	//
	//    	//    sendUp(msg);
	//
	//    }
    } else {
    	delete(msg);
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
					string tmp = msg->getName();
					int addr = 0;
					if (tmp!=""){
						std::istringstream iss(tmp);
						iss >> addr;
					}
					LAddress::L3Type destAddr = addr;

					/** Intercontacts duration stats 			*/
					std::map<LAddress::L3Type, double>::iterator it = lastEncouterTime.find(destAddr);
					if (it != lastEncouterTime.end()){
						double duration = simTime().dbl() - it->second;
						nbrRecontacts++;
						sumOfInterContactDur+=duration;
						if (nbrRecontacts!=0){
							intercontactDurVector.record(sumOfInterContactDur/ double (nbrRecontacts));
						}
					}

					executeInitiatorRole(RIB,NULL,destAddr);

					/** Contacts duration stats				 */
					contacts.insert(std::pair<LAddress::L3Type, double>(destAddr, simTime().dbl()));
				}
			}
			break;
		case NO_NEIGHBOR_AND_DISCONNECTED:
			canITransmit = false;
			break;
		case NEW_NEIGHBOR_GONE:
			{
				string tmp = msg->getName();
				int addr = 0;
				if (tmp!=""){
					std::istringstream iss(tmp);
					iss >> addr;
				}
				LAddress::L3Type destAddr = addr;

				/** Contacts duration stats				 */
				double duration = simTime().dbl() - (contacts.find(destAddr)->second);
				nbrContacts++;
				sumOfContactDur+=duration;
				contacts.erase(destAddr);
				if (nbrContacts!=0){
					contactDurVector.record(sumOfContactDur/ double (nbrContacts));
				}
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

//			ribPkt->setBitLength(headerLength);
//			ribPkt->setKind(RIB);
//			ribPkt->setSrcAddr(myNetwAddr);
//			ribPkt->setDestAddr(LAddress::L3BROADCAST);

			// aging of predictions that will be sent
			// creating a copy of preds
			std::map<LAddress::L3Type, double> tmp = std::map<LAddress::L3Type, double>();
			ageDeliveryPreds();
			tmp.insert(preds.begin(),preds.end());
//			ribPkt->setPreds(tmp);
//			ribPkt = prepareProphet(RIB, myNetwAddr, LAddress::L3BROADCAST, NULL, &tmp);
			ribPkt = prepareProphet(RIB, myNetwAddr, destAddr, NULL, &tmp);
			ribPkt->setBitLength(headerLength);
//			sendDown(ribPkt);
//			uniform(0.001,1);
			sendDelayed(ribPkt,dblrand(),"lowerLayerOut");
			/*
			 * Collecting data
			 */
			numSent++;
			ribPkt->setHopCount(ribPkt->getHopCount()+1);

		}
			break;
		case Bundle_Offer:
			/*
			 * nothing to do for now
			 */
			break;
		case Bundle_Response:
		{
			std::list<Prophet_Struct::bndl_meta> bundleToAcceptMeta;
			bundleToAcceptMeta = prophetPkt->getBndlmeta();
			
			/*
			 * step 1 : check if offered bundles are already stored in this node,
			 * in that case delete them from the offered list 
			 */ 
			for (std::list<Prophet_Struct::bndl_meta>::iterator it = bundleToAcceptMeta.begin(); it !=bundleToAcceptMeta.end(); ++it) {
				if (exist(*it)){
					it = bundleToAcceptMeta.erase(it);
				}
			}
			
			/*
			 * step 2 : sending the response
			 */
			
			Prophet *responsePkt = new Prophet();
			responsePkt = prepareProphet(Bundle_Response,myNetwAddr,prophetPkt->getSrcAddr(), &bundleToAcceptMeta);
			responsePkt->setBitLength(headerLength);
//			responsePkt->setKind(Bundle_Response);
//			responsePkt->setSrcAddr(myNetwAddr);
//			responsePkt->setDestAddr(prophetPkt->getSrcAddr());
//			responsePkt->setBndlmeta(bundleToAcceptMeta);
			sendDown(responsePkt);
			/*
			 * Collecting data
			 */
			numSent++;
			responsePkt->setHopCount(responsePkt->getHopCount()+1);
		}
			break;
		case Bundle:
		{
			WaveShortMessage *tmp = check_and_cast<WaveShortMessage*>(prophetPkt->getEncapsulatedPacket());
			WaveShortMessage *wsm = tmp->dup();
			storeBundle(wsm);
			if ((tmp->getRecipientAddress()==LAddress::L3BROADCAST)||(tmp->getRecipientAddress()==myNetwAddr)){
				sendUp(prophetPkt);
			}
		}
			break;
		default:
			assert(false);
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
		case Bundle:
		{
			std::list<Prophet_Struct::bndl_meta> bundleToSendMeta;
			bundleToSendMeta = prophetPkt->getBndlmeta();
			for (std::list<Prophet_Struct::bndl_meta>::iterator it = bundleToSendMeta.begin(); it !=bundleToSendMeta.end(); ++it) {
				bundlesIndexIterator it2 = bundlesIndex.find(it->recipientAddress);
				if (it2!=bundlesIndex.end()){
					innerIndexIterator it3 = it2->second.find(it->serial);
					if (it3!=it2->second.end()){
						Prophet *bundlePkt = new Prophet();
						bundlePkt = prepareProphet(Bundle,myNetwAddr,prophetPkt->getSrcAddr(),NULL,NULL,(it3->second)->dup());
						bundlePkt->setBitLength(headerLength);
//						bundlePkt->setKind(Bundle);
//						bundlePkt->setSrcAddr(myNetwAddr);
//						bundlePkt->setDestAddr(prophetPkt->getSrcAddr());
//						bundlePkt->encapsulate((it3->second)->dup());
						if (canITransmit){
							sendDown(bundlePkt);
							/*
							 * Collecting data
							 */
							numSent++;
							bundlePkt->setHopCount(bundlePkt->getHopCount()+1);
						}
					}
				}
			}
		}
			break;
		default:
			assert(false);
		break;
	}
}


Prophet *ProphetV2::prepareProphet(short  kind, LAddress::L3Type srcAddr,LAddress::L3Type destAddr, std::list<Prophet_Struct::bndl_meta> *meta, std::map<LAddress::L3Type,double> *preds, WaveShortMessage *msg)
{

//    sentVector.record(numSent);
//    sentStats.collect(numSent);

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
	std::list<WaveShortMessage*> bundleToOffer = std::list<WaveShortMessage*>();
	std::list<Prophet_Struct::bndl_meta> bundleToOfferMeta = std::list<Prophet_Struct::bndl_meta>();

	// step 1 : check if we have any bundle that are addressed to @encouterdNode

	bundlesIndexIterator it = bundlesIndex.find(encounterdNode);
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2;
		for (it2 = innerMap.begin(); it2 !=innerMap.end(); ++it2){
			bundleToOffer.push_back(it2->second);
			Prophet_Struct::bndl_meta meta;
			meta.senderAddress = it2->second->getSenderAddress();
			meta.recipientAddress = it2->second->getRecipientAddress();
			meta.serial = it2->second->getSerial();
			meta.timestamp = it2->second->getTimestamp();
			meta.bFlags = Prophet_Enum::Bndl_Accepted;
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
					bundleToOffer.push_back(it3->second);
					Prophet_Struct::bndl_meta meta;
					meta.senderAddress = it3->second->getSenderAddress();
					meta.recipientAddress = it3->second->getRecipientAddress();
					meta.serial = it3->second->getSerial();
					meta.timestamp = it3->second->getTimestamp();
					meta.bFlags = Prophet_Enum::Bndl_Accepted;
					bundleToOfferMeta.push_back(meta);
				}
			}
	}

	// step 3 : sending the ProphetPckt

	Prophet *offerPkt = new Prophet();
	offerPkt->setBitLength(headerLength);
	offerPkt = prepareProphet(Bundle_Offer,myNetwAddr,encounterdNode,&bundleToOfferMeta);
//	offerPkt->setKind(Bundle_Offer);
//	offerPkt->setSrcAddr(myNetwAddr);
//	offerPkt->setDestAddr(encounterdNode);
//	offerPkt->setBndlmeta(bundleToOfferMeta);
	sendDown(offerPkt);
	/*
	 * Collecting data
	 */
	numSent++;
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

bool ProphetV2::exist(Prophet_Struct::bndl_meta bndlMeta)
{
	bool found = false;
	bundlesIndexIterator it = bundlesIndex.find(bndlMeta.recipientAddress);
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(bndlMeta.serial);
		if (it2 !=innerMap.end()){
			found = true;
		}
	}
	return found;
}

void ProphetV2::finish()
{
	EV << "Sent:     " << numSent << endl;
	EV << "Received: " << numReceived << endl;
	EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
	EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
	EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
	EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

	recordScalar("#sent", numSent);
	recordScalar("#received", numReceived);

	recordScalar("# of Contacts", nbrContacts);
	recordScalar("# of intercontacts", nbrRecontacts);

	hopCountStats.recordAs("hop count");
//    // This function is called by OMNeT++ at the end of the simulation.
//    EV << "Sent:     " << numSent << endl;
//    EV << "Received: " << numReceived << endl;
////    EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
////    EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
////    EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
////    EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;
//
//
//	recordScalar("#total sent", numSent);
//    recordScalar("#total received", numReceived);
//
////    EV << "Number of Prophet message sent, total:    " << sentStats.getCount() << endl;
////    sentStats.recordAs("Total sent");
////
////    EV << "Number of Prophet message received, total:    " << receivedStats.getCount() << endl;
////	receivedStats.recordAs("Total received");
////    hopCountStats.recordAs("hop count");
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

const LAddress::L3Type ProphetV2::getMyNetwAddress()
{
	return myNetwAddr;
}

void ProphetV2::recordPredsStats()
{
	// nbPreds = preds.size()-1, because P(X,X) is counted as a prediction for every node X
	int nbPreds = this->preds.size()-1;
	nbrPredsVector.record(nbPreds);

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

ProphetV2::~ProphetV2()
{

}
