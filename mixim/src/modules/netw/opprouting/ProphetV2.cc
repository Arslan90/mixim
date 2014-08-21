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

		bundlesStructureSize = par("storageSize");
		bundles = std::list<WaveShortMessage*>();
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
	double timeDiff = (simTime().dbl()-lastAgeUpdate)/secondsInTimeUnit;
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
}

//void ProphetV2::canITransmit()
//{
//	ConnectionManager *cM = NULL;
//	cModule *cc = this->getParentModule();
//	cModule *nic = NULL;
//	if (cc!=NULL){
//			nic = cc->getSubmodule("nic");
//	}
//	while (cc->getParentModule()!=NULL){
//		cc = cc->getParentModule();
//	}
//	cM = (ConnectionManager *)(cc);
//	if (nic!=NULL){
//		cM->getGateList(nic->getId());
//	}
//
//}

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
//	NetwPkt *m = static_cast<NetwPkt *>(msg);
    Mac80211Pkt *m = check_and_cast<Mac80211Pkt *>(msg);
    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;
    NetwPkt *netwpckt = check_and_cast<NetwPkt *>(m->decapsulate());
    switch (netwpckt->getKind()) {
		case HELLO:
			break;
		case ERROR:
			break;
		case RIBD:
			break;
		case RIB:
			{
				Prophet *prophetPkt = check_and_cast<Prophet *>(netwpckt);
				// first step : updating preds
				executeListenerRole(RIB,prophetPkt);
				// second step : starting of Bundle_Offer phase
				executeListenerRole(Bundle_Offer,prophetPkt);
			}
			break;
		case Bundle_Offer:
			{
				Prophet *prophetPkt = check_and_cast<Prophet *>(netwpckt);
				executeInitiatorRole(Bundle_Offer,prophetPkt);
				executeInitiatorRole(Bundle_Response,prophetPkt);
			}
			break;
		case Bundle_Response:
			{
				Prophet *prophetPkt = check_and_cast<Prophet *>(netwpckt);
				executeListenerRole(Bundle_Response,prophetPkt);
				executeListenerRole(Bundle,prophetPkt);
			}
			break;
		case Bundle:
			{
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
}

void ProphetV2::handleUpperMsg(cMessage* msg)
{
	assert(dynamic_cast<WaveShortMessage*>(msg));
	WaveShortMessage *upper_msg = dynamic_cast<WaveShortMessage*>(msg);
//	if ((mapsForBundles.count(upper_msg->getRecipientAddress())==0)||(!existingBundle(upper_msg))){
//		bundles.push_back(upper_msg);
//		mapsForBundles.insert(std::pair<LAddress::L3Type, WaveShortMessage*>(upper_msg->getRecipientAddress(),upper_msg));
//	}
	storeBundle(upper_msg);
	if (canITransmit){
		sendDown(encapsMsg(upper_msg->dup()));
	}
//    sendDown(msg);
}

void ProphetV2::handleSelfMsg(cMessage* msg)
{
//	getParentModule();
//	BaseConnectionManager::
}

void ProphetV2::handleLowerControl(cMessage* msg)
{
//	if (strcmp(msg->getName(),"connection") == 0){
//		canITransmit = true;
//		resumeConnection();
//	}
//	if (strcmp(msg->getName(),"disconnection") == 0){
//		canITransmit = true;
//	}
	switch (msg->getKind()) {
		case NEWLY_CONNECTED:
			canITransmit = true;
			break;
		case NEW_NEIGHBOR:
//			resumeConnection();
			{
				if (canITransmit){
					executeInitiatorRole(RIB,NULL);
				}
			}
			break;
		case NO_NEIGHBOR_AND_DISCONNECTED:
			canITransmit = false;
			break;
	}
}
//
//void ProphetV2::handleUpperControl(cMessage* msg)
//{
//
//}

//cObject *const ProphetV2::setDownControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr)
//{
//	return BaseNetwLayer::setDownControlInfo(pMsg, pDestAddr);
//}
//
//cObject *const ProphetV2::setUpControlInfo(cMessage *const pMsg, const LAddress::L3Type& pSrcAddr)
//{
//	return BaseNetwLayer::setUpControlInfo(pMsg, pSrcAddr);
//}

void ProphetV2::resumeConnection()
{
	if (canITransmit){
		Prophet *msg = new Prophet("RIB Prophet",RIB);
		msg->setBitLength(headerLength);
		msg->setSrcAddr(myNetwAddr);
		msg->setDestAddr(LAddress::L3BROADCAST);
		msg->setPreds(preds);
		sendDown(msg);
	}
	if (bundles.size()!=0){
		for (std::list<WaveShortMessage*>::iterator it = bundles.begin(); it != bundles.end(); ++it){
			if (canITransmit){
				WaveShortMessage *dupMessage = (*it)->dup();
				sendDown(encapsMsg(dupMessage));
			}
		}
	}
}

void ProphetV2::storeBundle(WaveShortMessage *msg)
{
		// step 1 : check if the bundle is already stored
	if (!existingBundle(msg)){

		// step 2 : add the bundle to stored bundles
		switch (qStrategy) {
//		QUEUING_FIFO=1,
//		QUEUING_MOFO,
//		QUEUING_MOPR,
//		QUEUING_LinearMOPR,
//		QUEUING_SHLI,
//		QUEUING_LEPR,
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
//	if ((bundlesIndex.count(msg->getRecipientAddress())==0)||(!existingBundle(msg))){
//			bundles.push_back(msg);
//			std::map <int, WaveShortMessage*> inner_map;
//			inner_map.insert(std::pair<int,WaveShortMessage*>(msg->getSerial(),msg));
//			bundlesIndex.insert(std::pair<LAddress::L3Type, innerIndexMap >(msg->getRecipientAddress(),inner_map));
//	}
}

void ProphetV2::executeInitiatorRole(short  kind, Prophet *prophetPkt)
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
			ribPkt->setBitLength(headerLength);
			ribPkt->setKind(RIB);
			ribPkt->setSrcAddr(myNetwAddr);
			ribPkt->setDestAddr(LAddress::L3BROADCAST);
			// creating a copy of preds
			std::map<LAddress::L3Type, double> tmp = std::map<LAddress::L3Type, double>();
			tmp.insert(preds.begin(),preds.end());
			ribPkt->setPreds(tmp);
//			sendDown(ribPkt);
//			uniform(0.001,1);
			sendDelayed(ribPkt,dblrand(),"lowerLayerOut");
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
			 * step one : check if offered bundles are already stored in this node, 
			 * in that case delete them from the offered list 
			 */ 
			for (std::list<Prophet_Struct::bndl_meta>::iterator it = bundleToAcceptMeta.begin(); it !=bundleToAcceptMeta.end(); ++it) {
				if (existingBundle(*it)){
					it = bundleToAcceptMeta.erase(it);
				}
			}
			
			/*
			 * second step : sending the response
			 */
			
			Prophet *responsePkt = new Prophet();
			responsePkt->setBitLength(headerLength);
			responsePkt->setKind(Bundle_Response);
			responsePkt->setSrcAddr(myNetwAddr);
			responsePkt->setDestAddr(prophetPkt->getSrcAddr());
			responsePkt->setBndlmeta(bundleToAcceptMeta);
			sendDown(responsePkt);
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

void ProphetV2::executeListenerRole(short  kind, Prophet *prophetPkt)
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
			definingBundleOffer(prophetPkt);
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
						bundlePkt->setBitLength(headerLength);
						bundlePkt->setKind(Bundle);
						bundlePkt->setSrcAddr(myNetwAddr);
						bundlePkt->setDestAddr(prophetPkt->getSrcAddr());
						bundlePkt->encapsulate((it3->second)->dup());
						if (canITransmit){
							sendDown(bundlePkt);
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

void ProphetV2::definingBundleOffer(Prophet *prophetPkt)
{
	LAddress::L3Type encounterdNode = prophetPkt->getSrcAddr();
	std::map<LAddress::L3Type, double> concernedPreds = std::map<LAddress::L3Type, double>();
	std::vector<std::pair<LAddress::L3Type, double>	> sortedPreds;
	std::list<WaveShortMessage*> bundleToOffer = std::list<WaveShortMessage*>();
	std::list<Prophet_Struct::bndl_meta> bundleToOfferMeta = std::list<Prophet_Struct::bndl_meta>();

	// first step : check if we have any bundle that are addressed to @encouterdNode

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

//	std::pair <bundleIndexIterator, bundleIndexIterator> ret;
//	ret = bundlesIndex.equal_range(encounterdNode);
//	for (bundleIndexIterator it=ret.first; it!=ret.second; ++it){
//		bundleToOffer.push_back(it->second);
//		Prophet_Struct::bndl_meta meta;
//		meta.senderAddress = it->second->getSenderAddress();
//		meta.recipientAddress = it->second->getRecipientAddress();
//		meta.serial = it->second->getSerial();
//		meta.timestamp = it->second->getTimestamp();
//		meta.bFlags = Prophet_Enum::Bndl_Accepted;
//		bundleToOfferMeta.push_back(meta);
//	}

	// second step : check if we have any bundle that can be offered to @encouterdNode
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
//		FWD_GRTR	=1,
//		FWD_GTMX	=2,
//		FWD_GTHR	=3,
//		FWD_GRTRplus=4,
//		FWD_GTMXplus=5,
//		FWD_GRTRsort=6,
//		FWD_GRTRmax	=7,
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
//		ret = bundlesIndex.equal_range(it->first);
//		for (bundleIndexIterator it2=ret.first; it2!=ret.second; ++it2){
//				bundleToOffer.push_back(it2->second);
//				Prophet_Struct::bndl_meta meta;
//				meta.senderAddress = it2->second->getSenderAddress();
//				meta.recipientAddress = it2->second->getRecipientAddress();
//				meta.serial = it2->second->getSerial();
//				meta.timestamp = it2->second->getTimestamp();
//				meta.bFlags = Prophet_Enum::Bndl_Accepted;
//				bundleToOfferMeta.push_back(meta);
//		}
//
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

	//third step : sending the ProphetPckt

	Prophet *offerPkt = new Prophet();
	offerPkt->setBitLength(headerLength);
	offerPkt->setKind(Bundle_Offer);
	offerPkt->setSrcAddr(myNetwAddr);
	offerPkt->setDestAddr(encounterdNode);
	offerPkt->setBndlmeta(bundleToOfferMeta);
	sendDown(offerPkt);
}



//bool ProphetV2::fwd_GRTRmax_sortingFunc(std::pair<LAddress::L3Type,double> firstPair, std::pair<LAddress::L3Type,double> secondPair)
//{
//	return firstPair.second >secondPair.second;
//}

bool ProphetV2::existingBundle(WaveShortMessage *msg)
{
	bool exist = false;
	bundlesIndexIterator it = bundlesIndex.find(msg->getRecipientAddress());
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(msg->getSerial());
		if (it2 !=innerMap.end()){
			exist = true;
		}
	}
//	std::pair<bundleIndexIterator, bundleIndexIterator> iterpair = bundlesIndex.equal_range(msg->getRecipientAddress());
//	for (bundleIndexIterator it = iterpair.first; it != iterpair.second; ++it){
//		if (it->second == msg){
//			exist = true;
//			break;
//		}
//	}
	return exist;
}

bool ProphetV2::existingBundle(Prophet_Struct::bndl_meta bndlMeta)
{
	bool exist = false;
	bundlesIndexIterator it = bundlesIndex.find(bndlMeta.recipientAddress);
	if (it != bundlesIndex.end()){
		innerIndexMap innerMap(it->second);
		innerIndexIterator it2 = innerMap.find(bndlMeta.serial);
		if (it2 !=innerMap.end()){
			exist = true;
		}
	}
	return exist;
}

void ProphetV2::finish()
{

}

ProphetV2::~ProphetV2()
{

}
