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

//		preds = std::map<LAddress::L3Type,double>();



//		lastEncouterTime = std::map<LAddress::L3Type,double>();

		int tmp = par("storageSize");
		if (tmp>=0){
			bundlesStructureSize = tmp;
		}else {
			opp_error("Size of the structure that store bundles can not be negative");
		}
		bundles = std::list<WaveShortMessage*>();
		noInsert = 0;

		/*
		 * Collecting data & metrics
		 */
	    nbrL3Sent = 0;
	    nbrL3Received = 0;

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
		int size = preds.size();
		if (!( size < 80)){
			EV << "Anormal size : " << it->first << endl;
		}

		/*
		 * if iterator is equal map.end(), it means that there is no entry for BAdress in preds
		 * so lastEncTime equal 0
		 */
		predsForB = 0;
		lastEncTime = 0;
		PEnc = PEncMax;

	}else {
		if (!(it->first < 80)){
			EV << "Anormal Key : " << it->first << endl;
		}

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

		if (!(it->first < 80)){
			EV << "Anormal Key : " << it->first << endl;
		}

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

		preds[CAdress] = predsForC;
//		preds.insert(std::pair<LAddress::L3Type,double>(CAdress,predsForC));
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
		int size = preds.size();
		EV << " Actual size of map : " << size << endl;
		for (predsIterator it=preds.begin();it!=preds.end();it++){
			if (!(it->first < 80)){
				EV << "Anormal Key : " << it->first << endl;
			}
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
					hopcount = prophetPkt->getHopCount();
					hopCountVector.record(hopcount);
					hopCountStats.collect(hopcount);


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
					hopcount = prophetPkt->getHopCount();
					hopCountVector.record(hopcount);
					hopCountStats.collect(hopcount);

					executeInitiatorRole(Bundle_Offer,prophetPkt);
					executeInitiatorRole(Bundle_Response,prophetPkt);
				}
				break;
			case Bundle_Response:
				{
					// collecting data
					updatingL3Received();
					hopcount = prophetPkt->getHopCount();
					hopCountVector.record(hopcount);
					hopCountStats.collect(hopcount);

					executeListenerRole(Bundle_Response,prophetPkt);
					executeListenerRole(Bundle,prophetPkt);
				}
				break;
			case Bundle:
				{
					// collecting data
					updatingL3Received();
					hopcount = prophetPkt->getHopCount();
					hopCountVector.record(hopcount);
					hopCountStats.collect(hopcount);

					executeInitiatorRole(Bundle,prophetPkt);
				}
				break;
			default:
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


			/*
			 * Collecting data
			 */
			updatingContactState(prophetPkt->getSrcAddr(),Bundle_Offer);
			break;
		case Bundle_Response:
		{
			std::list<Prophet_Struct::bndl_meta> bundleToAcceptMeta;
			bundleToAcceptMeta = prophetPkt->getBndlmeta();
			
			/*
			 * step 1 : check if offered bundles are already stored in this node,
			 * in that case delete them from the offered list 
			 */ 
			for (std::list<Prophet_Struct::bndl_meta>::iterator it2 = bundleToAcceptMeta.begin(); it2 !=bundleToAcceptMeta.end(); ++it2) {
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
		case Bundle:
		{
			WaveShortMessage *tmp = check_and_cast<WaveShortMessage*>(prophetPkt->getEncapsulatedPacket());
			WaveShortMessage *wsm = tmp->dup();
			storeBundle(wsm);
			int addr = prophetPkt->getSrcAddr();
			if ((tmp->getRecipientAddress()==LAddress::L3BROADCAST)||(tmp->getRecipientAddress()==myNetwAddr)){
				sendUp(prophetPkt);
			}

			/*
			 * Collecting data
			 */
			updatingContactState(prophetPkt->getSrcAddr(),Bundle);
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
						if (canITransmit){
							sendDown(bundlePkt);
							/*
							 * Collecting data
							 */
							updatingL3Sent();
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
	EV << "Sent:     " << nbrL3Sent << endl;
	EV << "Received: " << nbrL3Received << endl;
	EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
	EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
	EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
	EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

	recordScalar("# L3sent", nbrL3Sent);
	recordScalar("# L3received", nbrL3Received);

	recordScalar("# of Contacts", nbrContacts);
	recordScalar("# of InterContacts", nbrRecontacts);

	recordScalar("# failed contacts before RIB", nbrFailedContactBeforeRIB);
	recordScalar("# failed contacts at RIB", nbrFailedContactAtRIB);
	recordScalar("# failed contacts at Bundle_Offer", nbrFailedContactAtBundle_Offer);
	recordScalar("# failed contacts at Bundle_Response", nbrFailedContactAtBundle_Response);
	recordScalar("# successful contacts", nbrSuccessfulContact);


//	recordScalar("# RIB Initiator Role", RIBInitRole);
//	recordScalar("# Bundle_Offer Initiator Role", Bundle_OfferInitRole);
//	recordScalar("# Bundle_Response Initiator Role", Bundle_ResponseInitRole);
//	recordScalar("# Bundle Initiator Role", BundleInitRole);
//
//	recordScalar("# RIB Listener Role", RIBListRole);
//  recordScalar("# Bundle_Offer Listener Role", Bundle_OfferListRole);
//  recordScalar("# Bundle_Response Listener Role", Bundle_ResponseListRole);
//	recordScalar("# Bundle Listener Role", BundleListRole);


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
