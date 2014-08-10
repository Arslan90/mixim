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
		PEncMax = par("PEncMax").doubleValue();
		I_TYP = par("I_TYP").doubleValue();
		Beta = par("Beta").doubleValue();
		GAMMA = par("GAMMA").doubleValue();
		secondsInTimeUnit = par("secondsInTimeUnit");
		int fwd = par("fwdStrategy");
		fwdStrategy = static_cast<t_prophet_forward>(fwd);
		int q = par("qStrategy");
		qStrategy = static_cast<t_prophet_queuing>(q);
		/*
		 * L3Address will be initialized by BaseNetwLayer::initialize(1);
		 */
//		myNetwAddr = LAddress::L3Type( getId() );
		lastAgeUpdate = 0;
		preds = std::map<LAddress::L3Type,double>();
		preds.insert(std::pair<LAddress::L3Type,double>(myNetwAddr,1));
		lastEncouterTime = std::map<LAddress::L3Type,double>();
		bundles = std::list<WaveShortMessage*>();
		mapsForBundles = std::multimap<LAddress::L3Type,WaveShortMessage*>();
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
	map_it it= preds.find(BAdress);
	map_it it2= lastEncouterTime.find(BAdress);
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
	map_it tmp_it;
	/*
	 * before calculating transitive predictions, we must age preds.
	 */
	ageDeliveryPreds();
	for (map_it it=Bpreds.begin(); it!=Bpreds.end();it++){
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
		for (map_it it=preds.begin();it!=preds.end();it++){
			it->second = it->second * mult;
		}
		lastAgeUpdate = simTime().dbl();
	}
}

void ProphetV2::update()
{
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
			break;
		case Bundle_Offer:
			break;
		case Bundle_Response:
			break;
		case Bundle:
			break;
		default:
			WaveShortMessage *tmp = check_and_cast<WaveShortMessage*>(netwpckt->getEncapsulatedPacket());
			WaveShortMessage *wsm = tmp->dup();
			if (LAddress::isL3Broadcast(m->getDestAddr())){
				storeBundle(wsm);
			}
			sendUp(netwpckt);
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

void ProphetV2::finish()
{

}

void ProphetV2::resumeConnection()
{
	if (canITransmit){
		Prophet *msg = new Prophet("test",RIB);
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
	if ((mapsForBundles.count(msg->getRecipientAddress())==0)||(!existingBundle(msg))){
			bundles.push_back(msg);
			mapsForBundles.insert(std::pair<LAddress::L3Type, WaveShortMessage*>(msg->getRecipientAddress(),msg));
	}
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
			Prophet *msg = new Prophet();
			msg->setBitLength(headerLength);
			msg->setKind(RIB);
			msg->setSrcAddr(myNetwAddr);
			msg->setDestAddr(LAddress::L3BROADCAST);
			msg->setPreds(preds);
			sendDown(msg);
		}
			break;
		case Bundle_Offer:
			break;
		case Bundle_Response:
			break;
		case Bundle:
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
			updateDeliveryPredsFor(prophetPkt->getSrcAddr());
			std::map<LAddress::L3Type, double> tmp = std::map<LAddress::L3Type, double>();
			for (map_it it=prophetPkt->getPreds().begin(); it!=prophetPkt->getPreds().end(); it++) {
				tmp.insert(std::pair<LAddress::L3Type, double>(it->first,it->second));
			}
			updateTransitivePreds(prophetPkt->getSrcAddr(),prophetPkt->getPreds());
		}
			break;
		case Bundle_Offer:
			break;
		case Bundle_Response:
			break;
		case Bundle:
			break;
		default:
			assert(false);
		break;
	}
}

ProphetV2::~ProphetV2()
{
}




bool ProphetV2::existingBundle(WaveShortMessage *msg)
{
	bool exist = false;
	std::pair<multimap_it, multimap_it> iterpair = mapsForBundles.equal_range(msg->getRecipientAddress());
	for (multimap_it it = iterpair.first; it != iterpair.second; ++it){
		if (it->second == msg){
			exist = true;
			break;
		}
	}
	return exist;
}




