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

#include "ClassifiedContactStats.h"

ClassifiedContactStats::ClassifiedContactStats() {
	// TODO Auto-generated constructor stub
	ContactStats();
    init();
}

ClassifiedContactStats::ClassifiedContactStats(string name, bool discardUnfinished) {
	// TODO Auto-generated constructor stub
	ContactStats();
	init();
	this->name = name;
	string tmp = name + "contacts duration stats";
	this->discardUnfinished = discardUnfinished;
	durationStats.setName(tmp.c_str());
}

ClassifiedContactStats::ClassifiedContactStats(string name, bool discardUnfinished, bool withCDF) {
	// TODO Auto-generated constructor stub
	ContactStats();
	init();
	this->name = name;
	string tmp = name + "contacts duration stats";
	this->discardUnfinished = discardUnfinished;
	durationStats.setName(tmp.c_str());
	this->withCDF = withCDF;
}

void ClassifiedContactStats::update(SimpleContactStats* newContact)
{
	bool haveToUpdate = false;
	double duration;

	if(newContact->isHasForcedEnding()){
		this->nbrToDiscard++;
		if (!discardUnfinished){
			haveToUpdate = true;
		}
	}else {
		haveToUpdate = true;
	}

	if (haveToUpdate){
		this->nbrContacts++;
		if (newContact->isRepeatedContact()){
			this->nbrRepeated++;
		}

		duration = newContact->getDuration();
		categorizeContactDuration(duration);

		this->L3Sent+=newContact->getL3Sent();
		this->L3Received+=newContact->getL3Received();
		this->ackSent+=newContact->getAckSent();
		this->ackReceived+=newContact->getAckReceived();
		this->bundleSent+=newContact->getBundleSent();
		this->bundleReceived+=newContact->getBundleReceived();
		this->predictionsSent+=newContact->getPredictionsSent();
		this->predictionsReceived+=newContact->getPredictionsReceived();

		this->durationStats.collect(duration);
	}
}

void ClassifiedContactStats::finish()
{
//	recordScalar("adaz",  this->L3Sent);
	this->durationStats.recordAs(this->name.c_str());
}

int ClassifiedContactStats::getNbrToDiscard() const
{
    return nbrToDiscard;
}

void ClassifiedContactStats::categorizeContactDuration(double contactDuration)
{
	if (contactDuration < 0 ){
		opp_error("Contact duration is lesser than 0(ClassifiedContactStats::categorizeContactDuration)");
	}

	if (contactDuration <= 5 ){
		nbrLQ5++;
	}else if ((contactDuration > 5)&&(contactDuration <= 20)){
		nbrG5LQ20++;
	}else if ((contactDuration > 20)&&(contactDuration <= 50)){
		nbrG20LQ50++;
	}else if ((contactDuration > 50)&&(contactDuration <= 100)){
		nbrG50LQ100++;
	}else if ((contactDuration > 100)&&(contactDuration <= 500)){
		nbrG100LQ500++;
	}else if ((contactDuration > 500)&&(contactDuration <= 1800)){
		nbrG500LQ1800++;
	}else if (contactDuration > 1800){
		nbrG1800++;
	}
}

void ClassifiedContactStats::init()
{
	discardUnfinished = false;
	withCDF = false;
	nbrContacts = 0;
	nbrRepeated = 0;
	nbrToDiscard = 0;
	durationStats.setName("contacts duration stats");

	nbrLQ5 = 0;
	nbrG5LQ20 = 0;
	nbrG20LQ50 = 0;
	nbrG50LQ100 = 0;
	nbrG100LQ500 = 0;
	nbrG500LQ1800 = 0;
	nbrG1800 = 0;
}

void ClassifiedContactStats::setNbrToDiscard(int nbrToDiscard)
{
    this->nbrToDiscard = nbrToDiscard;
}

ClassifiedContactStats::~ClassifiedContactStats() {
	// TODO Auto-generated destructor stub
}

