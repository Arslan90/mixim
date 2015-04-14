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
	discardUnfinished = false;
	nbrContacts = 0;
	nbrRepeated = 0;
	nbrToDiscard = 0;
	durationStats.setName("contacts duration stats");
}

ClassifiedContactStats::ClassifiedContactStats(string name, bool discardUnfinished) {
	// TODO Auto-generated constructor stub
	ContactStats();
	this->name = name;
	string tmp = name + "contacts duration stats";
	this->discardUnfinished = discardUnfinished;
	nbrContacts = 0;
	nbrRepeated = 0;
	nbrToDiscard = 0;
	durationStats.setName(tmp.c_str());
}

ClassifiedContactStats::ClassifiedContactStats(string name, SimpleContactStats firstContact) {
	// TODO Auto-generated constructor stub
	ContactStats();
	this->name = name;
	string tmp = name + "contacts duration stats";
	discardUnfinished = false;
	nbrContacts = 0;
	nbrRepeated = 0;
	nbrToDiscard = 0;
	durationStats.setName(tmp.c_str());
	update(firstContact);
}

void ClassifiedContactStats::update(SimpleContactStats newContact)
{
	bool haveToUpdate = false;
	double duration;

	if (discardUnfinished){
		if (newContact.isFinished()){
			haveToUpdate = true;
			duration = newContact.getDuration();
		}
	}else{
		haveToUpdate = true;
		if (newContact.isFinished()){
			duration = newContact.getDuration();
		}else{
			nbrToDiscard++;
			if (newContact.getStartTime()==std::numeric_limits<double>::max()){
				duration = 0;
			}else {
				duration = simTime().dbl() - newContact.getStartTime();
			}
		}
	}

	if (haveToUpdate){
		this->nbrContacts++;
		if (newContact.isRepeatedContact()){
			this->nbrRepeated++;
		}

		this->L3Sent+=newContact.getL3Sent();
		this->L3Received+=newContact.getL3Received();
		this->ackSent+=newContact.getAckSent();
		this->ackReceived+=newContact.getAckReceived();
		this->bundleSent+=newContact.getBundleSent();
		this->bundleReceived+=newContact.getBundleReceived();
		this->predictionsSent+=newContact.getPredictionsSent();
		this->predictionsReceived+=newContact.getPredictionsReceived();

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

void ClassifiedContactStats::setNbrToDiscard(int nbrToDiscard)
{
    this->nbrToDiscard = nbrToDiscard;
}

ClassifiedContactStats::~ClassifiedContactStats() {
	// TODO Auto-generated destructor stub
}

