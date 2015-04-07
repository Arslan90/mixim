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
	durationStats.setName("contacts duration stats");
}

ClassifiedContactStats::ClassifiedContactStats(string name) {
	// TODO Auto-generated constructor stub
	ContactStats();
	this->name = name;
	string tmp = name + "contacts duration stats";
	durationStats.setName(tmp.c_str());
}

ClassifiedContactStats::ClassifiedContactStats(string name, SimpleContactStats firstContact) {
	// TODO Auto-generated constructor stub
	ContactStats();
	this->name = name;
	string tmp = name + "contacts duration stats";
	durationStats.setName(tmp.c_str());
	update(firstContact);
}

void ClassifiedContactStats::update(SimpleContactStats newContact)
{
	this->nbrContacts++;
	this->L3Sent+=newContact.getL3Sent();
	this->L3Received+=newContact.getL3Received();
	this->ackSent+=newContact.getAckSent();
	this->ackReceived+=newContact.getAckReceived();
	this->bundleSent+=newContact.getBundleSent();
	this->bundleReceived+=newContact.getAckReceived();
	this->predictionsSent+=newContact.getPredictionsSent();
	this->predictionsReceived+=newContact.getPredictionsReceived();

	this->durationStats.collect(newContact.getDuration());
}

ClassifiedContactStats::~ClassifiedContactStats() {
	// TODO Auto-generated destructor stub
}

