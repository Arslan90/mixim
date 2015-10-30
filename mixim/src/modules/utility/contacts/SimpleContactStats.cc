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

#include "SimpleContactStats.h"

void SimpleContactStats::init()
{
	this->startTime = STARTTimeInitValue;
	this->endTime = ENDTimeInitValue;
	this->serial = -1;
	this->state = -1;
	this->successfulContact = false;
	this->repeatedContact = false;
	this->hasForcedEnding = false;

}

SimpleContactStats::SimpleContactStats() {
	// TODO Auto-generated constructor stub
	ContactStats();
	init();
}

SimpleContactStats::SimpleContactStats(double startingTime)
{
	ContactStats();
	init();
	this->startTime = startingTime;
}

SimpleContactStats::SimpleContactStats(double startingTime, int startingState)
{
	ContactStats();
	init();
	this->startTime = startingTime;
	this->state = startingState;
}

SimpleContactStats::SimpleContactStats(double startingTime, bool repeatedContact)
{
	ContactStats();
	init();
	this->startTime = startingTime;
	this->repeatedContact = repeatedContact;
}

SimpleContactStats::SimpleContactStats(double startingTime, bool repeatedContact, int startingState)
{
	ContactStats();
	init();
	this->startTime = startingTime;
	this->state = startingState;
	this->repeatedContact = repeatedContact;
}

SimpleContactStats::~SimpleContactStats() {
	// TODO Auto-generated destructor stub
}

bool SimpleContactStats::operator ==(const SimpleContactStats & b) const
{
	return (this->startTime==b.getStartTime());
}

bool SimpleContactStats::operator <(const SimpleContactStats & b) const
{
	return this->startTime<b.getStartTime();
}

bool SimpleContactStats::operator >(const SimpleContactStats & b) const
{
	return this->startTime>b.getStartTime();
}

bool SimpleContactStats::operator <=(const SimpleContactStats & b) const
{
	return !(this->operator >(b));
}

bool SimpleContactStats::hasFinished()
{
	return (endTime!=ENDTimeInitValue);
}



SimpleContactStats::SimpleContactStats(int serial, double startingTime)
{
	ContactStats();
	init();
	this->serial = serial;
	this->startTime = startingTime;
}

bool SimpleContactStats::hasStarted()
{
	return (startTime!=STARTTimeInitValue);
}

double SimpleContactStats::getDuration()
{
	double duration;
	if ((hasFinished())&&(hasStarted())){
		duration = endTime - startTime;
	}else {
		opp_error("Impossible to calculate the duration of the contact(SimpleContactStats::getDuration)");
	}
	return duration;
}

bool SimpleContactStats::operator >=(const SimpleContactStats & b) const
{
	return !(this->operator <(b));
}





