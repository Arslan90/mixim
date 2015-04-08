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

SimpleContactStats::SimpleContactStats() {
	// TODO Auto-generated constructor stub
	ContactStats();
	this->startTime = -std::numeric_limits<double>::max();
	this->endTime = std::numeric_limits<double>::max();
	this->state = -1;
	this->successfulContact = false;
	this->repeatedContact = false;
}

SimpleContactStats::SimpleContactStats(double startingTime)
{
	ContactStats();
	this->startTime = startingTime;
	this->endTime = -1;
	this->state = -1;
	this->successfulContact = false;
	this->repeatedContact = false;
}

SimpleContactStats::SimpleContactStats(double startingTime, int startingState)
{
	ContactStats();
	this->startTime = startingTime;
	this->endTime = -1;
	this->state = startingState;
	this->successfulContact = false;
	this->repeatedContact = false;
}

SimpleContactStats::SimpleContactStats(double startingTime, bool repeatedContact)
{
	ContactStats();
	this->startTime = startingTime;
	this->endTime = -1;
	this->state = -1;
	this->successfulContact = false;
	this->repeatedContact = false;
}

SimpleContactStats::SimpleContactStats(double startingTime, bool repeatedContact, int startingState)
{
	ContactStats();
	this->startTime = startingTime;
	this->endTime = -1;
	this->state = startingState;
	this->successfulContact = false;
	this->repeatedContact = repeatedContact;
}

SimpleContactStats::~SimpleContactStats() {
	// TODO Auto-generated destructor stub
}

