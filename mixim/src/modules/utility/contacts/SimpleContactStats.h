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

#ifndef SIMPLECONTACTSTATS_H_
#define SIMPLECONTACTSTATS_H_

#include "ContactStats.h"
//#include "Prophetv2Kinds.h"


class SimpleContactStats: public ContactStats {
private:
	double startTime;
	double endTime;
	double duration;
	bool successfulContact;
	bool repeatedContact;
	int state;
public:
	SimpleContactStats();
	SimpleContactStats(double startingTime);
	SimpleContactStats(double startingTime, int startingState);
	SimpleContactStats(double startingTime, bool repeatedContact);
	SimpleContactStats(double startingTime, bool repeatedContact, int startingState);
	virtual ~SimpleContactStats();

	double getEndTime() const
    {
        return endTime;
    }

    void setEndTime(double endTime)
    {
        this->endTime = endTime;
    }
    bool isRepeatedContact() const
    {
        return repeatedContact;
    }

    void setRepeatedContact(bool repeatedContact)
    {
        this->repeatedContact = repeatedContact;
    }

	double getDuration() const
    {
        return duration;
    }

    double getStartTime() const
    {
        return startTime;
    }

    int getState() const
    {
        return state;
    }

    bool isSuccessfulContact() const
    {
        return successfulContact;
    }

    void setDuration(double duration)
    {
        this->duration = duration;
    }

    void setStartTime(double startTime)
    {
        this->startTime = startTime;
    }

    void setState(int state)
    {
        this->state = state;
    }

    void setSuccessfulContact(bool successfulContact)
    {
        this->successfulContact = successfulContact;
    }

};

#endif /* SIMPLECONTACTSTATS_H_ */
