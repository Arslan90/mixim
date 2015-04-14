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

#ifndef CLASSIFIEDCONTACTSTATS_H_
#define CLASSIFIEDCONTACTSTATS_H_

#include "SimpleContactStats.h"
//#include "ContactStats.h"
#include "chistogram.h"
#include "string"
#include "csimulation.h"

using namespace std;

class ClassifiedContactStats: public ContactStats {
private:
	bool discardUnfinished;
	string name;
	int nbrContacts;
	int nbrRepeated;
	int nbrToDiscard;
	cDoubleHistogram durationStats;
public:
	ClassifiedContactStats();
	ClassifiedContactStats(string name, bool discardUnfinished);
	ClassifiedContactStats(string name, SimpleContactStats firstContact);
	void update(SimpleContactStats newContact);
	void finish();
	virtual ~ClassifiedContactStats();
    int getNbrToDiscard() const;
    void setNbrToDiscard(int nbrToDiscard);
    int getNbrRepeated() const
    {
        return nbrRepeated;
    }

    void setNbrRepeated(int nbrRepeated)
    {
        this->nbrRepeated = nbrRepeated;
    }

	bool isDiscardUnfinished() const
    {
        return discardUnfinished;
    }

    void setDiscardUnfinished(bool discardUnfinished)
    {
        this->discardUnfinished = discardUnfinished;
    }

	cDoubleHistogram getDurationStats() const
    {
        return durationStats;
    }

    string getName() const
    {
        return name;
    }

    int getNbrContacts() const
    {
        return nbrContacts;
    }

    void setNbrContacts(int nbrContacts)
    {
        this->nbrContacts = nbrContacts;
    }
};

#endif /* CLASSIFIEDCONTACTSTATS_H_ */
