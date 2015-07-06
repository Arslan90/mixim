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

	/*
	 * # of contacts of a duration lesser or equal(LQ) or greater (G) to X seconds (5s,20s,50s,100s,500s,1800s)
	 */
	int nbrLQ5;
	int nbrG5LQ20;
	int nbrG20LQ50;
	int nbrG50LQ100;
	int nbrG100LQ500;
	int nbrG500LQ1800;
	int nbrG1800;

public:
	ClassifiedContactStats();
	ClassifiedContactStats(string name, bool discardUnfinished);
	ClassifiedContactStats(string name, SimpleContactStats firstContact);
	void update(SimpleContactStats newContact);
	/*
	 * Categorize the contact duration of a simple contact in order to estimate the CDF and the PDF
	 * NOTE: automatically discard the contact with a incoherent contact duration,
	 * the overall calcul must be done based on total contact minus discarded contact
	 */
	void categorizeContactDuration(double contactDuration);
	void finish();
	virtual ~ClassifiedContactStats();
    int getNbrToDiscard() const;
    void setNbrToDiscard(int nbrToDiscard);
    int getNbrG100Lq500() const
    {
        return nbrG100LQ500;
    }

    int getNbrG1800() const
    {
        return nbrG1800;
    }

    int getNbrG20Lq50() const
    {
        return nbrG20LQ50;
    }

    int getNbrG500Lq1800() const
    {
        return nbrG500LQ1800;
    }

    int getNbrG50Lq100() const
    {
        return nbrG50LQ100;
    }

    int getNbrG5Lq20() const
    {
        return nbrG5LQ20;
    }

    int getNbrLq5() const
    {
        return nbrLQ5;
    }

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
