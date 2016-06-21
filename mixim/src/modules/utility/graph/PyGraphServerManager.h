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

#ifndef __MIXIM_PYGRAPHSERVERMANAGER_H_
#define __MIXIM_PYGRAPHSERVERMANAGER_H_

#include <omnetpp.h>
#include "stdlib.h"
#include "iostream"

/**
 * TODO - Generated class
 */
class PyGraphServerManager : public cSimpleModule
{
  protected:
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    bool debug;
    std::string host;
    int port;
    double communication_range;
    bool autoBuild;
    std::string edgeFile;
    std::string nodeFile;
    std::string vpaFile;
    int margin;
    int waitingTime;
};

#endif
