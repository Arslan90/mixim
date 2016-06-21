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

#include "PyGraphServerManager.h"
#include <unistd.h>
#include "stdio.h"

Define_Module(PyGraphServerManager);

void PyGraphServerManager::initialize(int stage)
{
    // TODO - Generated method body
	if (stage ==0){
		debug = par("debug").boolValue();
		host = par("host").stdstringValue();
		port = par("port");
		communication_range = par("range").doubleValue();
		autoBuild = par("autoBuild").boolValue();
		edgeFile = par("edgeFile").stdstringValue();
		nodeFile = par("nodeFile").stdstringValue();
		vpaFile = par("vpaFile").stdstringValue();
		margin = par("margin");
		waitingTime = par("waitingTime");


//		std::string command = "pkill -f ProdServer.py";
//		system(command.c_str());

//		char cwd[1024];
//		if (getcwd(cwd, sizeof(cwd)) != NULL){
//			std::cout <<"Current working dir:"<< cwd << std::endl;
//		} else {
//			opp_error("getCWD() command failed");
//		}
//
//		command = "python "+std::string(cwd)+"/ProdServer.py";
//		popen(command.c_str(), "r");
//		sleep(waitingTime);

	}
}

void PyGraphServerManager::handleMessage(cMessage *msg)
{
    // TODO - Generated method body

}

void PyGraphServerManager::finish(){
//	std::string command = "pkill -f ProdServer.py";
//	system(command.c_str());
}
