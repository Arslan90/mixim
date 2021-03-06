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

#include "ContactStats.h"

ContactStats::ContactStats() {
	// TODO Auto-generated constructor stub
	L3Sent = 0;
	L3Received = 0;
	ackSent= 0;
	ackReceived= 0;
	bundleSent= 0;
	bundleReceived= 0;
	predictionsSent= 0;
	predictionsReceived= 0;
	offerSent= 0;
	offerReceived= 0;
	acceptSent= 0;
	acceptReceived= 0;
	nbrAlreadyAcked= 0;
}

ContactStats::~ContactStats() {
	// TODO Auto-generated destructor stub
}

