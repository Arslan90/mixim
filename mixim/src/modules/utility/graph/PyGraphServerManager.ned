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

package org.mixim.modules.utility.graph;

//
// TODO auto-generated module
//
simple PyGraphServerManager
{
    parameters:
    	@display("i=block/network2");
        bool debug = default(false);  // emit debug messages?
        string host = default("localhost");  // TraCI server hostname
        int port = default(19999);  // TraCI server port
        double range; // Communication Range for VPA, usefull to map them to neigbhorhood nodes & edges
        bool autoBuild = default(false);
        string edgeFile = default("edges.txt");
        string nodeFile = default("nodes.txt");
        string vpaFile = default("TAPAS_VPA.txt");
        int margin = default(0);  // margin to add to all received vehicle positions
        int waitingTime = default(200);
        bool collectStatOnly = default(false);
        double updateInterval = default(1.0);
        
		@statistic[deliveryRatio](title="DR";source="deliveryRatio";record=mean,last,vector);
		@statistic[totalOverhead](title="OT";source="totalOverhead";record=mean,last,vector);
		
		@statistic[uniqBndlSent](title="UniqBundleSent";source="uniqBndlSent";record=mean,last,vector);
		@statistic[uniqBndlReceived](title="UniqBndlReceived";source="uniqBndlReceived";record=mean,last,vector);
		@statistic[l3BndlSent](title="L3BundleSent";source="l3BndlSent";record=mean,last,vector);
		@statistic[l3BndlReceived](title="L3BundleReceived";source="l3BndlReceived";record=mean,last,vector);				
		
		@statistic[ratioCtrlData](title="RCD";source="ratioCtrlData";record=mean,last,vector);
		@statistic[ratioRecvSent](title="RRS";source="ratioRecvSent";record=mean,last,vector);
		
		@statistic[TotalSizeHelloCtrl](title="THC";source="TotalSizeHelloCtrl";record=mean,last,vector);
		@statistic[TotalSizeOtherCtrl](title="TOC";source="TotalSizeOtherCtrl";record=mean,last,vector);
		@statistic[TotalSizeCtrl](title="TC";source="TotalSizeCtrl";record=mean,last,vector);
		@statistic[TotalSizeData](title="TD";source="TotalSizeData";record=mean,last,vector);
		
		@statistic[THC_SB](title="THC_SB";source="t_sizeHC_SB";record=mean,last,vector);
		@statistic[THC_SA](title="THC_SA";source="t_sizeHC_SA";record=mean,last,vector);
		@statistic[THC_CL](title="THC_CL";source="t_sizeHC_CL";record=mean,last,vector);
		@statistic[THC_RCC](title="THC_RCC";source="t_sizeHC_RCC";record=mean,last,vector);
		
		@statistic[TOC_SB](title="TOC_SB";source="t_sizeOC_SB";record=mean,last,vector);
		@statistic[TOC_SA](title="TOC_SA";source="t_sizeOC_SA";record=mean,last,vector);
		@statistic[TOC_CL](title="TOC_CL";source="t_sizeOC_CL";record=mean,last,vector);
		@statistic[TOC_RCC](title="TOC_RCC";source="t_sizeOC_RCC";record=mean,last,vector);
		
		//@statistic[controlOverhead](title="OverheadControlMsg";source="t_sizeOC_RCC";record=mean,last,vector);
}
