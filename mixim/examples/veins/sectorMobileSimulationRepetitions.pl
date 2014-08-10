#!/usr/bin/perl
###############################################
#OK, the idea here is to retrieve the real number of Transmissions
# That sent a message to receivers in their coverage range.
#
# 1. I'll check all the TX logs
# 2. For Every TX grep a list of vehicles running in that time
# 3. Then check the euclidian distance to know if they are close.
# 4. all the VehId in range create an array: @vehTX[vehId]++
#    to sume up the vehicles supposed to receive the transmission.
#
# IMPORTANT NOTE: WELL AFTER 24hrs I think this script do not work well
# I've to find anoter indicator of RX/TX ratio per vehicle.
###############################################

#Data That I need to know:
#$ARGC=@ARGV;
#if ($ARGC != 1) { die "Usage: number of logfile\n"; }
#$logFile=$ARGV[0]; # get the number of nodes from the command line

##################  GUTs of the stats..

#Ok first grep all the transmissions happened in log file
#$file= "./results/General-0.elog";
#$file= "./results/General-".$logFile.".elog";
$file= "./results/logs-".$logFile.".txt";
$file= "./results/logs1.txt";

@TX= `/bin/grep ',tx,' $file`;

print " RETRIEVING the supposed TX to every node \n";
# Genera el array con el numero de TX por cada segundo.
foreach $item (@TX) {
	#print $item;
	@simTime=split(/,/,$item);
	$timeInt= int($simTime[2]);

	if ($simTime[1] eq " VPA") { 
	   if ($simTime[4]== 0) { $simTime[12]=100; $simTime[13]=100; }
	   if ($simTime[4]== 1) { $simTime[12]=300; $simTime[13]=200; }
	   if ($simTime[4]== 2) { $simTime[12]=100; $simTime[13]=300; }
	   if ($simTime[4]== 3) { $simTime[12]=300; $simTime[13]=300; }
	}
	
	#VPA/VEH, time, X, Y,
	##print "$simTime[1], $timeInt,$simTime[5], $simTime[12], $simTime[13], \n ";
			#VPA/VEH,   time,    vehId,         X,         Y
	push(@activeTX,"$simTime[1],$timeInt,$simTime[4],$simTime[12],$simTime[13]");
}#foreach



###########FOREACH MY TX, in order to retrieve the nodes in range.
foreach $item (@activeTX) {
	@simTime=split(/,/,$item);
	
	#X,Y of TX NODE
	$idSource= $simTime[0]; ## VPA or VEH
	$timeCheck= $simTime[1]; ## TimeSim
	$vehId= $simTime[2];    ##Veh Id
	$X= $simTime[3];
	$Y= $simTime[4];
		#VPA/VEH, time, vehId, X, Y
	##print "$simTime[0],$simTime[1],$simTime[2],$simTime[3],$simTime[4],\n ";
	##print "\n checking TX for $vehId in time $timeCheck in $simTime[3],$simTime[4] \n";

	@carPosition= `/bin/grep 'carPosition,$timeCheck,' $file `;

###########Foreach the list of carPosition.
	##print "***carPosition list \n";
	foreach $car (@carPosition) {
	    @rangeVeh= split(/,/,$car);
		##print $car;
		#carPosition, time, vehId, X, Y
		##print "  -$rangeVeh[1],$rangeVeh[2],$rangeVeh[3],$rangeVeh[4],$rangeVeh[5] \n ";
		
		#XY of RX NODE		
		$X1=$rangeVeh[4];
		$Y1=$rangeVeh[5];
		#Now check the Euclidian distance from TX to near nodes
		$distance= sqrt(($X-$X1)**2 + ($Y-$Y1)**2);
		##print "		+distance $distance \n";

		#Enseguida agregar un contador de las TX 
		#Que se supone debio recibir el nodo.
		
		##print "checking for $idSource:$rangeVeh[3] \n";
		if ( ($idSource eq " VEH" && $distance <= 127) ||
			($idSource eq " VPA" && $distance <= 300 )  ) {
			##print "TX from $simTime[0] \n";
			##print "  -$rangeVeh[1],$rangeVeh[2],$rangeVeh[3],$rangeVeh[4],$rangeVeh[5] \n ";
			##print "		+distance $distance \n";

################Finally count the TX vehId to the destination in range
			@supposedRX[$vehId]++;  
		 }

	}#end_foreach	

}#foreach


############Y FINALMENTE COMPARARLO CON EL VALOR DE FINISH REAL RX
$vehCounter=0;
foreach $item (@supposedRX) { 
	##print "veh $vehCounter: $supposedRX[$vehCounter] \n ";			
	$vehCounter++;
}


#Step 2, obtain the vehicles finish data.
# and calculate the transmission reception probability
# sample: logs, finish,vehBack.12,34,vehTimeIn,85,vehTimeOut,99,RX,13,
@RX= `/bin/grep 'logs, finish,' $file`;


$globalCounter=0;
foreach $item (@RX) { ##### DO EVERY VEHICLE
	@simTime=split(/,/,$item);
	print "Veh $simTime[2],in $simTime[5], out $simTime[7],rx  $simTime[9], Supposed_RX $supposedRX[$globalCounter] \n";
	
		$globalCounter++;#real number of vehicles
		#$tauxRx=  $simTime[9]/$periodTx; #finaly get the RX rate per vehicle.
		#print "$globalCounter, $simTime[2], RX, $simTime[9], periodTx= $periodTx, tauxRx, $tauxRx\n";
		#$globalTauxRx+= $tauxRx;

}#End_foreach

#$globalTauxRx= $globalTauxRx/@RX.""; #avoid this because some vehicles where not valid
#$globalTauxRx= $globalTauxRx/$globalCounter;
#print "file: $file\n";
#print "\n  TOTAL RATE RX IS= $globalTauxRx, vehicles, $globalCounter, \n\n";


################" END ################

