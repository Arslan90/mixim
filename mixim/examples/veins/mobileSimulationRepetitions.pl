#!/usr/bin/perl
###############################################
# Ce program c'est pour trouver le taux de non reception des messages 
# Dans un environment mobile. c-à-d en utilisant SUMO.
# La difenrence par rapport au script simulationRepetitions.pl cela utilise de noeuds statics
#
# Il faut obtenir cette information de n simulations avec un valeur SED different.
#
# 1. Alor d'abord je dois comencer a creer la table de temps avec les tranmissions
# de chaque seconds.
# 2. Trouver chaque logs, finish et calculer la voiture son taux de reception.
###############################################

#Data That I need to know:
$ARGC=@ARGV;
if ($ARGC != 1) { die "Usage: number of logfile\n"; }
#$logFile=$ARGV[0]; # get the number of nodes from the command line
$file= $ARGV[0]; # get the number of nodes from the command line

##################  GUTs of the stats..

#$file= "./results/General-0.elog";
#$file= "./results/General-".$logFile.".elog";
#$file= "./results/logs-".$logFile.".txt";
#$file= "./results/logs1.txt";

#Ok first grep all the transmissions happened in log file
#Sample: logs, VEH,16.004919655871,From,1,S2.0,tx,A2_tail, messageSequence, 1, messageSequenceVPA, 0,202.048,408.35,

@TX= `/bin/grep ',tx,' $file`;

# Genera el array con el numero de TX generados cada segundo.
foreach $item (@TX) {
	@simTime=split(/,/,$item);
	$timeInt= int($simTime[2]);
	##print "$simTime[2],$timeInt, \n ";
	@timerTX[$timeInt]++; 
}#foreach

# Print results.
#$time=0;
#foreach $item (@timerTX) {
#	if($item){ print "$time: $item \n"; } # avoid to print empty values.
#	$time++;
#}#foreach


#Step 2, obtain the vehicles finish data.
# and calculate the transmission reception probability
# sample: logs, finish,vehBack.12,34,vehTimeIn,85,vehTimeOut,99,RX,13,
@RX= `/bin/grep 'logs, finish,' $file`;

$globalCounter=0;
foreach $item (@RX) { ##### DO EVERY VEHICLE
	@simTime=split(/,/,$item);
	##print "Veh $simTime[2],in $simTime[5], out $simTime[7],rx  $simTime[9], \n";
	
	######Now get the vehicle TX under vehicle live period of time.
	$periodTx=0;
	for($start= $simTime[5]; $start <= $simTime[7]; $start++) { 
		##  printing not empty values.
		##if($timerTX[$start]){ print "$start = $timerTX[$start] tx\n"; } 
		$periodTx+= $timerTX[$start]; #add up all the TX into the period.
	}#end_for

	##### Get vehicles and global rate Reception.
	if ($periodTx){ #avoid divisor in Zeros
		$globalCounter++;#real number of vehicles
		$tauxRx=  $simTime[9]/$periodTx; #finaly get the RX rate per vehicle.
		print "$globalCounter, $simTime[2], RX, $simTime[9], periodTx= $periodTx, tauxRx, $tauxRx\n";
		$globalTauxRx+= $tauxRx;
	}
}#End_foreach

#$globalTauxRx= $globalTauxRx/@RX.""; #avoid this because some vehicles where not valid
$globalTauxRx= $globalTauxRx/$globalCounter;
print "file: $file\n";
print "\n  TOTAL RATE RX IS= $globalTauxRx, vehicles, $globalCounter, \n\n";



######################################""
