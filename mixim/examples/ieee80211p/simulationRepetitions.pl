#!/usr/bin/perl
###############################################
# Ce program c'est pour trouver le taux de non reception des messages 
# Il faut obtenir cette information de 20 simulations avec un valeur SEED different.
#
# 1. Alors en premier lieu je doit chercher chaque fichier pour trouver le taux  de
# non-reception par noeud. 
# 2. Puis calculer taux de non-reception de tous les noeuds.
# 3. Et finalement obtenir la moyenne entre les 20 fichiers.
###############################################

#Data That I need to know:
$ARGC=@ARGV;
if ($ARGC != 1) { die "Usage: number of nodes\n"; }
$nodes=$ARGV[0]; # get the number of nodes from the command line

$SimTime=600; #seconds
$intervalTX= 1; #10hz Node TX. this is the T time.
$numberSimulation= 1; #20; #the number of simulations

##################  GUTs of the stats..
$nodeTX= $SimTime/$intervalTX; #Get Every node number transmitions of total simulation.
$nodeRX= $nodeTX*($nodes-1); # node expected reception
$globalTX= $nodeTX*$nodes; # Global expected Transmission of all nodes
$globalRX= $nodeTX*($nodes-1)*$nodes; # Global expected Reception of all nodes

#Ok first open the files.

for ($j=1; $j<=$numberSimulation; $j++) {
#$file= "./results/General-".$j.".elog";
$file= "./results/logFinal.log";
print "CHECKING FILE: $file \n";

	&header(); #GENERAL DATA

	$Trx=0; $realRx=0; $Tri=0; $Trx=0;
	for ($i=0; $i<$nodes; $i++) {
	@data= `/bin/grep ',me RX,$i,' $file | /usr/bin/wc -l`;
	chop $data[0];
	$realRx= $data[0];
	$Tri= ($nodeRX - $realRx)/$nodeRX; #Taux non reception par noeud
	$tauxRX= 1-$Tri; #Taux reception par noeud
	print "NODE $i, RealRX= $realRx, Taux non-RX= $Tri, Taux-RX= $tauxRX\n"; 
	$Trx += $tauxRX; #Sumatoire de tous les noeuds RECEPTION
	}#FOR  

$Trxx= $Trx/$nodes; #Taux-RX pour tous les noeuds
push (@deviationStandar,$Trxx); #remember all the nodes averageTx.
#print "All nodes taux-Rx= $Trx, moyenne= $Trxx \n\n"; 
print "All nodes moyenne= $Trxx \n\n"; 
$TrxFinal += $Trxx; 

}#FOR files counting

#fait un moyenne des resultats de tous les simulations
$TrxFinal= $TrxFinal/$numberSimulation; 

#And Calculate the Standard deviation
foreach $item (@deviationStandar) { # first substract each point difference.
	$difference+= ($item - $TrxFinal)**2;
	#print "HERE: $item - $TrxFinal =  $difference \n";
	}
$SD= sqrt( $difference /@deviationStandar."");#Calculate Standard Deviation.
$confidenceInterval= $SD/sqrt( @deviationStandar.""); #standarDeviation/sqrt(of measures)

print "Final taux-Rx= $TrxFinal, SD= $SD, Cinterval=$confidenceInterval \n\n"; 


############# SUBROUTINES  ##########
sub header {
#GENERAL DATA
print<<HEAD

Mi number of nodes are: $nodes
The node expected TX are: $nodeTX
The node expected RX are: $nodeRX
The global expected TX are: $globalTX
The global expected RX are: $globalRX

HEAD
}
