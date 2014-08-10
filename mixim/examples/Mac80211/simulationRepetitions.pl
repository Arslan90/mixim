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
$nodes=$ARGV[0];

#$nodes=3;
$SimTime=300; #seconds
$numberSimulation= 20; #the number of simulations
$burst= 1; #ce rafale c'est pour montrer le souci d'absence de backoff car DIFS < temps medium libre. 


################## SOME STATS.
$nodeTX= $SimTime/5; #Every 5 seconds every node transmit along the total simulation.
$nodeRX= $nodeTX*($nodes-1)* $burst; # node expected reception
$globalTX= $nodeTX*$nodes* $burst; # Global expected Transmission of all nodes
$globalRX= $nodeTX*($nodes-1)*$nodes* $burst; # Global expected Reception of all nodes

#Ok first open the files.

for ($j=1; $j<=$numberSimulation; $j++) {
$file= "./results/General-".$j.".elog";
print "CHECKING FILE: $file \n";

	&header(); #GENERAL DATA

	$Trx=0; $realRx=0; $Tri=0; $Trx=0;
	for ($i=0; $i<$nodes; $i++) {
	@data= `/bin/grep ',me RX,$i,' $file | /usr/bin/wc -l`;
	chop $data[0];
	$realRx= $data[0];
	$Tri= ($nodeRX - $realRx)/$nodeRX; #Taux non reception par noeud
	print "NODE $i, RX= $realRx, Tri= $Tri \n"; 
	$Trx += $Tri; #Sumatoire de tous les noeuds
	}#FOR  

$Trxx= $Trx/$nodes; #Taux non reception pour tous les noeuds
print "All nodes taux non reception= $Trx, moyenne= $Trxx \n\n"; 
$TrxFinal += $Trxx; 

}#FOR files counting

$TrxFinal= $TrxFinal/$numberSimulation; #fait un moyenne des resultats de tous les simulations
print "Final taux NON reception= $TrxFinal \n\n"; 

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
