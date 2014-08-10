#!/usr/bin/perl
#############
#  Stats for Globecom
#  
#############

$file= "../results/accident-0.elog";
$file2= "disseminationLogs.txt";
$file3="vehiclesActiveLogs.txt";
$file4="vehiclesCrossingJunctionsLogs.txt";


#&TX(); #CDF of tx per junction
#&RX(); #CDF of tx per junction
&vehCrossedJunction(); #CDF of  vehicles that have crossed a junction

#&disseminationCount(); #CDF of tx per junction
#&runningVeh(); #CDF of tx per junction
##########################"

sub runningVeh() {
#Sample
#- Flag,vehicles active,16.5,24,
#NOTA IMPORTANTE DESHACERSE DE LAS FRACCIONES YA QUE NO ENCAJAN 
#CUANDO LOS USO EN EL ARRAY

@data= `/bin/cat $file3 `;
#print @data;

foreach $item (@data) {
@result=split(/,/,$item);
if ( $result[2] =~ m/\./) { next; } # No incluyas los numeros con decimales.
print "$result[2] $result[3]\n";   
}
}


sub disseminationCount() { 
#Sample:
#- Flag, dissemination ON,246,S3.8
#NOTA IMPORTANTE DESHACERSE DE LAS FRACCIONES YA QUE NO ENCAJAN 
#CUANDO LOS USO EN EL ARRAY

@data= `/bin/cat $file2`;
#print @data;

foreach $item (@data) {
@result=split(/,/,$item);
if ( $result[2] =~ m/\./) { next; } # No incluyas los numeros con decimales.
#print $item; 
#print "$result[2] \n";   
@final[$result[2]]++;
}

$i=0;
foreach $item (@final) {
$item = $item *1; #just print zeros instead of spaces
print "$i ";
print "$item \n";
$i++;
}
} 


sub vehCrossedJunction() {
@data= `/bin/cat $file4 `;
#print @data;

#Sample
#- Flag carInJunctions: S6.99,junctionId:,57,

@final[441]=0;
foreach $item (@data) {
@result=split(/,/,$item);
#print $item; 
#print "$result[2] \n";   
@final[$result[2]]++;
}

$i=0;
foreach $item (@final) {
$item = $item *1; #just print zeros instead of spaces
#print "$i ";
print "$item \n";
$i++;
}
}



sub RX() {
@data= `/bin/grep 'stats' $file | /bin/grep rx`;
#print @data;

#Sample
#- stats, from VEH:92,TO me VEH: 98,S3.16,198.703478123734, rx,299,

@final[441]=0;
foreach $item (@data) {
@result=split(/,/,$item);
#print $item;
#print "$result[6] \n";   
@final[$result[6]]++;
}

$i=0;
foreach $item (@final) {
$item = $item *1; #just print zeros instead of spaces
#print "$i ";
print "$item \n";
$i++;
}
}



sub TX() {
@data= `/bin/grep 'stats, VEH' $file | /bin/grep tx`;
#print @data;

#Sample
#- stats, VEH, 999.016636380029,From: S2.154,989, tx,191,messageSequence: 166, messageSequenceVPA: 2

@final[441]=0;
foreach $item (@data) {
@result=split(/,/,$item);
#print $item;
#print "$result[6] \n";   
@final[$result[6]]++;
}

$i=0;
foreach $item (@final) {
$item = $item *1; #just print zeros instead of spaces
print "$item \n";
$i++;
}
}
