#!/usr/bin/perl
######################
# Obtain the averages of omnetpp results.
# This functions are only to obtain average results from the junctions 
# in the petite scenario. And only for detailled vision of what's going on..
#
######################

$timeStep = 5; #seconds
$file="accident-borrar.log";
#open(LEE,"$file") || die "cant open file $! \n";
#@data=<LEE>;
#print @data;


#&tx(); #Get number of Tx along time
#&rx(); #Get number of Rx along time

#&tx_space(); #Get number of Tx in space.
#&rx_space(); #Get number of Tx in space. 

#&veh_number_tx(); #vehicleId and the number of Tx. Counting to create the CDF.
#&veh_number_rx(); #vehicleId and the number of Rx. Counting to create the CDF.

#&veh_tx_discret(); #Get the discret value of vehicleId number of Tx.
&veh_rx_discret(); #Get the discret value of vehicleId number of Rx.
#&veh_tx_messageVivant(); #vehicleId and the number of Rx.

############### FUNCTIONS ################

sub veh_tx_messageVivant {
my @tx=`/bin/grep 'stats, VEH,' $file | /bin/grep tx `;
#print @tx;

#Sample
#- stats, VEH, 35.71899039409, vehBack.0, tx,(314.752,98.35,0), 4, 
#- stats, VEH, VEH, 35.802490961138, veh.0, rx,(305.26,101.65,0), 4, 

#junctionRadio
$jr= 20;
#Junctions positions
my $j1=100;
my $j2=200;
my $j3=300;
my $j4=400;
#junction 1
my $j1x= $j1 - $jr;
my $j1X= $j1 + $jr;
#junction 2
my $j2x= $j2 - $jr;
my $j2X= $j2 + $jr;
#junction 3
my $j3x= $j3 - $jr;
my $j3X= $j3 + $jr;
#junction 4
my $j4x= $j4 - $jr;
my $j4X= $j4 + $jr;

#print "#Number of Tx along time \n#time of Tx \n";
my $counter=1; my $timeline=0;
foreach $item (@tx) {
        #print  $item;
        my @txVeh = split (/\,/,$item);
        $txVeh[5] =~ s/\(//g; #Chop the first string the '('
        #print "X position: $txVeh[5]\n"; 

        ## "IN JUNCTION 1 \n"; 
        if ( $txVeh[5] >= $j1x  && $txVeh[5] <= $j1X ){
	$array= int($txVeh[2]/ 5); #Hack creamos un array de 200
	$messageVivant[$array]= 1; #Activamos una celda si hay TX.
        ##print "$txVeh[2], $txVeh[5], $array \n"; 
	}

        ## "IN JUNCTION 2
        #if ( $txVeh[5] >= $j2x  && $txVeh[5] <= $j2X ){
        #$array= int($txVeh[2]/ 5); #Hack creamos un array de 200
        #$messageVivant[$array]= 1; #Activamos una celda si hay TX.
        ##print "$txVeh[2], $txVeh[5], $array \n";
	#}

        ## "IN JUNCTION 3  
        #if ( $txVeh[5] >= $j3x  && $txVeh[5] <= $j3X ){
        #$array= int($txVeh[2]/ 5); #Hack creamos un array de 200
        #$messageVivant[$array]= 1; #Activamos una celda si hay TX.
        ##print "$txVeh[2], $txVeh[5], $array \n";
	#}

        ## "IN JUNCTION 
        #if ( $txVeh[5] >= $j4x  && $txVeh[5] <= $j4X ){
        #$array= int($txVeh[2]/ 5); #Hack creamos un array de 200
        #$messageVivant[$array]= 1; #Activamos una celda si hay TX.
        ##print "$txVeh[2], $txVeh[5], $array \n";
        #}


}#END foreach
	for($j=0; $j<200; $j++) {
	$timeLine = $j*5;
	$messageVivant[$j]= $messageVivant[$j]* 1;
	print "$timeLine $messageVivant[$j]\n";
	}

}#end sub




sub veh_rx_discret {
my @rx=`/bin/grep 'stats, VEH,' $file | /bin/grep rx `;
#print @rx;

#Sample
#- stats, VEH, 35.71899039409, vehBack.0, tx,(314.752,98.35,0), 4, 
#- stats, VEH, VEH, 35.802490961138, veh.0, rx,(305.26,101.65,0), 4, 

#junctionRadio
$jr= 20;
#Junctions positions
my $j1=100;
my $j2=200;
my $j3=300;
my $j4=400;
#junction 1
my $j1x= $j1 - $jr;
my $j1X= $j1 + $jr;
#junction 2
my $j2x= $j2 - $jr;
my $j2X= $j2 + $jr;
#junction 3
my $j3x= $j3 - $jr;
my $j3X= $j3 + $jr;
#junction 4
my $j4x= $j4 - $jr;
my $j4X= $j4 + $jr;

#print "#Number of Tx along time \n#time of Tx \n";
my $counter=1; my $timeline=0;
foreach $item (@rx) {
        #print  $item;
        my @rxVeh = split (/\,/,$item);
        $rxVeh[6] =~ s/\(//g; #Chop the first string the '('
        #print "X position: $rxVeh[6]\n"; 

        ## IN JUNCTION 1
        #if ( $rxVeh[6] >= $j1x  && $rxVeh[6] <= $j1X ){
        #print "$rxVeh[3] \n";
	# }

        ## "IN JUNCTION 2 
        #if ( $rxVeh[6] >= $j2x  && $rxVeh[6] <= $j2X ){
        #print "$rxVeh[3] \n"; }

        ##"IN JUNCTION 3 
        #if ( $rxVeh[6] >= $j3x  && $rxVeh[6] <= $j3X ){
        #print "$rxVeh[3] \n"; }

        ##"IN JUNCTION 4 
        if ( $rxVeh[6] >= $j4x  && $rxVeh[6] <= $j4X ){
        print "$rxVeh[3] \n"; }

}#END foreach
}#end sub





sub veh_tx_discret {
my @tx=`/bin/grep 'stats, VEH,' $file | /bin/grep tx `;
#print @tx;

#Sample
#- stats, VEH, 35.71899039409, vehBack.0, tx,(314.752,98.35,0), 4, 
#- stats, VEH, VEH, 35.802490961138, veh.0, rx,(305.26,101.65,0), 4, 

#junctionRadio
$jr= 20;
#Junctions positions
my $j1=100;
my $j2=200;
my $j3=300;
my $j4=400;
#junction 1
my $j1x= $j1 - $jr;
my $j1X= $j1 + $jr;
#junction 2
my $j2x= $j2 - $jr;
my $j2X= $j2 + $jr;
#junction 3
my $j3x= $j3 - $jr;
my $j3X= $j3 + $jr;
#junction 4
my $j4x= $j4 - $jr;
my $j4X= $j4 + $jr;

#print "#Number of Tx along time \n#time of Tx \n";
my $counter=1; my $timeline=0;
foreach $item (@tx) {
        #print  $item;
        my @txVeh = split (/\,/,$item);
        $txVeh[5] =~ s/\(//g; #Chop the first string the '('
	#print "X position: $txVeh[5]\n"; 

	#if ( $txVeh[5] >= $j1x  && $txVeh[5] <= $j1X ){
	##print "IN JUNCTION 1 \n"; 
	#print "$txVeh[2] \n"; }

	#if ( $txVeh[5] >= $j2x  && $txVeh[5] <= $j2X ){
	##print "IN JUNCTION 2 \n"; 
	#print "$txVeh[2] \n"; }

	#if ( $txVeh[5] >= $j3x  && $txVeh[5] <= $j3X ){
	##print "IN JUNCTION 3 \n"; 
	#print "$txVeh[2] \n"; }

	if ( $txVeh[5] >= $j4x  && $txVeh[5] <= $j4X ){
	##print "IN JUNCTION 4 \n"; 
	print "$txVeh[2] \n"; }

}#END foreach
}#end sub




sub tx {
my @tx=`/bin/grep stats $file | /bin/grep tx `;
#print @tx;

print "#Number of Tx along time \n#timeline, number of Tx \n";
my $counter=1; my $timeline=0;
foreach $item (@tx) {
	##print " $item ";
	my @txVeh = split (/\,/,$item);
	##print "$txVeh[2] \n";

	if ($txVeh[2] >= $timeline ) {
        $average= $counter/$timeStep;
        ##print " -- Seconds:$timeline, number=$counter, average=$average\n";
        print "$timeline $average\n";
	#print "$timeline $counter \n"; 
	$timeline+=$timeStep; $counter=1;
	} else { $counter++;}

}#END foreach
}#end sub


sub rx {
@rx=`/bin/grep stats $file | /bin/grep rx `;
#print @rx;

print "#Number of Rx along time \n#timeline, number of Rx \n";
my $counter=1; my $timeline=0;
foreach $item (@rx) {
        #print " $item ";
        @txVeh = split (/\,/,$item);
        ##print "$txVeh[3] \n";

        if ($txVeh[3] >= $timeline ) {
	$average= $counter/$timeStep; 
        ##print " -- Seconds:$timeline, number=$counter, average=$average\n"; 
        print "$timeline $average\n"; 
        $timeline+=$timeStep; $counter=1;
        } else { $counter++;}

}#END foreach
}#end sub



sub tx_space {
my @tx=`/bin/grep stats $file | /bin/grep tx `;
#print @tx;

print "#Number of Tx in space \n#X Position, number of Tx \n";

#OK first obtain only the X position of every TX/RX
foreach $item (@tx) {
        ##print " $item ";
        my @txVeh = split (/\,/,$item);
	$txVeh[5] =~ s/\(//g; #Chop the first string the '('
        ##print "$txVeh[5] \n";
        push(@final,$txVeh[5]);
}#END foreach

my @final = sort { $a <=> $b } @final; # Sort my numeric array

#Have the average of TX/RS every T distance
my $counter=1; my $timeline=0;
foreach $item (@final) {
	#print "$item \n";
        if ($item >= $timeline ) {
        $average= $counter/$timeStep;
        #print " -- Seconds:$timeline, number=$counter, average=$average\n";
        #print "$timeline $counter \n"; 
        #print "$timeline $average\n";
        print "$average\n";
        $timeline+=$timeStep; $counter=1;
        } else { $counter++;}

}#End @final
}#end sub



sub rx_space {
my @rx=`/bin/grep stats $file | /bin/grep rx `;
#print @rx;

print "#Number of Rx in space \n#X Position, number of Rx \n";

#OK first obtain only the X position of every TX/RX
foreach $item (@rx) {
        ##print " $item ";
        my @rxVeh = split (/\,/,$item);
        $rxVeh[6] =~ s/\(//g; #Chop the first string the '('
        ##print "$rxVeh[6]\n";
        push(@final,$rxVeh[6]);
}#END foreach

my @final = sort { $a <=> $b } @final; # Sort my numeric array

#Have the average of TX/RS every T distance
my $counter=1; my $timeline=0;
foreach $item (@final) {
        ##print "$item \n";
        if ($item >= $timeline ) {
        $average= $counter/$timeStep;
        #print " -- Seconds:$timeline, number=$counter, average=$average\n";
        #print "$timeline $counter \n"; 
        #print "$timeline $average\n";
        print "$average\n";
        $timeline+=$timeStep; $counter=1;
        } else { $counter++;}

}#End @final
}#end sub


#line to create the CDF data.
#/home/arturo/SIM-TOOLS/TAPASCologne-0.0.3/testbed/scripts/COURBES/pl/CDF.pl < vehNumberRx.txt > vehNumberRx.dat

sub veh_number_rx {

print "#Number of Rx per vehicle \n#VehicleId, number of Rx \n";
my $maxNumberCars=50;

for ($i=0; $i<=$maxNumberCars; $i++) {
$carId= "veh.".$i.",";
my @rx=`/bin/grep $carId $file | /bin/grep rx - | wc -l `;
chomp @rx;
##print " $i: $carId $rx[0],\n";
print "$rx[0]\n";

$carId= "vehBack.".$i.",";
my @rx=`/bin/grep $carId $file | /bin/grep rx - | wc -l `;
chomp @rx;
##print " $i: $carId $rx[0],\n";
print "$rx[0]\n";
}
}#end sub



sub veh_number_tx {

print "#Number of Tx per vehicle \n#VehicleId, number of Tx \n";
my $maxNumberCars=50;

for ($i=0; $i<=$maxNumberCars; $i++) {
$carId= "veh.".$i.",";
my @rx=`/bin/grep $carId $file | /bin/grep tx - | wc -l `;
chomp @rx;
#print " $i: $carId $rx[0],\n";
print "$rx[0]\n";

$carId= "vehBack.".$i.",";
my @rx=`/bin/grep $carId $file | /bin/grep tx - | wc -l `;
chomp @rx;
#print " $i: $carId $rx[0],\n";
print "$rx[0]\n";
}
}#end sub


