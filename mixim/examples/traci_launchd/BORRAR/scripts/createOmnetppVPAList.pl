#!/usr/bin/perl
######################
# This is just to  create the list of VPA for the Omnetpp.ini
# I'm too lazy to type one by one...
#
# IMPORTANT NOTE: 
# IN the sumo network the Y axis is inverted in comparison with my 
# regular use of the Y axis on other plans. 
# Therefore I've to adjust the Y value sustracting the total height of
# of the SUMO ROAD NETWORK which is a heigth of: 41947 meters
# So; here I'm adjusting this value in the VPA locations.
#
######################

$file="vpaXYList.txt";
open(LEE,"$file") || die "cant open file $! \n";
@data=<LEE>;
#print @data;

#Sample:
#scenario.VPA[0].linearMobility.x = 1500
#scenario.VPA[0].linearMobility.y = 500

chomp @data;
my $counter=0; 
foreach $item (@data) {
        #print  "$item \n";
	@line= split(/,/,$item);
	my $offsetY= 41947 - $line[1];
	print "scenario.VPA[$counter].linearMobility.x = $line[0]\n";
	print "scenario.VPA[$counter].linearMobility.y = $offsetY\n";
	$counter++;
}
