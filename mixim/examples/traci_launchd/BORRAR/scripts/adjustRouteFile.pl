#!/usr/bin/perl
######################
# This is just to adjust the value of the demarrage de voitures a 0 instead of 21600
# Because the Network simulator starts at 0 
######################

$file="cologne.rou.xml";
open(LEE,"$file") || die "cant open file $! \n";
@data=<LEE>;
#print @data;


my $counter=1; 
foreach $item (@data) {
        #print  $item;

	#only change the starting time of vehicle substracting 21600
	if ($item =~ m/vehicle id=/) {
	@line= split(/"/,$item);
	$line[3]= $line[3] - 21600;
        print "$line[0]\"$line[1]\" $line[2]\"$line[3].00\" $line[4]";
	} else { print  $item; }
}
