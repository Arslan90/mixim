#!/usr/bin/perl
#############
# This is to create a list of junctions in the small plan 2km square
# This code I'll past in the TraCIDemo.cc
#############

$junctionId=1;
for($y=100; $y<=2100; $y+=100) {

	for($x=100; $x<=2100; $x+=100) {
	print "positionVPA[$junctionId][1]= $x; //JunctionId: $junctionId \n"; 
	print "positionVPA[$junctionId][2]= $y; \n";
	$junctionId++;
	}
}
