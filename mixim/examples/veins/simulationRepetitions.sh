#!/bin/bash

#for i in {1..1}
for i in {1..5}
do 
	echo ""
	echo "************** DOING REPETITION $i ***********"
	./veins -u Cmdenv -f paper80211p.omnetpp.ini -r $i
	#./Mac80211 -u Cmdenv -f testing.omnetpp.ini -r $i
done

