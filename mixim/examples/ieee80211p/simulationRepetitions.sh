#!/bin/bash

for i in {1..1}
#for i in {1..20}
do 
	echo ""
	echo "************** DOING REPETITION $i ***********"
	./ieee80211p -u Cmdenv -f testing.omnetpp.ini -r $i
	#./Mac80211 -u Cmdenv -f testing.omnetpp.ini -r $i
done

#!/bin/sh
#for ( (i=1; $i<5; i++) ); do
#	./Mac80211 -u Cmdenv -f testing.omnetpp.ini  -r $i
#done
