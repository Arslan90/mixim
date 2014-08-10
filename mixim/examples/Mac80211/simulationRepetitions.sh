#!/bin/bash

#for i in {1..2}
for i in {1..20}
do 
	echo ""
	echo "************** DOING REPETITION $i ***********"
	./Mac80211 -u Cmdenv -f testing.omnetpp.ini -r $i
done

#!/bin/sh
#for ( (i=1; $i<5; i++) ); do
#	./Mac80211 -u Cmdenv -f testing.omnetpp.ini  -r $i
#done
