#! /bin/sh
# VANETs simulation done by Arslan HAMZA CHERIF
# starting from the november 13th 2014

# Saving some variables
path=`pwd`

# 1st step : launching TraCI
cd ~/omnetpp-4.2/samples/mixim 
./sumo-launchd.py -vv -c sumo -k -d

# 2nd step : launching simulation
cd ~/omnetpp-4.2/samples/mixim/examples/veins
echo "start of the simulation : `date`" > $path/vanet_sim_time.log
git log -n 1 >> $path/vanet_sim_time.log
opp_runall -j2 -V ./veins -f $path/prophet.omnetpp.ini -u Cmdenv -c General -r 0..4
echo "end of the simulation : `date`" >> $path/vanet_sim_time.log
