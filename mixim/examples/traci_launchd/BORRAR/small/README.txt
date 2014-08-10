
set title "TX/Rx of messages Per junction "
#set xrange [0:4]
#set key 4000,0.7
#set xtics ("6h00" 60, "6h30" 1800, "7h00" 3600, "7h30" 5400, "8h00" 7200)
##set xlabel "Simulation Time (seconds)"
set ylabel "total of Junctions: 441"
set xlabel "Number of Messages"
set grid
plot \
"./TX.cdf" u 1:2 w l t"TX",\
"./RX.cdf" u 1:2 w l t"RX"


set title "Number of Vehicles that crossed junctions "
#set xrange [0:4]
#set key 4000,0.7
#set xtics ("6h00" 60, "6h30" 1800, "7h00" 3600, "7h30" 5400, "8h00" 7200)
##set xlabel "Simulation Time (seconds)"
set ylabel "total of Junctions: 441"
set xlabel "Crossing vehicles"
set grid
plot \
"./vehCrossedJunction.cdf" u 1:2 w l t"Vehicles"


set title "Level of dissemination in vehicles with 4 VPAs"
#set xrange [0:4]
#set key 4000,0.7
#set xtics ("6h00" 60, "6h30" 1800, "7h00" 3600, "7h30" 5400, "8h00" 7200)
##set xlabel "Simulation Time (seconds)"
set ylabel "incoming vehicles"
set xlabel "Time simulation (s)"
set grid
plot \
"./runningVehNumberInTime.txt" u 1:2 w l t"Total Vehicles", \
"./disseminationVehNumberInTime.txt" u 1:2 w l t"Vehicles' dissemination"


set title "Number of neighbors dectected in junctions"
#set xrange [0:4]
#set key 4000,0.7
#set xtics ("6h00" 60, "6h30" 1800, "7h00" 3600, "7h30" 5400, "8h00" 7200)
##set xlabel "Simulation Time (seconds)"
set ylabel "Total vehicles in junctions"
set xlabel "Number of neighbors detected"
set grid
plot \
"./neighborsDetectedInJunctionsLogs.cdf" u 1:2 w l t"Neighbors"



