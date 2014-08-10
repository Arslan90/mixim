##### GREP TO OBTAIN THE DISCRET VALUES FOR TX OR RX
grep stats accident-borrar.log | grep tx - | cut -d"," -f3 | uniq -c | mor
e > veh_tx_discret.txt 

grep stats accident-borrar.log | grep rx - |  cut -d"," -f4 | uniq -c > ve
h_rx_discret.txt
## Fast scripts for charting VEINS results

set title "Average of Tx and Rx of messages in TIME "
#set xrange [0:4]
#set key 4000,0.7
#set xtics ("6h00" 60, "6h30" 1800, "7h00" 3600, "7h30" 5400, "8h00" 7200)
##set xlabel "Simulation Time (seconds)"
set ylabel "average of messages"
set xlabel "Simulation Time (Seconds)"
set grid
plot \
"./time_rx.dat" u 1:2 w l t"Rx",\
"./time_tx.dat" u 1:2 w l t"Tx"


# THIS IS the distribution of messages along the 
set title "Distribution of average Tx/Rx of messages in SPACE"
#set xrange [0:4]
#set key 4000,0.7
set xtics ("1" 100, "2" 200, "3" 300, "4" 400, "4" 400,)
##set xlabel "Simulation Time (seconds)"
set ylabel "average of messages"
set xlabel "Junctions"
set grid
plot \
"./space_tx.dat" u 1:2 w l t"Tx",\
"./space_rx.dat" u 1:2 w l t"Rx"


#MESSAGES RX/TX per ENTITY
# THIS IS the distribution of messages along the 
set title "CDF  Tx/RX of messages per Vehicle (~100 vehicles)"
#set xrange [0:4]
#set key 4000,0.7
#set xtics ("1" 100, "2" 200, "3" 300, "4" 400, "4" 400,)
##set xlabel "Simulation Time (seconds)"
set ylabel "% vehicles"
set xlabel "Number of Messages"
set grid
plot \
"./vehNumberTx.dat" u 1:2 w l t"Tx",\
"./vehNumberRx.dat" u 1:2 w l t"Rx"




############ CREACION DE TX/RX PER JUNCTION ##############
#MESSAGES RX/TX per ENTITY
# THIS IS the distribution of messages along the 
set title "TX messages in junction 1 in TIME."
#set xrange [0:4]
#set key 4000,0.7
#set xtics ("1" 100, "2" 200, "3" 300, "4" 400, "4" 400,)
##set xlabel "Simulation Time (seconds)"
set ylabel "% vehicles"
set xlabel "Number of Messages"
#set grid
plot \
"./junction3_RX.txt" u 2:1 w i t"Tx" #,\
#"./vehNumberRx.dat" u 1:2 w l t"Rx"



######### CUOPLED IMAGES #######
##########" JUNCTION 1
set multiplot layout 3, 1 title "JUNCTION 1 (Radio of 20m)"
#
#set title "RX of messages 1"
set ylabel "RX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction1_RX.txt" u 2:1 w i notitle"Rx"
#
#set title "TX of messages 1"
set ylabel "TX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction1_TX.txt" u 2:1 w i notitle"Tx"
#
#set title "TX of messages 1"
set ylabel "ALIVE"
set yrange [-0.2:1.5]
set xrange [0:1000]
set ytics (0, 10)
plot "./junction1_TX_messageVivant.txt" u 1:2 w steps notitle"Tx"
unset multiplot
#
 

##########" JUNCTION 2
set multiplot layout 3, 1 title "JUNCTION 2 (Radio of 20m)"
#
#set title "RX of messages 1"
set ylabel "RX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction2_RX.txt" u 2:1 w i notitle"Rx"
#
#set title "TX of messages 1"
set ylabel "TX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction2_TX.txt" u 2:1 w i notitle"Tx"
#
#set title "TX of messages 1"
set ylabel "ALIVE"
set yrange [-0.2:1.5]
set xrange [0:1000]
set tic scale 0
set ytics (0,10)
plot "./junction2_TX_messageVivant.txt" u 1:2 w steps notitle"Tx"
#
unset multiplot
#


##########" JUNCTION 3
set multiplot layout 3, 1 title "JUNCTION 3 (Radio of 20m)"
#
#set title "RX of messages 1"
set ylabel "RX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction3_RX.txt" u 2:1 w i notitle"Rx"
#
#set title "TX of messages 1"
set ylabel "TX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction3_TX.txt" u 2:1 w i notitle"Tx"
#
#set title "TX of messages 1"
set ylabel "ALIVE"
set yrange [-0.2:1.5]
set xrange [0:1000]
set tic scale 0
set ytics (0,10)
plot "./junction3_TX_messageVivant.txt" u 1:2 w steps notitle"Tx"
#
unset multiplot
#


##########" JUNCTION 4
set multiplot layout 3, 1 title "JUNCTION 4 (Radio of 20m)"
#
#set title "RX of messages 1"
set ylabel "RX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction4_RX.txt" u 2:1 w i notitle"Rx"
#
#set title "TX of messages 1"
set ylabel "TX"
set yrange [0:]
set xrange [0:1000]
set ytics (0, 1, 2, 3,4,5,6,7,8,9,10,13,15)
plot "./junction4_TX.txt" u 2:1 w i notitle"Tx"
#
#set title "TX of messages 1"
set ylabel "ALIVE"
set yrange [-0.2:1.5]
set xrange [0:1000]
set tic scale 0
set ytics (0,10)
plot "./junction4_TX_messageVivant.txt" u 1:2 w steps notitle"Tx"
#
unset multiplot
#

