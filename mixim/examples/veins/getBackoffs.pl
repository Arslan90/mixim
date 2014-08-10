#!/usr/bin/perl

###########################
# This obtain the backoff tries from logs

$file=1;

for ($file=1; $file <= 5; $file++) {
@uno =`grep 'Nr. $file' results/logs-1.txt  | wc -l`;
@dos =`grep 'Nr. $file' results/logs-2.txt  | wc -l`;
@tres =`grep 'Nr. $file' results/logs-3.txt  | wc -l`;
@cuatro =`grep 'Nr. $file' results/logs-4.txt  | wc -l`;
@cinco =`grep 'Nr. $file' results/logs-5.txt  | wc -l`;

#print " $uno[0], $dos[0], $tres[0], $cuatro[0], $cinco[0] \n";
$total= $uno[0]+ $dos[0]+ $tres[0]+ $cuatro[0]+ $cinco[0];
#print "Total $total \n";
$total = int($total/5);
#ok here print the backoffs attempts
print "$file $total \n";

}

