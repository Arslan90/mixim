#!/usr/bin/perl
use POSIX qw(ceil floor);

sub by_number { $a <=> $b }
$num_tot = 0;

print "#HERE START ARRAY \n";

while (<>) {
@inp = split / /;
chomp($inp[0]);
if (exists $freq{$inp[0]}) {
$freq{$inp[0]} ++;
}
else {
$freq{$inp[0]} = 1;
}
$num_tot++;
}
$tempx = 0;

foreach $key (sort by_number keys %freq){
if (defined($key)) {
#print "$key " , $freq{$key}*100/$num_tot, "\n";
#print OUT3 "$key $freq{$key}\n";
$tempx = $tempx + $freq{$key};
print "$key " , $tempx/$num_tot, "\n";
}
}

print "#ARRAY FINAL = $num_tot \n";
