#! /bin/sh
opp_runall -j5 -V ./../veins -f prophet50M.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet100M.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet127M.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet150M.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet200M.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet250M.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet300M.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet50MCR.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet100MCR.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet127MCR.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet150MCR.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet200MCR.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet250MCR.omnetpp.ini -u Cmdenv -c General -r 0..19
opp_runall -j5 -V ./../veins -f prophet300MCR.omnetpp.ini -u Cmdenv -c General -r 0..19
