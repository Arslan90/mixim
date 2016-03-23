#!/bin/sh

tail results50M/testScript.csv        > globalResults.csv
tail -n 1 results100M/testScript.csv    >> globalResults.csv
tail -n 1 results127M/testScript.csv    >> globalResults.csv
tail -n 1 results150M/testScript.csv    >> globalResults.csv
tail -n 1 results200M/testScript.csv    >> globalResults.csv
tail -n 1 results250M/testScript.csv    >> globalResults.csv
tail -n 1 results300M/testScript.csv    >> globalResults.csv
tail -n 1 results50MCR/testScript.csv   >> globalResults.csv
tail -n 1 results100MCR/testScript.csv  >> globalResults.csv
tail -n 1 results127MCR/testScript.csv  >> globalResults.csv
tail -n 1 results150MCR/testScript.csv  >> globalResults.csv
tail -n 1 results200MCR/testScript.csv  >> globalResults.csv
tail -n 1 results250MCR/testScript.csv  >> globalResults.csv
tail -n 1 results300MCR/testScript.csv  >> globalResults.csv
