import networkx as nx 
import random
import sys
import os
import math
import subprocess
from subprocess import call
import numpy as np
import matplotlib.pyplot as plt 
import getopt
import multiprocessing  # the module we will be using for multiprocessing
##############################################################################
def worker(param):
##############################################################################
    """ 
    Multiprocessing work 
    """ 
    print param 
    os.system(param)
########################################################################################
def main(argv):
    # # os.system(' /media/DATA/DoctoratTlemcen/Git/mixim/mixim/sumo-launchd.py -vv -c sumo -d >Sumo.out 2>Sumo.err ')
    # # os.system(' python ProdServer.py >PyServer.out 2>PyServer.err ')
    #
    # pythonBin = "/usr/bin/python"
    # dir = os.getcwd()
    # # out = "PyServer.out"
    # # err = "PyServer.err"
    # with open("PyServer.out", "wb") as out, open("PyServer.err", "wb") as err:
    #     subprocess.Popen([pythonBin, "ProdServer.py"], stdout=out, stderr=err, cwd=dir)
    CommandLines=[]
    for i in range(0, 20):
        CommandLines.append(' /media/DATA/DoctoratTlemcen/Git/mixim/mixim/examples/veins/veins -f /media/DATA/DoctoratTlemcen/Git/mixim/mixim/examples/veins/gridGeoDTN/MagiGeoDTN.omnetpp.ini -u Cmdenv -c General -r '+str(i))
    #number_processes = multiprocessing.cpu_count()
    number_processes = 2
    pool = multiprocessing.Pool(number_processes)
    results = pool.map_async(worker, CommandLines)
    pool.close()
    pool.join()
if __name__ == "__main__":
    sys.exit(main(sys.argv))
