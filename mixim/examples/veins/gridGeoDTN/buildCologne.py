import os
import sys
import networkx as nx
import matplotlib.pyplot as plt

def build_nodes(G, fileName):
    myFile = open(fileName, "r")
    f_line = myFile.readline()
    while f_line <> "":
        tokens = f_line.split(' ')
        # print tokens[1]
        # print tokens[3].translate(None, '[,')
        # print tokens[4].translate(None, ']')
        G.add_node(tokens[1], pos=(float(tokens[3].translate(None, '[,')), float(tokens[4].translate(None, ']'))))
        f_line = myFile.readline()
    myFile.close()
    return G

def build_edges(G, fileName):
    dict_edges = {}
    myFile = open(fileName, "r")
    f_line = myFile.readline()
    while f_line <> "":
        tokens = f_line.split(' ')
        # print tokens[1]
        # print tokens[3]
        # print tokens[5]
        # print tokens[7]
        G.add_edge(tokens[3], tokens[5], label=tokens[1], weight=float(tokens[7]))
        dict_edges[tokens[1]] = (tokens[3], tokens[5])
        f_line = myFile.readline()
    myFile.close()
    return G, dict_edges