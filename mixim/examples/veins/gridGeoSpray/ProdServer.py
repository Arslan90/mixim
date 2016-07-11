import os
import sys
import subprocess
# import traci
import sumolib
#import traci.constants as tc
import myconstants as constx
import networkx as nx
import matplotlib.pyplot as plt
import random

import buildCologne
import buildVPA
import time
import socket
import select
import cPickle as pickle
from thread import *

##################### 11/07/2016

# def retrieveGraphEdge(edgesDict ,edgeID):
#     nodesOfEdge = edgesDict[edgeID]
#     return G.get_edge_data(nodesOfEdge[0],nodesOfEdge[1])

def nodesOfEdgedRoute(edgesDict, Route, WithFirstNode = True):
    nodesAlongRoad = []
    for i, val in enumerate(Route):
        if i == 0 and WithFirstNode:
            nodesAlongRoad.append(edgesDict[val][0].decode().encode('utf-8'))
        nodesAlongRoad.append(edgesDict[val][1].decode().encode('utf-8'))
    return nodesAlongRoad

def edgesOfNodedRoute(Graph, Route, orignalNodedRoute):
    edgeesAlongRoad = []
    for i in range(0, len(Route)-1):
        if Graph.has_edge(Route[i], Route[i+1]):
            if Graph.number_of_edges(Route[i], Route[i+1]) == 1:
                edges = Graph[Route[i]][Route[i+1]]
                edgeesAlongRoad.append(edges[0]["label"].decode().encode('utf-8'))
            elif Graph.number_of_edges(Route[i], Route[i+1]) > 1:
                edges = Graph[Route[i]][Route[i + 1]]
                rightEdge = orignalNodedRoute[len(edgeesAlongRoad)]
                for j, val in edges.iteritems():
                    if val == rightEdge:
                        break
                edgeesAlongRoad.append(edges[j]["label"].decode().encode('utf-8'))
        else:
            raise ValueError("Unable to retrieve edge between nodes:", Route[i], "&", Route[i+1])
    return edgeesAlongRoad

def edgesOfNodedRoute(Graph, Route, shortest=True):
    edgeesAlongRoad = []
    totalLength = 0
    for i in range(0, len(Route)-1):
        if Graph.has_edge(Route[i], Route[i+1]):
            edge = ""
            edges = Graph[Route[i]][Route[i + 1]]
            if Graph.number_of_edges(Route[i], Route[i+1]) > 1 and shortest == True:
                for j, val in edges.iteritems():
                    length = sys.maxint
                    if val["weight"] < length:
                        length = val["weight"]
                        edge = val
            else:
                edge = edges[0]
            edgeesAlongRoad.append(edge["label"].decode().encode('utf-8'))
            totalLength += float(edge["weight"])
        else:
            raise ValueError("Unable to retrieve edge between nodes:", Route[i], "&", Route[i+1])
    return edgeesAlongRoad, totalLength

def remainingEdgedRoute(currentEdge, Route = []):
    remainingRoute = []
    index = -1
    for i, val in enumerate(Route):
        if val == currentEdge:
            index = i
            break
    if index >= 0:
        remainingRoute = Route[index:]
    return remainingRoute

def remainingNodedRoute(currentNode, Route = []):
    remainingRoute = []
    index = -1
    for i, val in enumerate(Route):
        if val == currentNode:
            index = i
            break
    if index >= 0:
        remainingRoute = Route[index:]
    return remainingRoute

def indexRoadIDInRoute(currentRoadID, Route = []):
    index = -1
    for i, val in enumerate(Route):
        if val == currentRoadID:
            index = i
            break
    return index

def indexNodeIDInRoute(currentNodeID, Route = []):
    index = -1
    for i, val in enumerate(Route):
        if val == currentNodeID:
            index = i
            break
    return index

def extractNodeIDFromInvalidRoadID(currentRoadID):
    nodeID = ""
    tokens = currentRoadID.split('_')
    nodeID = tokens[0].translate(None, ':')
    if nodeID == "":
        raise ValueError("Unable to extract currentNodeID from InvalidRoadID")
    return nodeID

def randomNodeID(Graph):
    randomNode = ""
    if (len(Graph.nodes())) > 0:
        randomNode = Graph.nodes()[random.randrange(0, len(Graph.nodes()))]
    else:
        raise ValueError("Number of nodes in the network is equal to zero (0)")
    print "[MyProg] RandomNode:", randomNode
    return randomNode

def randomTargetVPA(sumo_VPA):
    if (len(sumo_VPA)) > 0:
        randomVPA = random.randrange(0, len(sumo_VPA()))
    else:
        raise ValueError("Number of nodes in the network is equal to zero (0)")
    print "[MyProg] RandomVPDIndex:", randomVPA
    return randomVPA

def nearestNodeToTarget(Graph, targetNode, nodedRoute = []):
    allResults = []
    nearestNode =""
    nearestDist = sys.maxint
    for i,val in enumerate(nodedRoute):
        if nx.has_path(Graph,val,targetNode):
            currentDist = nx.dijkstra_path_length(Graph, val, targetNode)
            allResults.append((val, currentDist))
            if currentDist < nearestDist:
                nearestNode = val
                nearestDist = currentDist
    # print str(allResults)
    return nearestNode, nearestDist

def nearestNodeToVPA(Graph, edgesDict, map_list, map_type, nodedRoute):
    nearestNodes = ("","")
    nearestDist = sys.maxint
    for i,val in enumerate(nodedRoute):
        if (map_type == "None") or (map_type == None):
            break
        else:
            for j, val2 in enumerate(map_list):
                if map_type == "Node":
                    currentDist = dijkstraPathLength(Graph, val, val2[0])
                    if currentDist < nearestDist:
                        nearestNodes = (val,val2[0])
                        nearestDist = currentDist
                elif map_type == "Edge":
                    fromNode = edgesDict[val2[0]][0]
                    toNode = edgesDict[val2[0]][1]
                    currentDist = dijkstraPathLength(Graph, val, fromNode)
                    if currentDist < nearestDist:
                        nearestNodes = (val, fromNode)
                        nearestDist = currentDist
                    currentDist = dijkstraPathLength(Graph, val, toNode)
                    if currentDist < nearestDist:
                        nearestNodes = (val, fromNode)
                        nearestDist = currentDist
    return nearestNodes, nearestDist

def nearestNodeToVPA_dist_path(Graph, edgesDict, map_list, map_type, map_length_dist, nodedRoute):
    nearestNodes = (" "," ")
    nearestDist = sys.maxint
    nearestPath = ""
    # for i,val in enumerate(nodedRoute):
    #     if (map_type == "None") or (map_type == None):
    #         break
    #     else:
    for j, val2 in enumerate(map_list):
        if (map_type == "None") or (map_type == None):
            break
        elif map_type == "Node":
            length = map_length_dist[val2[0]][0]
            path = map_length_dist[val2[0]][1]
            for key, value in length.iteritems():
                if key not in nodedRoute:
                    continue
                if value < nearestDist:
                    nearestNodes = (val2[0], key.decode().encode('utf-8'))
                    nearestDist = value
                    nearestPath = path[key]
        elif map_type == "Edge":
            fromNode = edgesDict[val2[0]][0]
            toNode = edgesDict[val2[0]][1]
            length = map_length_dist[fromNode][0]
            path = map_length_dist[fromNode][1]
            for key, value in length.iteritems():
                if key not in nodedRoute:
                    continue
                if value < nearestDist:
                    nearestNodes = (val2[0], key.decode().encode('utf-8'))
                    nearestDist = value
                    nearestPath = path[key]
            length = map_length_dist[toNode][0]
            path = map_length_dist[toNode][1]
            for key, value in length.iteritems():
                if key not in nodedRoute:
                    continue
                if value < nearestDist:
                    nearestNodes = (val2[0], key.decode().encode('utf-8'))
                    nearestDist = value
                    nearestPath = path[key]
    return nearestNodes, nearestDist, nearestPath

def dijkstraPathLength(Graph, node1, node2):
    if nx.has_path(Graph, node1, node2):
        return nx.dijkstra_path_length(Graph, node1, node2)
    else:
        return sys.maxint

def sumo2omnet(sumo_x,sumo_y, netBound1, netBound2, margin):
    omnet_x = sumo_x - netBound1[0] + margin
    omnet_y = (netBound2[1] - netBound1[1]) - (sumo_y - netBound1[1]) + margin
    return omnet_x, omnet_y

def omnet2sumo(omnet_x,omnet_y, netBound1, netBound2, margin):
    sumo_x = omnet_x + netBound1[0] - margin
    sumo_y = (netBound2[1] - netBound1[1]) - (omnet_y - netBound1[1]) + margin
    return sumo_x, sumo_y

def buildGraph(netFile, graph, edgeDict):
    net = sumolib.net.readNet(netFile)
    for Node in net.getNodes():
        graph.add_node(Node._id, pos=(Node._coord[0], Node._coord[1]))

    for Edge in net.getEdges():
        graph.add_edge(Edge._from._id, Edge._to._id, label=Edge.getID(), weight=float(Edge.getLength()), maxSpeed=float(Edge.getSpeed()))
        edgeDict[Edge.getID()] = (Edge._from._id,Edge._to._id)

    return graph, edgeDict

def edgesOfNearestPoint(Route, NP_Node, edgeDict):
    edge_to_NP = ""
    edge_from_NP = ""
    for j, val2 in enumerate(Route):
        if edge_to_NP != "" and edge_from_NP != "":
            break
        if edgeDict[val2][0] == NP_Node:
            edge_from_NP = val2.decode().encode('utf-8')
        if edgeDict[val2][1] == NP_Node:
            edge_to_NP = val2.decode().encode('utf-8')
    edgesOfNP = (edge_to_NP, edge_from_NP)
    return edgesOfNP




def main(argv):

    def compute_NP_ALL(VPA_ID, Route):
        service_time = time.time()
        tokensForEdgedRoute = Route.split(' ')
        edgedRoute = []
        for j, val2 in enumerate(tokensForEdgedRoute):
            edgedRoute.append(val2)
        nodedRoute = nodesOfEdgedRoute(dict_edges, edgedRoute, WithFirstNode = False)
        nearestNodes, dist, path = nearestNodeToVPA_dist_path(G, dict_edges, mapped_VPA[int(VPA_ID)],mapped_Type_VPA[int(VPA_ID)], mapped_VPA_length_dist,nodedRoute)
        reversedPath = path[::-1]
        edge_to_NP = " "
        edge_from_NP = " "
        for j, val2 in enumerate(edgedRoute):
            if edge_to_NP != " " and edge_from_NP != " ":
                break
            if dict_edges[val2][0] == nearestNodes[1]:
                edge_from_NP = val2.decode().encode('utf-8')
            if dict_edges[val2][1] == nearestNodes[1]:
                edge_to_NP = val2.decode().encode('utf-8')
        edgesOfNP = edgesOfNearestPoint(edgedRoute, nearestNodes[1], dict_edges)
        edgedRoute_NP_VPA, dist_NP_VPA = edgesOfNodedRoute(G, reversedPath, shortest=True)
        if (dist_NP_VPA != dist):
            print "Distance in the 2 direction are not the same\n"
        # reversedPath.reverse()
        service_time = time.time() - service_time
        print "[VPA_NODE,NP_NODE]", str(nearestNodes), "[Edge->NP, NP->Edge]", edgesOfNP, "[Dist_NP_VPA]", str(
            dist), "[Route_NP_VPA]", str(edgedRoute_NP_VPA), "Service duration", service_time, "\n"
        route_as_string = ""
        for i, val in enumerate(edgedRoute_NP_VPA):
            if i == 0:
                route_as_string = val.decode().encode('utf-8')
            else:
                route_as_string = route_as_string + " " + val.decode().encode('utf-8')
        if edgedRoute_NP_VPA == []:
            route_as_string = " "
        msg = [str(constx.RESPONSE),str(constx.RESPONSE_NP_ALL),str(constx.EDGE_TO_NP),str(edgesOfNP[0]), str(constx.NODE_NP),str(nearestNodes[1]), str(constx.EDGE_FROM_NP),str(edgesOfNP[1]) , str(constx.NODE_VPA_MAPPING),str(nearestNodes[0]), str(constx.ROUTE_NP_VPA),str(route_as_string), str(constx.ROUTE_LENGTH_NP_VPA),str(float(dist_NP_VPA))]
        msg_as_string = ""
        for j in xrange(0, len(msg), 2):
            msg_as_string = msg_as_string + msg[j].decode().encode('utf-8') + ":" + msg[j + 1].decode().encode('utf-8')
            if j == len(msg)-2:
                msg_as_string = msg_as_string+"#"
            else:
                msg_as_string = msg_as_string + ";"
        return msg_as_string

    def compute_EDGE_BEST_TRAVEL_TIME(EDGE_ID):
        travelTime = sys.maxint
        service_time = time.time()
        nodesOfEdge = ("","")
        nodesOfEdge = dict_edges[EDGE_ID]
        if (nodesOfEdge != ("","")) and (G.has_edge(nodesOfEdge[0],nodesOfEdge[1])):
            edges = G[nodesOfEdge[0]][nodesOfEdge[1]]
            for j, val in edges.iteritems():
                if val["label"] == EDGE_ID:
                    if val["maxSpeed"] != 0.0:
                        travelTime = val["weight"] / val["maxSpeed"]
                    else:
                        raise ValueError("Edge Max Speed value equals 0.0")
        else:
            raise ValueError("Edge not found")
        service_time = time.time() - service_time
        print "[EDGE_ID]", str(EDGE_ID), "[EDGE_BEST_TRAVEL_TIME]", str(travelTime), "\n"
        msg = [str(constx.RESPONSE),str(constx.RESPONSE_EDGE_BEST_TRAVEL_TIME),str(constx.EDGE_BEST_TRAVEL_TIME),str(float(travelTime))]
        msg_as_string = ""
        for j in xrange(0, len(msg), 2):
            msg_as_string = msg_as_string + msg[j].decode().encode('utf-8') + ":" + msg[j + 1].decode().encode('utf-8')
            if j == len(msg)-2:
                msg_as_string = msg_as_string+"#"
            else:
                msg_as_string = msg_as_string + ";"
        return msg_as_string

#    sys.stdout = open("PyServerLog.txt", "w")

    start_time = time.time()
    random.seed(0)

    dict_edges = {}

    G=nx.MultiDiGraph()

    networkFile = "/media/DATA/DoctoratTlemcen/Git/mixim/mixim/examples/veins/gridGeoDTN/grid_9x9.net.xml"
    G, dict_edges = buildGraph(networkFile, G, dict_edges)

    print "Nbr Nodes:", G.number_of_nodes()
    print "Nbr Edges:", G.number_of_edges()

    with open("PyServerLog.txt", "a") as outFile:
        outFile = open("PyServerLog.txt", "w")
        outFile.write("Nbr Nodes:"+str(G.number_of_nodes()))
        outFile.write("\nNbr Edges:"+str(G.number_of_edges()))
        outFile.close()

    radio_range = 127
    netBound1 = (0, 0)
    netBound2 = (1650, 1650)
    margin = 10


    omnet_VPA = [(760,760)]
    sumo_x, sumo_y = omnet2sumo(omnet_VPA[0][0],omnet_VPA[0][1], netBound1, netBound2, margin)
    sumo_VPA = [(sumo_x,sumo_y)]

    print "Nbr VPAs:", len(sumo_VPA), "(SUMO)", len(omnet_VPA), "(OMNET)"

    with open("PyServerLog.txt", "a") as outFile:
        outFile.write("\nNbr VPAs:"+str(len(sumo_VPA))+"(SUMO)"+str(len(omnet_VPA))+"(OMNET)")

    mapped_VPA = []
    mapped_Type_VPA = []

    mapped_VPA, mapped_Type_VPA = buildVPA.mapping_VPA(G, sumo_VPA, radio_range)

    NbrMappedToNodes = 0
    NbrMappedToEdges = 0
    NbrNotMapped = 0

    for i, val in enumerate(mapped_Type_VPA):
        if val == "Node":
            NbrMappedToNodes +=1
        if val == "Edge":
            NbrMappedToEdges +=1
        if (val == "None") or (val == None):
            NbrNotMapped +=1

    mapped_VPA_length_dist = {}

    for i, val in enumerate(mapped_VPA):
        # if mapped_Type_VPA[i] != "Node":
        if (mapped_Type_VPA[i] == "None") or (mapped_Type_VPA[i] == None):
            continue
        for j, val2 in enumerate(val):
            if mapped_Type_VPA[i] == "Node":
                length, path = nx.single_source_dijkstra(G, val2[0])
                mapped_VPA_length_dist[val2[0]] = (length, path)
            elif mapped_Type_VPA[i] == "Edge":
                fromNode = dict_edges[val2][0]
                length, path = nx.single_source_dijkstra(G, fromNode)
                mapped_VPA_length_dist[fromNode] = (length, path)
                toNode = dict_edges[val2][1]
                length, path = nx.single_source_dijkstra(G, toNode)
                mapped_VPA_length_dist[toNode] = (length, path)


    print "Nbr Mapped To Node:", NbrMappedToNodes, "To Edge:", NbrMappedToEdges, "To None:", NbrNotMapped
    print("--- %s seconds ---" % (time.time() - start_time))

    with open("PyServerLog.txt", "a") as outFile:
        outFile.write("\nNbr Mapped To Node:"+str(NbrMappedToNodes)+"To Edge:"+str(NbrMappedToEdges)+"To None:"+str(NbrNotMapped))
        outFile.write("\n--- %s seconds ---: "+str((time.time() - start_time)))

    HOST = '127.0.0.1'  # Local host
    PORT = 19999  # Aun port arbitraire
    RECV_BUFFER = 4096

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(0)

    # Function for handling connections. This will be used to create threads
    def clientthread(conn,addr):
        # infinite loop so that function do not terminate and thread do not end.
        while True:

            # Receiving from client
            data = conn.recv(RECV_BUFFER)
            if data == "CLOSE":
                print "Client (%s, %s) is going offline" % addr
                conn.close()
                break
            else:
                tmp_str = data
                tokens = tmp_str.split('#')
                for i, val in enumerate(tokens):
                    if val == "":
                        continue
                    tokens_msg = val.split(';')
                    # for j, val2 in enumerate(tokens_msg):
                    tokens_command = tokens_msg[0].split(':')
                    msg_type = int(tokens_command[0])
                    cmd = int(tokens_command[1])
                    tokens_args = []
                    for j, val2 in enumerate(tokens_msg[1:]):
                        tmp = val2.split(':')
                        tokens_args.append(int(tmp[0]))
                        tokens_args.append(tmp[1])
                    if msg_type == constx.QUERY:
                        if cmd in constx.KNOWN_QUERIES:
                            cmd_format = constx.FORMAT_QUERIES[cmd]
                            error = False
                            if len(tokens_args) != len(cmd_format):
                                error = True
                                for j in xrange(2, len(tokens_args), 2):
                                    if tokens_msg[j] != cmd_format[j]:
                                        error = True
                            if error:
                                raise ValueError("Error query format")
                            if cmd == constx.CMD_NP_ALL:
                                response_msg = compute_NP_ALL(tokens_args[1], tokens_args[3])
                                conn.send(response_msg)
                            elif cmd == constx.CMD_EDGE_BEST_TRAVEL_TIME:
                                response_msg = compute_EDGE_BEST_TRAVEL_TIME(tokens_args[1])
                                conn.send(response_msg)
                    elif msg_type == constx.RESPONSE:
                        print "Not supported for now"
                    else:
                        raise ValueError("unrecognazible msg type received")
        #
        # # came out of loop
        # conn.close()

    # now keep talking with the client
    while 1:
        # wait to accept a connection - blocking call
        conn, addr = server_socket.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1])

        # start new thread takes 1st argument as a function name to be run, second is the tuple of arguments to the function.
        start_new_thread(clientthread, (conn,addr))

    server_socket.close()


    # CONNECTION_LIST = []  # list of socket clients
    #
    #
    # # Add server socket to the list of readable connections
    # CONNECTION_LIST.append(server_socket)
    #
    # while 1:
    #     # Get the list sockets which are ready to be read through select
    #     read_sockets, write_sockets, error_sockets = select.select(CONNECTION_LIST, [], [])
    #
    #     for sock in read_sockets:
    #
    #         # New connection
    #         if sock == server_socket:
    #             # Handle the case in which there is a new connection recieved through server_socket
    #             sockfd, addr = server_socket.accept()
    #             CONNECTION_LIST.append(sockfd)
    #             print "Client (%s, %s) connected" % addr
    #
    #         # Some incoming message from a client
    #         else:
    #             # Data recieved from client, process it
    #             try:
    #                 # In Windows, sometimes when a TCP program closes abruptly,
    #                 # a "Connection reset by peer" exception will be thrown
    #                 data = sock.recv(RECV_BUFFER)
    #                 if data == "CLOSE":
    #                     print "Client (%s, %s) is going offline" % addr
    #                     sock.close()
    #                     CONNECTION_LIST.remove(sock)
    #                     continue
    #                 tmp_str = data
    #                 tokens = tmp_str.split('#')
    #                 for i, val in enumerate(tokens):
    #                     if val == "":
    #                         continue
    #                     tokens_msg = val.split(';')
    #                     # for j, val2 in enumerate(tokens_msg):
    #                     tokens_command = tokens_msg[0].split(':')
    #                     msg_type = int(tokens_command[0])
    #                     cmd = int(tokens_command[1])
    #                     tokens_args = []
    #                     for j, val2 in enumerate(tokens_msg[1:]):
    #                         tmp = val2.split(':')
    #                         tokens_args.append(int(tmp[0]))
    #                         tokens_args.append(tmp[1])
    #                     if msg_type == constx.QUERY:
    #                         if cmd in constx.KNOWN_QUERIES:
    #                             cmd_format = constx.FORMAT_QUERIES[cmd]
    #                             error = False
    #                             if len(tokens_args) != len(cmd_format):
    #                                 error = True
    #                                 for j in xrange(2, len(tokens_args), 2):
    #                                     if tokens_msg[j] != cmd_format[j]:
    #                                         error = True
    #                             if error:
    #                                 raise ValueError("Error query format")
    #                             if cmd == constx.CMD_NP_ALL:
    #                                 response_msg = compute_NP_ALL(tokens_args[1], tokens_args[3])
    #                                 sock.send(response_msg)
    #                             elif cmd == constx.CMD_EDGE_BEST_TRAVEL_TIME:
    #                                 response_msg = compute_EDGE_BEST_TRAVEL_TIME(tokens_args[1])
    #                                 sock.send(response_msg)
    #                     elif msg_type == constx.RESPONSE:
    #                         print "Not supported for now"
    #                     else:
    #                         raise ValueError("unrecognazible msg type received")
    #
    #             # client disconnected, so remove from socket list
    #             except:
    #                 print "Client (%s, %s) is offline" % addr
    #                 sock.close()
    #                 CONNECTION_LIST.remove(sock)
    #                 continue
    #
    # server_socket.close()

    # conn, addr = server_socket.accept()
    # while True:
    #     print 'Connected by client', addr
    #     data = conn.recv(4096)
    #     if not data:
    #         print ("no data")
    #     else:
    #         print 'Received data from client', data
    #         if data == "CLOSE":
    #             conn.close()
    #         else:
    #             tmp_str=data
    #             tokens = tmp_str.split('#')
    #             for i, val in enumerate(tokens):
    #                 if val == "":
    #                     continue
    #                 tokens_msg = val.split(';')
    #                 # for j, val2 in enumerate(tokens_msg):
    #                 tokens_command = tokens_msg[0].split(':')
    #                 msg_type = int(tokens_command[0])
    #                 cmd = int(tokens_command[1])
    #                 tokens_args =[]
    #                 for j, val2 in enumerate(tokens_msg[1:]):
    #                     tmp = val2.split(':')
    #                     tokens_args.append(int(tmp[0]))
    #                     tokens_args.append(tmp[1])
    #                 if msg_type == constx.QUERY:
    #                     if cmd in constx.KNOWN_QUERIES:
    #                         cmd_format = constx.FORMAT_QUERIES[cmd]
    #                         error = False
    #                         if len(tokens_args) != len(cmd_format):
    #                             error = True
    #                             for j in xrange(2, len(tokens_args),2):
    #                                 if tokens_msg[j] != cmd_format[j]:
    #                                     error = True
    #                         if error:
    #                             raise ValueError("Error query format")
    #                         if cmd == constx.CMD_NP_ALL:
    #                             response_msg = compute_NP_ALL(tokens_args[1], tokens_args[3])
    #                             conn.send(response_msg)
    #                         elif cmd == constx.CMD_EDGE_BEST_TRAVEL_TIME:
    #                             response_msg = compute_EDGE_BEST_TRAVEL_TIME(tokens_args[1])
    #                             conn.send(response_msg)
    #                 elif msg_type == constx.RESPONSE:
    #                     print "Not supported for now"
    #                 else:
    #                     raise ValueError("unrecognazible msg type received")
    #             # cmd_args = cmd.split(';')
    #             # nbrArgs = len(cmd_args)
    #             # print "nbr arguments", nbrArgs
    #             # cmdID = cmd_args[0]
    #             # print "command ID", cmdID
    #             # # dist = dijkstraPathLength(G, cmd_args[1], cmd_args[2])
    #             # # conn.send("Shortest distance between:"+str(cmd_args[1])+"&"+str(cmd_args[2])+"="+str(dist)+"\n")
    #             # # conn.send('Thank you for connecting\n')
    #             # vpa_index = cmd_args[1]
    #             # tokensForEdgedRoute = cmd_args[2].split(' ')
    #             # edgedRoute = []
    #             # for j, val2 in enumerate(tokensForEdgedRoute):
    #             #     edgedRoute.append(val2)
    #             # nodedRoute = nodesOfEdgedRoute(dict_edges, edgedRoute)
    #             # nearestNodes, dist, path = nearestNodeToVPA_dist_path(G, dict_edges, mapped_VPA[int(vpa_index)], mapped_Type_VPA[int(vpa_index)], mapped_VPA_length_dist, nodedRoute)
    #             # # nearestNodes, dist = nearestNodeToVPA(G, dict_edges, mapped_VPA[int(vpa_index)], mapped_Type_VPA[int(vpa_index)], nodedRoute)
    #             # service_time = time.time() - service_time
    #             # reversedPath = path[::-1]
    #             # edge_to_NP = ""
    #             # edge_from_NP = ""
    #             # for j, val2 in enumerate(edgedRoute):
    #             #     if edge_to_NP != "" and edge_from_NP != "":
    #             #         break
    #             #     if dict_edges[val2][0] == nearestNodes[1]:
    #             #         edge_from_NP = val2.decode().encode('utf-8')
    #             #     if dict_edges[val2][1] == nearestNodes[1]:
    #             #         edge_to_NP = val2.decode().encode('utf-8')
    #             # edgesOfNP = edgesOfNearestPoint(edgedRoute, nearestNodes[1], dict_edges)
    #             # edgedRoute_NP_VPA, dist_NP_VPA = edgesOfNodedRoute(G, reversedPath, shortest=True)
    #             # if (dist_NP_VPA != dist):
    #             #     print "Distance in the 2 direction are not the same\n"
    #             # # reversedPath.reverse()
    #             # print "[VPA_NODE,NP_NODE]", str(nearestNodes), "[Edge->NP, NP->Edge]", edgesOfNP,"[Dist_NP_VPA]", str(dist),"[Route_NP_VPA]", str(edgedRoute_NP_VPA) ,"Service duration", service_time,"\n"
    #             # route_as_string = ""
    #             # for i, val in enumerate(edgedRoute_NP_VPA):
    #             #     if i == 0:
    #             #         route_as_string = val.decode().encode('utf-8')
    #             #     else:
    #             #         route_as_string = route_as_string+" "+val.decode().encode('utf-8')
    #             # # conn.send("[VPA_NODE]:"+str(nearestNodes[0])+";[NP_NODE]:"+str(nearestNodes[1])+";[NP_EdgeTo]:"+str(edgesOfNP[0])+";[NP_EdgeFm]:"+str(edgesOfNP[1])+";[Dist_NP_VPA]:"+str(dist)+";[Route_NP_VPA]:"+str(edgedRoute_NP_VPA))
    #             # service_time = time.time() - service_time
    #             # conn.send("[VPA_NODE]:" + str(nearestNodes[0]) + ";[NP_NODE]:" + str(nearestNodes[1]) + ";[NP_EdgeTo]:" + str(edgesOfNP[0]) + ";[NP_EdgeFm]:" + str(edgesOfNP[1]) + ";[Dist_NP_VPA]:" + str(dist) + ";[Route_NP_VPA]:" + str(route_as_string))
    #             # conn.send("Shortest distance between:" + str(nearestNodes[0]) + "&" + str(nearestNodes[1]) + "=" + str(dist) + ":"+ str(edgedRoute_NP_VPA) +"NP Edges:"+ str(edgesOfNP) +";")
    #             # conn.send('Thank you for connecting\n')

        # conn.close()
if __name__ == "__main__":
    main(sys.argv)
