import os
import sys
import math
# import vectors

#############################################################

################## Functions for vectors ####################

#############################################################

def dot(v,w):
    # x,y,z = v
    # X,Y,Z = w
    # return x*X + y*Y + z*Z
    return v[0]*w[0]+v[1]*w[1]

def length(v):
    # x,y,z = v
    # return math.sqrt(x*x + y*y + z*z)
    return math.sqrt(float(v[0])**2+float(v[1])**2)

def vector(b,e):
    # x,y,z = b
    # X,Y,Z = e
    # return (X-x, Y-y, Z-z)
    return (e[0]-b[0],e[1]-b[1])

def unit(v):
    # x,y,z = v
    # mag = length(v)
    # return (x/mag, y/mag, z/mag)
    mag = length(v)
    return (v[0]/mag, v[1]/mag)

def distance(p0,p1):
    return length(vector(p0,p1))

def scale(v,sc):
    # x,y,z = v
    # return (x * sc, y * sc, z * sc)
    return (v[0] * sc, v[1] * sc)

def add(v,w):
    # x,y,z = v
    # X,Y,Z = w
    # return (x+X, y+Y, z+Z)
    return (v[0]+w[0],v[1]+w[1])

def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return nearest, dist

#############################################################

###################### Main functions #######################

#############################################################

def build_VPA(netBound1, netBound2, margin, fileName):
    myFile = open(fileName, "r")
    index = 0
    sumo_VPA = []
    omnet_VPA = []
    while True:
        line1 = myFile.readline()
        if not line1: break  # EOF
        tokens = line1.split('.')
        x_string = tokens[2].split('=')
        x_string = x_string[1].replace(" ", "")
        x_string = x_string.rstrip()
        x = float(x_string)
        line2 = myFile.readline()
        if not line2: break  # EOF
        tokens = line2.split('.')
        y_string = tokens[2].split('=')
        y_string = y_string[1].replace(" ", "")
        y_string = y_string.rstrip()
        y = float(y_string)
        sumo_x, sumo_y = omnet2sumo(x, y, netBound1, netBound2, margin)
        omnet_VPA.append((x, y))
        sumo_VPA.append((sumo_x, sumo_y))
        # print "\nVPA Index:", index, "Omnet_Coord:", x, ";", y, "Sumo_Coord:", sumo_x, ";", sumo_y
        index += 1
    myFile.close()
    return sumo_VPA, omnet_VPA

def mapping_VPA(G, sumo_VPA, radio_range):
    mapping_list = []
    mapping_type = []
    ############ Try to map VPA to Nodes ##############
    for i, val in enumerate(sumo_VPA):
        map = []
        for j, val2 in enumerate(G.nodes(data=True)):
            result = False
            dist = -1
            node_pos = val2[1]["pos"]
            vpa_pos = (val[0], val[1])
            result, dist = node_inside_circle(node_pos, vpa_pos, radio_range)
            if result == True:
                map.append((val2[0],dist))
        if map == []:
            mapping_type.append(None)
        else:
            mapping_type.append("Node")
        mapping_list.append(map)
    ############ Try to map remaining VPA to Edges ##############
    for i, val in enumerate(sumo_VPA):
        if mapping_list[i] != []:
            continue
        map = []
        for j, val2 in enumerate(G.edges(data=True)):
            result = False
            dist = -1
            node_pos1 = G.node[val2[0]]["pos"]
            node_pos2 = G.node[val2[1]]["pos"]
            vpa_pos = (val[0], val[1])
            result, dist = edge_inside_circle(node_pos1, node_pos2, vpa_pos, radio_range)
            if result == True:
                map.append((val2[2]["label"],dist))
        if map != []:
            mapping_type[i] = "Edge"
        mapping_list[i] = map
        # mapping_list.append(map)
    return mapping_list, mapping_type

def node_inside_circle(node_pos, vpa_pos, r):
    sqr_dist = -1
    sqr_dist = (node_pos[0] - vpa_pos[0])**2 + (node_pos[1] - vpa_pos[1])**2
    if sqr_dist <= r**2:
        return True, math.sqrt(sqr_dist)
    else:
        return False, sqr_dist

def edge_inside_circle(node_pos1, node_pos2, vpa_pos, r):
    near_dist = -1
    start = (node_pos1[0], node_pos1[1])
    end = (node_pos2[0], node_pos2[1])
    pnt = (vpa_pos[0], vpa_pos[1])
    near_coord, near_dist = pnt2line(pnt,start,end)
    if near_dist <= r:
        return True, near_dist
    else:
        return False, near_dist

def sumo2omnet(sumo_x,sumo_y, netBound1, netBound2, margin):
    omnet_x = sumo_x - netBound1[0] + margin
    omnet_y = (netBound2[1] - netBound1[1]) - (sumo_y - netBound1[1]) + margin
    return omnet_x, omnet_y

def omnet2sumo(omnet_x,omnet_y, netBound1, netBound2, margin):
    sumo_x = omnet_x + netBound1[0] + margin
    sumo_y = (netBound2[1] - netBound1[1]) - (omnet_y - netBound1[1]) + margin
    return sumo_x, sumo_y