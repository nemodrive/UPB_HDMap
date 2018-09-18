import googlemaps
import requests
import responses
import json
import polyline_decoder as pd
import matplotlib.pyplot as plt
import numpy as np
import utm
from map_elements import *
import math
import numpy as np
from modules.map.proto import map_road_pb2
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2
from utils import distance
from shapely.geometry import LineString, Polygon, Point
from transforms import *

from datetime import datetime

SOLID_YELLOW = map_lane_pb2.LaneBoundaryType.SOLID_YELLOW
CURB = map_lane_pb2.LaneBoundaryType.CURB
DOTTED_WHITE = map_lane_pb2.LaneBoundaryType.DOTTED_WHITE
UNKNOWN = map_lane_pb2.LaneBoundaryType.DOTTED_WHITE

def make_lanes(points, map, id1, id2, roadid, border):
	road = map.road.add()
	road.id.id = str(roadid)
	section = road.section.add()
	section.id.id = "2"

	laneL = Lane(id1, map)
	laneR = Lane(id2, map)
	left_lane_xu = []
	left_lane_yu = []

	left_lane_x, right_lane_x, left_lane_y, right_lane_y = laneL.justGetMeTheLanes(points, 2.8, 0.1)
	
	densify = 4
	for i in range(densify):
		lx = []
		ly = []
		rx = []
		ry = []
		for j in range(len(left_lane_x)-1):
			a = ((left_lane_x[j] + left_lane_x[j + 1])/2)
			b = ((left_lane_y[j] + left_lane_y[j + 1])/2)
			c = ((right_lane_x[j] + right_lane_x[j + 1])/2)
			d = ((right_lane_y[j] + right_lane_y[j + 1])/2)
			lx.append(left_lane_x[j])
			lx.append(a)
			ly.append(left_lane_y[j])
			ly.append(b)
			rx.append(right_lane_x[j])
			rx.append(c)
			ry.append(right_lane_y[j])
			ry.append(d)
		lx.append(left_lane_x[len(left_lane_x) - 1])
		ly.append(left_lane_y[len(left_lane_y) - 1])
		rx.append(right_lane_x[len(right_lane_x) - 1])
		ry.append(right_lane_y[len(right_lane_y) - 1])

		left_lane_x, right_lane_x, left_lane_y, right_lane_y = [], [], [], []

		for j in range(len(lx)):
			left_lane_x.append(lx[j])
			left_lane_y.append(ly[j])
			right_lane_x.append(rx[j])
			right_lane_y.append(ry[j])

	for i in range(len(left_lane_x)-1, -1, -1):
		left_lane_xu.append(left_lane_x[i])
		left_lane_yu.append(left_lane_y[i])

	z = np.zeros(len(left_lane_x))
	left_points = []
	left_points = np.array(list(zip(left_lane_xu, left_lane_yu, z)))
	right_points = []
	right_points = np.array(list(zip(right_lane_x, right_lane_y, z)))
	laneL.add(left_points, 30, map_lane_pb2.Lane.NO_TURN, map_lane_pb2.Lane.CITY_DRIVING, map_lane_pb2.Lane.FORWARD, 2.8, 0.1)
	laneL.add_overlap(id1)
	
	if(border == True):
		laneL.set_left_lane_boundary_type(DOTTED_WHITE, False)
		laneL.set_right_lane_boundary_type(CURB, False)
	else:
		laneL.set_left_lane_boundary_type(UNKNOWN, True)
		laneL.set_right_lane_boundary_type(UNKNOWN, True)
	
	laneL.add_right_neighbor_forward(id2)
	laneR.add(right_points, 30, map_lane_pb2.Lane.NO_TURN, map_lane_pb2.Lane.CITY_DRIVING, map_lane_pb2.Lane.BACKWARD, 2.8, 0.1)
	laneR.add_overlap(id2)
	
	if(border == True):
		laneR.set_left_lane_boundary_type(DOTTED_WHITE, False)
		laneR.set_right_lane_boundary_type(CURB, False)
	else:
		laneR.set_left_lane_boundary_type(UNKNOWN, True)
		laneR.set_right_lane_boundary_type(UNKNOWN, True)
	
	laneR.add_right_neighbor_forward(id1)
	return laneL, laneR

def make_arr(arr1, arr2):
	for i in range(len(arr1)):
		arr2.append([utm.from_latlon(arr1[i][0], arr1[i][1])[0], utm.from_latlon(arr1[i][0], arr1[i][1])[1]])

# First junction

red1 = [[44.435387, 26.048221], [44.435378, 26.047810]]
orange1 = [[44.435374, 26.047697], [44.435363, 26.047149]]
aux1 = [[44.435422, 26.047754], [44.435563, 26.047750]]
aux2orange = [[44.435422, 26.047754], [44.435401, 26.047755], [44.435374, 26.047718], [44.435374, 26.047697]]
red2aux = [[44.435378, 26.047810], [44.435378, 26.047787], [44.435408, 26.047754], [44.435422, 26.047754]]
red2orange = [[44.435378, 26.047810], [44.435374, 26.047697]]

red11 = []
orange11 = []
aux11 = []
aux2orange1 = []
red2aux1 = []
red2orange1 = []

map = map_pb2.Map()

make_arr(red1, red11)
make_arr(orange1, orange11)
make_arr(aux1, aux11)
make_arr(aux2orange, aux2orange1)
make_arr(red2aux, red2aux1)
make_arr(red2orange, red2orange1)

lr1, rr1 = make_lanes(red11, map, 1, 2, 1, True)
lo1, ro1 = make_lanes(orange11, map, 3, 4, 2, True)
la1, ra1 = make_lanes(aux11, map, 5, 6, 3, True)
la2o, ra2o = make_lanes(aux2orange1, map, 7, 8, 4, False)
lr2a, rr2a = make_lanes(red2aux1, map, 9, 10, 5, False)
lr2o, rr2o = make_lanes(red2orange1, map, 11, 12, 6, False)

rr1.add_successor(rr2a.get_id())
rr1.add_successor(rr2o.get_id())
rr2o.add_successor(ro1.get_id())
rr2a.add_successor(ra1.get_id())
lo1.add_successor(lr2o.get_id())
lr2o.add_successor(lr1.get_id())

rr2a.add_predecessor(rr1.get_id())
rr2o.add_predecessor(rr1.get_id())
ro1.add_predecessor(rr2o.get_id())
ra1.add_predecessor(rr2a.get_id())
lr2o.add_predecessor(lo1.get_id())
lr1.add_predecessor(lr2o.get_id())

# Second junction

yellow1 = [[44.435358, 26.046953], [44.435347, 26.046270]]
aux2 = [[44.435416, 26.047041], [44.436024, 26.047018]]
orange2yellow = [[44.435363, 26.047149], [44.435358, 26.046953]]
orange2aux = [[44.435363, 26.047149], [44.435362, 26.047101], [44.435390, 26.047043], [44.435416, 26.047041]]
aux22yellow = [[44.435416, 26.047041], [44.435390, 26.047043], [44.435359, 26.046994], [44.435358, 26.046953]]

yellow11 = []
aux22 = []
orange2yellow1 = []
orange2aux1 = []
aux22yellow1 = []

make_arr(yellow1, yellow11)
make_arr(aux2, aux22)
make_arr(orange2yellow, orange2yellow1)
make_arr(orange2aux, orange2aux1)
make_arr(aux22yellow, aux22yellow1)

ly1, ry1 = make_lanes(yellow11, map, 13, 14, 7, True)
la2, ra2 = make_lanes(aux22, map, 15, 16, 8, True)
lo2y, ro2y = make_lanes(orange2yellow1, map, 17, 18, 9, False)
lo2a, ro2a = make_lanes(orange2aux1, map, 19, 20, 10, False)
la2y, ra2y = make_lanes(aux22yellow1, map, 21, 22, 11, False)

ro1.add_successor(ro2a.get_id())
ro1.add_successor(ro2y.get_id())
ro2y.add_successor(ry1.get_id())
ro2a.add_successor(ra2.get_id())
ly1.add_successor(lo2y.get_id())
lo2y.add_successor(lo1.get_id())

ro2a.add_predecessor(ro1.get_id())
ro2y.add_predecessor(ro1.get_id())
ry1.add_predecessor(ro2y.get_id())
ra2.add_predecessor(ro2a.get_id())
lo2y.add_predecessor(ly1.get_id())
lo1.add_predecessor(lo2y.get_id())

# Third junction

green1 = [[44.435342, 26.046073], [44.435329, 26.045288]]
aux3 = [[44.435298, 26.046167], [44.434739, 26.046185]]
yellow2aux3 = [[44.435347, 26.046270], [44.435347, 26.046208], [44.435315, 26.046166], [44.435298, 26.046167]]
aux32green1 = [[44.435298, 26.046167], [44.435315, 26.046166], [44.435344, 26.046113], [44.435342, 26.046073]]
yellow12green1 = [[44.435347, 26.046270], [44.435342, 26.046073]]

green11 = []
aux33 = []
yellow2aux31 = []
aux32green11 = []
yellow12green11 = []

make_arr(green1, green11)
make_arr(aux3, aux33)
make_arr(yellow2aux3, yellow2aux31)
make_arr(aux32green1, aux32green11)
make_arr(yellow12green1, yellow12green11)

lg1, rg1 = make_lanes(green11, map, 23, 24, 12, True)
la3, ra3 = make_lanes(aux33, map, 25, 26, 13, True)
ly2a, ry2a = make_lanes(yellow2aux31, map, 27, 28, 14, False)
la2g, ra2g = make_lanes(aux32green11, map, 29, 30, 15, False)
ly2g, ry2g = make_lanes(yellow12green11, map, 31, 32, 16, False)

ry1.add_successor(ry2g.get_id())
ry1.add_successor(ry2a.get_id())
ry2g.add_successor(rg1.get_id())
ry2a.add_successor(ra3.get_id())
lg1.add_successor(ly2g.get_id())
ly2g.add_successor(ly1.get_id())

ry2g.add_predecessor(ry1.get_id())
ry2a.add_predecessor(ry1.get_id())
rg1.add_predecessor(ry2g.get_id())
ra3.add_predecessor(ry2a.get_id())
ly2g.add_predecessor(lg1.get_id())
ly1.add_predecessor(ly2g.get_id())

# Fourth junction

blue1 = [[44.435388, 26.045198], [44.435727, 26.045189]]
aux4 = [[44.435308, 26.045199], [44.434763, 26.045216]]
aux5 = [[44.435331, 26.045181], [44.435325, 26.045066]]
green12aux4 = [[44.435329, 26.045288], [44.435329, 26.045249], [44.435300, 26.045200], [44.435280, 26.045200]]
green12aux5 = [[44.435329, 26.045288], [44.435331, 26.045181]]
green12blue1 = [[44.435329, 26.045288], [44.435329, 26.045249], [44.435360, 26.045198], [44.435388, 26.045198]]

blue11 = []
aux44 = []
aux55 = []
green12aux41 = []
green12aux51 = []
green12blue11 = []

make_arr(blue1, blue11)
make_arr(aux4, aux44)
make_arr(aux5, aux55)
make_arr(green12aux4, green12aux41)
make_arr(green12aux5, green12aux51)
make_arr(green12blue1, green12blue11)

lb1, rb1 = make_lanes(blue11, map, 33, 34, 17, True)
la4, ra4 = make_lanes(aux44, map, 35, 36, 18, True)
la5, ra5 = make_lanes(aux55, map, 37, 38, 19, True)
lg2a4, rg2a4 = make_lanes(green12aux41, map, 39, 40, 20, False)
lg2a5, rg2a5 = make_lanes(green12aux51, map, 41, 42, 21, False)
lg2b, rg2b = make_lanes(green12blue11, map, 43, 44, 22, False)

rg1.add_successor(rg2a4.get_id())
rg1.add_successor(rg2b.get_id())
rg1.add_successor(rg2a5.get_id())
rg2b.add_successor(rb1.get_id())
rg2a5.add_successor(ra5.get_id())
rg2a4.add_successor(ra4.get_id())
lb1.add_successor(lg2b.get_id())
lg2b.add_successor(lg1.get_id())

rg2a4.add_predecessor(rg1.get_id())
rg2b.add_predecessor(rg1.get_id())
rg2a5.add_predecessor(rg1.get_id())
rb1.add_predecessor(rg2b.get_id())
ra5.add_predecessor(rg2a5.get_id())
ra4.add_predecessor(rg2a4.get_id())
lg2b.add_predecessor(lb1.get_id())
lg1.add_predecessor(lg2b.get_id())

# Fifth junction

purple1 = [[44.435869, 26.045032], [44.436413, 26.045021]]
aux6 = [[44.435803, 26.045225], [44.435846, 26.045312]]
aux7 = [[44.435740, 26.045038], [44.435551, 26.045042]]
blue12aux6 = [[44.435727, 26.045189], [44.435746, 26.045188], [44.435783, 26.045188], [44.435803, 26.045225]]
blue12aux7 = [[44.435727, 26.045189], [44.435746, 26.045188], [44.435785, 26.045145], [44.435784, 26.045079], [44.435758, 26.045037], [44.435740, 26.045038]]
blue12purple1 = [[44.435727, 26.045189], [44.435746, 26.045188], [44.435785, 26.045145], [44.435784, 26.045059], [44.435811, 26.045034],[44.435869, 26.045032]]

purple11 = []
aux66 = []
aux77 = []
blue12aux61 = []
blue12aux71 = []
blue12purple11 = []

make_arr(purple1, purple11)
make_arr(aux6, aux66)
make_arr(aux7, aux77)
make_arr(blue12aux6, blue12aux61)
make_arr(blue12aux7, blue12aux71)
make_arr(blue12purple1, blue12purple11)

lp1, rp1 = make_lanes(purple11, map, 45, 46, 23, True)
la6, ra6 = make_lanes(aux66, map, 47, 48, 24, True)
la7, ra7 = make_lanes(aux77, map, 49, 50, 25, True)
lb2a6, rb2a6 = make_lanes(blue12aux61, map, 51, 52, 26, False)
lb2a7, rb2a7 = make_lanes(blue12aux71, map, 53, 54, 27, False)
lb2p, rb2p = make_lanes(blue12purple11, map, 55, 56, 28, False)

rb1.add_successor(rb2a6.get_id())
rb1.add_successor(rb2p.get_id())
rb1.add_successor(rb2a7.get_id())
rb2a6.add_successor(ra6.get_id())
rb2p.add_successor(rp1.get_id())
rb2a7.add_successor(ra7.get_id())
lp1.add_successor(lb2p.get_id())
lb2p.add_successor(lb1.get_id())

rb2a6.add_predecessor(rb1.get_id())
rb2p.add_predecessor(rb1.get_id())
rb2a7.add_predecessor(rb1.get_id())
ra6.add_predecessor(rb2a6.get_id())
rp1.add_predecessor(rb2p.get_id())
ra7.add_predecessor(rb2a7.get_id())
lb1.add_predecessor(lb2p.get_id())
lb2p.add_predecessor(lp1.get_id())

# Sixth junction

red2 = [[44.436466, 26.045017], [44.436815, 26.045006], [44.436918, 26.044995], [44.436985, 26.044974], [44.437030, 26.044957], [44.437256, 26.044775], [44.437314, 26.044733], [44.437360, 26.044718], [44.437417, 26.044717], [44.437422, 26.044719]]
aux8 = [[44.436443, 26.045050], [44.436339, 26.045416]]
purple12red2 = [[44.436421, 26.045018], [44.436482, 26.045015]]
purple12aux8 = [[44.436414, 26.045020], [44.436423, 26.045021], [44.436438, 26.045065], [44.436437, 26.045073]]
aux82red2 = [[44.436437, 26.045073], [44.436450, 26.045019], [44.436507, 26.045015]]

red21 = []
aux88 = []
purple12red21 = []
purple12aux81 = []
aux82red21 = []

make_arr(red2, red21)
make_arr(aux8, aux88)
make_arr(purple12red2, purple12red21)
make_arr(purple12aux8, purple12aux81)
make_arr(aux82red2, aux82red21)

lr2, rr2 = make_lanes(red21, map, 57, 58, 29, True)
la8, ra8 = make_lanes(aux88, map, 59, 60, 30, True)
lp2r, rp2r = make_lanes(purple12red21, map, 61, 62, 31, False)
lp2a, rp2a = make_lanes(purple12aux81, map, 63, 64, 32, False)
#la2r, ra2r = make_lanes(aux82red21, map, 65, 66, 33)

rp1.add_successor(rp2a.get_id())
rp1.add_successor(rp2r.get_id())
rp2a.add_successor(ra8.get_id())
rp2r.add_successor(rr2.get_id())
lr2.add_successor(lp2r.get_id())
lp2r.add_successor(lp1.get_id())

rp2a.add_predecessor(rp1.get_id())
rp2r.add_predecessor(rp1.get_id())
ra8.add_predecessor(rp2a.get_id())
rr2.add_predecessor(rp2r.get_id())
lp1.add_predecessor(lp2r.get_id())
lp2r.add_predecessor(lr2.get_id())

# Seventh junction

orange2 = [[44.437480, 26.044820], [44.437465, 26.046649]]
aux9 = [[44.437494, 26.044660], [44.437641, 26.043885]]
red22orange2 = [[44.437422, 26.044719], [44.437447, 26.044726], [44.437479, 26.044780], [44.437480, 26.044820]]
red22aux9 = [[44.437422, 26.044719], [44.437454, 26.044722], [44.437487, 26.044694], [44.437494, 26.044660]]
aux92orange2 = [[44.437494, 26.044660], [44.437479, 26.044732], [44.437479, 26.044780]]

orange21 = []
aux99 = []
red22orange21 = []
red22aux91 = []
aux92orange21 = []

make_arr(orange2, orange21)
make_arr(aux9, aux99)
make_arr(red22orange2, red22orange21)
make_arr(red22aux9, red22aux91)
make_arr(aux92orange2, aux92orange21)

lo2, ro2 = make_lanes(orange21, map, 67, 68, 34, True)
la9, ra9 = make_lanes(aux99, map, 69, 70, 35, True)
lr2o, rr2o = make_lanes(red22orange21, map, 71, 72, 36, False)
lr2a, rr2a = make_lanes(red22aux91, map, 73, 74, 37, False)
la2o, ra2o = make_lanes(aux92orange21, map, 75, 76, 38, False)

rr2.add_successor(rr2o.get_id())
rr2.add_successor(rr2a.get_id())
rr2o.add_successor(ro2.get_id())
rr2a.add_successor(ra9.get_id())
lo2.add_successor(lr2o.get_id())
lr2o.add_successor(lr2.get_id())

rr2o.add_predecessor(rr2.get_id())
rr2a.add_predecessor(rr2.get_id())
ro2.add_predecessor(rr2o.get_id())
ra9.add_predecessor(rr2a.get_id())
lr2.add_predecessor(lr2o.get_id())
lr2o.add_predecessor(lo2.get_id())

# Eight junction

aux10 = [[44.437476, 26.046669], [44.437514, 26.046719]]
yellow2 = [[44.437462, 26.046685], [44.437253, 26.047806]]
orange22yellow2 = [[44.437466, 26.046615], [44.437466, 26.046651], [44.437463, 26.046684], [44.437457, 26.046725]]
orange22aux10 = [[44.437466, 26.046621], [44.437466, 26.046655], [44.437475, 26.046666], [44.437492, 26.046688]]
#aux102yellow2

aux101 = []
yellow21 = []
orange22yellow21 = []
orange22aux101 = []

for i in range(len(aux10)):
	aux101.append([utm.from_latlon(aux10[i][0], aux10[i][1])[0], utm.from_latlon(aux10[i][0], aux10[i][1])[1]])
	yellow21.append([utm.from_latlon(yellow2[i][0], yellow2[i][1])[0], utm.from_latlon(yellow2[i][0], yellow2[i][1])[1]])

for i in range(len(orange22yellow2)):
	orange22yellow21.append([utm.from_latlon(orange22yellow2[i][0], orange22yellow2[i][1])[0], utm.from_latlon(orange22yellow2[i][0], orange22yellow2[i][1])[1]])
	orange22aux101.append([utm.from_latlon(orange22aux10[i][0], orange22aux10[i][1])[0], utm.from_latlon(orange22aux10[i][0], orange22aux10[i][1])[1]])

la10, ra10 = make_lanes(aux101, map, 77, 78, 39, True)
ly2, ry2 = make_lanes(yellow21, map, 79, 80, 40, True)
lo2y, ro2y = make_lanes(orange22yellow21, map, 81, 82, 41, False)
lo2a, ro2a = make_lanes(orange22aux101, map, 83, 84, 42, False)

ro2.add_successor(ro2y.get_id())
ro2.add_successor(ro2a.get_id())
ro2a.add_successor(ra10.get_id())
ro2y.add_successor(ry2.get_id())
ly2.add_successor(lo2y.get_id())
lo2y.add_successor(lo2.get_id())

ro2y.add_predecessor(ro2.get_id())
ro2a.add_predecessor(ro2.get_id())
ra10.add_predecessor(ro2a.get_id())
ry2.add_predecessor(ro2y.get_id())
lo2.add_predecessor(lo2y.get_id())
lo2y.add_predecessor(ly2.get_id())

# Ninth junction

aux11 = [[44.437217, 26.047842], [44.436794, 26.047682]]
green2 = [[44.437242, 26.047892], [44.437224, 26.048105], [44.437069, 26.048948]]
yellow22aux11 = [[44.437253, 26.047806], [44.437248, 26.047833], [44.437233, 26.047847], [44.437217, 26.047842]]
aux112green2 = [[44.437217, 26.047842], [44.437235, 26.047849], [44.437243, 26.047869], [44.437242, 26.047892]]
yellow22green2 = [[44.437253, 26.047806], [44.437243, 26.047852], [44.437242, 26.047892]]

aux111 = []
green21 = []
yellow22aux111 = []
aux112green21 = []
yellow22green21 = []

for i in range(len(aux11)):
	aux111.append([utm.from_latlon(aux11[i][0], aux11[i][1])[0], utm.from_latlon(aux11[i][0], aux11[i][1])[1]])

for i in range(len(green2)):
	green21.append([utm.from_latlon(green2[i][0], green2[i][1])[0], utm.from_latlon(green2[i][0], green2[i][1])[1]])
	yellow22green21.append([utm.from_latlon(yellow22green2[i][0], yellow22green2[i][1])[0], utm.from_latlon(yellow22green2[i][0], yellow22green2[i][1])[1]])

for i in range(len(yellow22aux11)):
	yellow22aux111.append([utm.from_latlon(yellow22aux11[i][0], yellow22aux11[i][1])[0], utm.from_latlon(yellow22aux11[i][0], yellow22aux11[i][1])[1]])
	aux112green21.append([utm.from_latlon(aux112green2[i][0], aux112green2[i][1])[0], utm.from_latlon(aux112green2[i][0], aux112green2[i][1])[1]])

la11, ra11 = make_lanes(aux111, map, 85, 86, 43, True)
lg2, rg2 = make_lanes(green21, map, 87, 88, 44, True)
ly2a, ry2a = make_lanes(yellow22aux111, map, 89, 90, 45, False)
la2g, ra2g = make_lanes(aux112green21, map, 91, 92, 46, False)
ly2g, ry2g = make_lanes(yellow22green21, map, 93, 94, 47, False)

ry2.add_successor(ry2a.get_id())
ry2.add_successor(ry2g.get_id())
ry2a.add_successor(ra11.get_id())
ry2g.add_successor(rg2.get_id())
lg2.add_successor(ly2g.get_id())
ly2g.add_successor(ly2.get_id())

ry2a.add_predecessor(ry2.get_id())
ry2g.add_predecessor(ry2.get_id())
ra11.add_predecessor(ry2a.get_id())
rg2.add_predecessor(ry2g.get_id())
ly2.add_predecessor(ly2g.get_id())
ly2g.add_predecessor(lg2.get_id())

# Tenth junction

blue2 = [[44.437026, 26.049027], [44.437004, 26.049022], [44.436964, 26.049024], [44.436934, 26.049039], [44.436914, 26.049058], [44.436880, 26.049105], [44.436857, 26.049162], [44.436677, 26.050088], [44.436646, 26.050175], [44.436593, 26.050263], [44.436541, 26.050307], [44.436498, 26.050327], [44.436461, 26.050332], [44.436422, 26.050325], [44.436387, 26.050310], [44.436354, 26.050280], [44.436279, 26.050198], [44.436046, 26.049713], [44.436003, 26.049586], [44.435980, 26.049461], [44.435973, 26.049331], [44.436001, 26.048857]]
aux12 = [[44.437092, 26.049043], [44.437318, 26.049130]]
green22aux12 = [[44.437069, 26.048948], [44.437061, 26.048998], [44.437072, 26.049040], [44.437092, 26.049043]]
green22blue2 = [[44.437069, 26.048948], [44.437055, 26.049017], [44.437040, 26.049029], [44.437026, 26.049027]]
aux122blue2 = [[44.437092, 26.049043], [44.437053, 26.049033], [44.437026, 26.049027]]

blue21 = []
aux121 = []
green22aux121 = []
green22blue21 = []
aux122blue21 = []

make_arr(blue2, blue21)
make_arr(aux12, aux121)
make_arr(green22aux12, green22aux121)
make_arr(green22blue2, green22blue21)
make_arr(aux122blue2, aux122blue21)

lb2, rb2 = make_lanes(blue21, map, 95, 96, 48, True)
la12, ra12 = make_lanes(aux121, map, 97, 98, 49, True)
lg2a, rg2a = make_lanes(green22aux121, map, 99, 100, 50, False)
lg2b, rg2b = make_lanes(green22blue21, map, 101, 102, 51, False)
la2b, ra2b = make_lanes(aux122blue21, map, 103, 104, 52, False)

rg2.add_successor(rg2b.get_id())
rg2.add_successor(rg2a.get_id())
rg2b.add_successor(rb2.get_id())
rg2a.add_successor(ra12.get_id())
lb2.add_successor(lg2b.get_id())
lg2b.add_successor(lg2.get_id())

rg2b.add_predecessor(rg2.get_id())
rg2a.add_predecessor(rg2.get_id())
rb2.add_predecessor(rg2b.get_id())
ra12.add_predecessor(rg2a.get_id())
lg2.add_predecessor(lg2b.get_id())
lg2b.add_predecessor(lb2.get_id())

# Eleventh junction

aux13 = [[44.436006, 26.048822], [44.436159, 26.047991]]
purple2 = [[44.435994, 26.048833], [44.435732, 26.048713], [44.435676, 26.048671]]
blue22aux13 = [[44.436001, 26.048875], [44.436004, 26.048839], [44.436015, 26.048784]]
blue22purple2 = [[44.436000, 26.048884], [44.436003, 26.048854], [44.435995, 26.048833], [44.435980, 26.048827]]

aux131 = []
purple21 = []
blue22aux131 = []
blue22purple21 = []

make_arr(aux13, aux131)
make_arr(purple2, purple21)
make_arr(blue22aux13, blue22aux131)
make_arr(blue22purple2, blue22purple21)

la13, ra13 = make_lanes(aux131, map, 105, 106, 53, True)
lp2, rp2 = make_lanes(purple21, map, 107, 108, 54, True)
lb2a, rb2a = make_lanes(blue22aux131, map, 109, 110, 55, False)
lb2p, rb2p = make_lanes(blue22purple21, map, 111, 112, 56, False)

rb2.add_successor(rb2p.get_id())
rb2.add_successor(rb2a.get_id())
rb2p.add_successor(rp2.get_id())
rb2a.add_successor(ra13.get_id())
lp2.add_successor(lb2p.get_id())
lb2p.add_successor(lb2.get_id())

rb2p.add_predecessor(rb2.get_id())
rb2a.add_predecessor(rb2.get_id())
rp2.add_predecessor(rb2p.get_id())
ra13.add_predecessor(rb2a.get_id())
lb2.add_predecessor(lb2p.get_id())
lb2p.add_predecessor(lp2.get_id())

# Twelvth junction

red3 = [[44.435654, 26.048649], [44.435586, 26.048544], [44.435556, 26.048525]]
aux14 = [[44.435650, 26.048669], [44.435535, 26.048672]]
purple22aux14 = [[44.435676, 26.048671], [44.435663, 26.048667], [44.435640, 26.048670]]
purple22red3 = [[44.435676, 26.048671], [44.435661, 26.048660], [44.435654, 26.048649]]

red31 = []
aux141 = []
purple22aux141 = []
purple22red31 = []

make_arr(red3, red31)
make_arr(aux14, aux141)
make_arr(purple22aux14, purple22aux141)
make_arr(purple22red3, purple22red31)

lr3, rr3 = make_lanes(red31, map, 113, 114, 57, True)
la14, ra14 = make_lanes(aux141, map, 115, 116, 58, True)
lp2a, rp2a = make_lanes(purple22aux141, map, 117, 118, 59, False)
lp2r, rp2r = make_lanes(purple22red31, map, 119, 120, 60, False)

rp2.add_successor(rp2r.get_id())
rp2.add_successor(rp2a.get_id())
rp2r.add_successor(rr3.get_id())
rp2a.add_successor(ra14.get_id())
lr3.add_successor(lp2r.get_id())
lp2r.add_successor(lp2.get_id())

rp2r.add_predecessor(rp2.get_id())
rp2a.add_predecessor(rp2.get_id())
rr3.add_predecessor(rp2r.get_id())
ra14.add_predecessor(rp2a.get_id())
lp2.add_predecessor(lp2r.get_id())
lp2r.add_predecessor(lr3.get_id())

# 13 junction

aux15 = [[44.435520, 26.048585], [44.435525, 26.048663]]
aux16 = [[44.435507, 26.048523], [44.435404, 26.048523]]
orange3 = [[44.435516, 26.048493], [44.435452, 26.048451], [44.435401, 26.048409]]
red32orange3 = [[44.435556, 26.048525], [44.435516, 26.048493]]
red32aux15 = [[44.435556, 26.048525], [44.435534, 26.048524], [44.435519, 26.048551], [44.435520, 26.048585]]
red32aux16 = [[44.435556, 26.048525], [44.435507, 26.048523]]

aux151 = []
aux161 = []
orange31 = []
red32aux151 = []
red32aux161 = []
red32orange31 = []

make_arr(aux15, aux151)
make_arr(aux16, aux161)
make_arr(orange3, orange31)
make_arr(red32aux15, red32aux151)
make_arr(red32aux16, red32aux161)
make_arr(red32orange3, red32orange31)

la15, ra15 = make_lanes(aux151, map, 121, 122, 61, True)
la16, ra16 = make_lanes(aux161, map, 123, 124, 62, True)
lo3, ro3 = make_lanes(orange31, map, 125, 126, 63, True)
lr2a15, rr2a15 = make_lanes(red32aux151, map, 127, 128, 64, False)
lr2a16, rr2a16 = make_lanes(red32aux161, map, 129, 130, 65, False)
lr2o, rr2o = make_lanes(red32orange31, map, 131, 132, 66, False)

rr3.add_successor(rr2a15.get_id())
rr2a15.add_successor(ra15.get_id())
rr3.add_successor(rr2a16.get_id())
rr2a16.add_successor(ra16.get_id())
rr3.add_successor(rr2o.get_id())
rr2o.add_successor(ro3.get_id())
lo3.add_successor(lr2o.get_id())
lr2o.add_successor(lr3.get_id())

rr2a15.add_predecessor(rr3.get_id())
ra15.add_predecessor(rr2a15.get_id())
rr2a16.add_predecessor(rr3.get_id())
ra16.add_predecessor(rr2a16.get_id())
rr2o.add_predecessor(rr3.get_id())
ro3.add_predecessor(rr2o.get_id())
lr3.add_predecessor(lr2o.get_id())
lr2o.add_predecessor(lo3.get_id())

# 14 junction

aux17 = [[44.435392, 26.048426], [44.435393, 26.048509]]
yellow3 = [[44.435391, 26.048392], [44.435387, 26.048255]]
orange32yellow3 = [[44.435424, 26.048427], [44.435401, 26.048411], [44.435390, 26.048394], [44.435391, 26.048368]]
orange32aux17 = [[44.435424, 26.048427], [44.435401, 26.048411],[44.435391, 26.048423], [44.435394, 26.048450]]
aux172yellow3 = [[44.435394, 26.048450], [44.435391, 26.048392]]

aux171 = []
yellow31 = []
orange32yellow31 = []
orange32aux171 = []
aux172yellow31 = []

make_arr(aux17, aux171)
make_arr(yellow3, yellow31)
make_arr(orange32yellow3, orange32yellow31)
make_arr(orange32aux17, orange32aux171)
make_arr(aux172yellow3, aux172yellow31)

la17, ra17 = make_lanes(aux171, map, 141, 142, 66, True)
ly3, ry3 = make_lanes(yellow31, map, 133, 134, 67, True)
lo2y, ro2y = make_lanes(orange32yellow31, map, 135, 136, 68, False)
#lo2a, ro2a = make_lanes(orange22aux171, map, 137, 138, 69)
la2y, ra2y = make_lanes(aux172yellow31, map, 139, 140, 70, False)

ro3.add_successor(ro2y.get_id())
#ro3.add_successor(ro2a.get_id())
ro2y.add_successor(ry3.get_id())
#ro2a.add_successor(ra17.get_id())
ly3.add_successor(lo2y.get_id())
lo2y.add_successor(lo3.get_id())

ro2y.add_predecessor(ro3.get_id())
#ro2a.add_predecessor(ro3.get_id())
ry3.add_predecessor(ro2y.get_id())
#ra17.add_predecessor(ro2a.get_id())
lo3.add_predecessor(lo2y.get_id())
lo2y.add_predecessor(ly3.get_id())

# 15 junction

aux18 = [[44.435374, 26.048238], [44.434835, 26.048258]]
yellow32red1 = [[44.435387, 26.048266], [44.435386, 26.048217]]
yellow32aux18 = [[44.435388, 26.048291], [44.435387, 26.048252], [44.435376, 26.048237], [44.435361, 26.048240]]
aux182red1 = [[44.435361, 26.048240], [44.435376, 26.048237], [44.435386, 26.048219], [44.435386, 26.048188]]

aux181 = []
yellow32red11 = []
yellow32aux181 = []
aux182red11 = []

make_arr(aux18, aux181)
make_arr(yellow32red1, yellow32red11)
make_arr(yellow32aux18, yellow32aux181)
#make_arr(aux182red1, aux182red11)

la18, ra18 = make_lanes(aux181, map, 147, 148, 71, True)
ly2r, ry2r = make_lanes(yellow32red11, map, 143, 144, 72, False)
ly2a, ry2a = make_lanes(yellow32aux181, map, 145, 146, 73, False)
#make_lanes(aux182red11, map, 147, 148, 74)

ry3.add_successor(ry2r.get_id())
ry3.add_successor(ry2a.get_id())
ry2r.add_successor(rr1.get_id())
ry2a.add_successor(ra18.get_id())
lr1.add_successor(ly2r.get_id())
ly2r.add_successor(ly3.get_id())
ry2r.add_predecessor(ry3.get_id()) 
ry2a.add_predecessor(ry3.get_id())
rr1.add_predecessor(ry2r.get_id()) 
ra18.add_predecessor(ry2a.get_id())
ly2r.add_predecessor(lr1.get_id()) 
ly3.add_predecessor(ly2r.get_id())
#plt.show()

map_file = open('try.txt', 'w')
map_file.write(str(map))
map_file.close()