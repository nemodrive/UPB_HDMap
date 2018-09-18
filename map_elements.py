#import cv2
import math
import numpy as np
from modules.map.proto import map_road_pb2
from modules.map.proto import map_lane_pb2
from modules.map.proto import map_pb2
from utils import distance
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon, Point


class RoadObject(object):

    def __init__(self, id):
        self._id = id

    def get_id(self):
        return self._id


class Lane(RoadObject):

    def __init__(self, id, map):
        super(Lane, self).__init__(id)
        self.lane = map.lane.add()
        self.lane.id.id = str(id)

    def get_polygon(self):
        return self._polygon

    def set_length(self, points):
        length = 0
        for i in range(1, len(points)):
            length += distance(points[i - 1], points[i])
        self.lane.length = length
        return length

    def set_speed_limit(self, speed_limit):
        self.lane.speed_limit = speed_limit

    def add_overlap(self, id):
        self.lane.overlap_id.add().id = str(id)

    def add_predecessor(self, id):
        self.lane.predecessor_id.add().id = str(id)

    def add_successor(self, id):
        self.lane.successor_id.add().id = str(id)

    def add_left_neighbor_forward(self, id):
        self.lane.left_neighbor_forward_lane_id.add().id = str(id)

    def add_right_neighbor_forward(self, id):
        self.lane.right_neighbor_forward_lane_id.add().id = str(id)

    def set_type(self, type):
        self.lane.type = type

    def set_turn(self, turn):
        self.lane.turn = turn

    def set_direction(self, direction):
        self.lane.direction = direction

    def set_left_lane_boundary_type(self, type, virtual):
        # boundary type params
        boundary_type = self.lane.left_boundary.boundary_type.add()
        boundary_type.types.append(type)
        self.lane.left_boundary.virtual = virtual  # bool

    def set_right_lane_boundary_type(self, type, virtual):
        # boundary type params
        boundary_type = self.lane.right_boundary.boundary_type.add()
        boundary_type.types.append(type)
        self.lane.right_boundary.virtual = virtual  # bool

    def add_left_lane_boundary(self, x, y, z, heading):  # removed s
        # curve segment params
        left_boundary = self.lane.left_boundary.curve.segment.add()
        left_boundary.heading = heading
        left_boundary.start_position.x = x
        left_boundary.start_position.y = y
        left_boundary.start_position.z = z
        left_boundary.s = 0
        return left_boundary

    def add_right_lane_boundary(self, x, y, z, heading):  # removed s
        # curve segment params
        right_boundary = self.lane.right_boundary.curve.segment.add()
        right_boundary.heading = heading
        right_boundary.start_position.x = x
        right_boundary.start_position.y = y
        right_boundary.start_position.z = z
        right_boundary.s = 0
        return right_boundary

    def add_central_curve(self, x, y, z, heading):  # removed s, removed length
        central_curve = self.lane.central_curve.segment.add()
        central_curve.heading = heading
        central_curve.start_position.x = x
        central_curve.start_position.y = y
        central_curve.start_position.z = z
        central_curve.s = 0
        return central_curve

    def add_left_sample_assoc(self, s, width):
        ls = self.lane.left_sample.add()
        ls.s = s
        ls.width = width

    def add_right_sample_assoc(self, s, width):
        rs = self.lane.right_sample.add()
        rs.s = s
        rs.width = width

    def add_left_road_sample_assoc(self, s, width):
        lrs = self.lane.left_road_sample.add()
        lrs.s = s
        lrs.width = width

    def add_right_road_sample_assoc(self, s, width):
        rrs = self.lane.right_road_sample.add()
        rrs.s = s
        rrs.width = width

    def add_left_forward_neighbor(self, id):
        self.lane.left_neighbor_forward_lane_id.add().id = str(id)

    def add_left_reverse_neighbor(self, id):
        self.lane.left_neighbor_reverse_lane_id.add().id = str(id)

    def add_right_forward_neighbor(self, id):
        self.lane.right_neighbor_forward_lane_id.add().id = str(id)

    def add_right_reverse_neighbor(self, id):
        self.lane.right_neighbor_reverse_lane_id.add().id = str(id)

    def lane_sampling(self, points, width, left_boundary, right_boundary, central_curve, thresh, do_sampling=False):
        length = len(points)
        self.left_poly = []
        self.right_poly = []

        # variables used to compute central line and boundaries length
        c_dist = 0
        cx, cy = (central_curve.start_position.x, central_curve.start_position.y)
        l_dist = 0
        lx, ly = (left_boundary.start_position.x, central_curve.start_position.y)
        r_dist = 0
        rx, ry = (right_boundary.start_position.x, central_curve.start_position.y)
        left_lane_x = []
        right_lane_x = []
        left_lane_y = []
        right_lane_y = []

        for i in range(length):
            if i > 1:
                p = Point(points[i-1])
                p2 = Point(points[i])
                if do_sampling:
                    if (p.x == p2.x or p.y == p2.y) and i != length - 1 and i % 10 != 0:
                        continue
                    elif i % 3 != 0 and i != length - 1:
                        continue

            if i < length - 1:
                p = Point(points[i])
                p2 = Point(points[i + 1])
            else:
                p = Point(points[i - 1])
                p2 = Point(points[i])
            distance = width / 2.0
            lp, rp = self.convert(p, p2, distance, use_first=(i != length - 1))

            if(i > 0 and abs(lp[0] - prevlx) < thresh and abs(lp[1] - prevly) < thresh):
                if(i > 0 and abs(rp[0] - prevrx) < thresh and abs(rp[1] - prevry) < thresh):
                    continue
                else:
                    right_bound_point = right_boundary.line_segment.point.add()
                    right_bound_point.x = rp[0]
                    right_bound_point.y = rp[1]
                    right_bound_point.z = points[i][2]
                    right_lane_x.append(rp[0])
                    right_lane_y.append(rp[1])


                    r_dist += math.sqrt(math.pow(rp[0] - rx, 2) + math.pow(rp[1] - ry, 2))
                    rx = rp[0]
                    ry = rp[1]
            else:
                if(i > 0 and abs(rp[0] - prevrx) < thresh and abs(rp[1] - prevry) < thresh):
                    continue
                else:
                    right_bound_point = right_boundary.line_segment.point.add()
                    right_bound_point.x = rp[0]
                    right_bound_point.y = rp[1]
                    right_bound_point.z = points[i][2]
                    right_lane_x.append(rp[0])
                    right_lane_y.append(rp[1])

                    r_dist += math.sqrt(math.pow(rp[0] - rx, 2) + math.pow(rp[1] - ry, 2))
                    rx = rp[0]
                    ry = rp[1]

                left_bound_point = left_boundary.line_segment.point.add()
                left_bound_point.x = lp[0]
                left_bound_point.y = lp[1]
                left_bound_point.z = points[i][2]
                left_lane_x.append(lp[0])
                left_lane_y.append(lp[1])

                l_dist += math.sqrt(math.pow(lp[0] - lx, 2) + math.pow(lp[1] - ly, 2))
                lx = lp[0]
                ly = lp[1]

            # plt.plot(rp[0], rp[1], 'ro')
            # plt.plot(lp[0], lp[1], 'bo')
            
            prevrx, prevry, prevlx, prevly = rp[0], rp[1], lp[0], lp[1]

            ####

            central_point = central_curve.line_segment.point.add()
            if i < length - 1:
                central_point.x = p.x
                central_point.y = p.y
                central_point.z = points[i][2]
            else:
                central_point.x = p2.x
                central_point.y = p2.y
                central_point.z = points[i-1][2]
            c_dist += math.sqrt(math.pow(central_point.x - cx, 2) + math.pow(central_point.y - cy, 2))
            cx = central_point.x
            cy = central_point.y

            if i < length - 1:
                self.left_poly.append(np.array(lp))
                self.right_poly.append(np.array(rp))

            # Get distance from lane start
            if i > 0:
                line = LineString(points[:i + 1])
                dist = line.length
            else:
                dist = 0

            left_sample = self.lane.left_sample.add()
            left_sample.s = dist
            left_sample.width = width / 2.0

            right_sample = self.lane.right_sample.add()
            right_sample.s = dist
            right_sample.width = width / 2.0

        central_curve.length = c_dist
        left_boundary.length = l_dist
        right_boundary.length = r_dist
        
        self._polygon = np.array(self.left_poly + list(reversed(self.right_poly)))
        return left_lane_x, right_lane_x, left_lane_y, right_lane_y

    def justGetMeTheLanes(self, points, width, thresh):
        length = len(points)
        left_lane_x = []
        right_lane_x = []
        left_lane_y = []
        right_lane_y = []
        for i in range(length):
            if i > 1:
                p = Point(points[i-1])
                p2 = Point(points[i])
            if i < length - 1:
                p = Point(points[i])
                p2 = Point(points[i + 1])
            else:
                p = Point(points[i - 1])
                p2 = Point(points[i])
            distance = width / 2.0
            lp, rp = self.convert(p, p2, distance, use_first=(i != length - 1))

            if(i > 0 and abs(lp[0] - prevlx) < thresh and abs(lp[1] - prevly) < thresh):
                if(i > 0 and abs(rp[0] - prevrx) < thresh and abs(rp[1] - prevry) < thresh):
                    continue
                else:
                    right_lane_x.append(rp[0])
                    right_lane_y.append(rp[1])
            else:
                if(i > 0 and abs(rp[0] - prevrx) < thresh and abs(rp[1] - prevry) < thresh):
                    continue
                else:
                    right_lane_x.append(rp[0])
                    right_lane_y.append(rp[1])
                left_lane_x.append(lp[0])
                left_lane_y.append(lp[1])
            prevrx, prevry, prevlx, prevly = rp[0], rp[1], lp[0], lp[1]
        return left_lane_x, right_lane_x, left_lane_y, right_lane_y

    def convert(self, p, p2, distance, use_first=True):
        delta_y = p2.y - p.y
        delta_x = p2.x - p.x

        if use_first:
            point = p
        else:
            point = p2

        left_angle = math.atan2(delta_y, delta_x) + math.pi / 2.0
        right_angle = math.atan2(delta_y, delta_x) - math.pi / 2.0

        lp = []
        lp.append(point.x + (math.cos(left_angle) * distance))
        lp.append(point.y + (math.sin(left_angle) * distance))

        rp = []
        rp.append(point.x + (math.cos(right_angle) * distance))
        rp.append(point.y + (math.sin(right_angle) * distance))
        return lp, rp

    def add(self, points, speed_limit, lane_turn, lane_type, direction, width, thresh):
        self.set_length(points)
        self.set_speed_limit(speed_limit)
        self.set_turn(lane_turn)
        self.set_type(lane_type)
        self.set_direction(direction)

        path = LineString(points)
        p = path.interpolate(0)
        p2 = path.interpolate(0.5)
        distance = width / 2.0
        lp, rp = self.convert(p, p2, distance)

        central_curve = self.add_central_curve(points[0][0], points[0][1], points[0][2], 0)
        left_boundary = self.add_left_lane_boundary(lp[0], lp[1], points[0][2], 0)
        right_boundary = self.add_right_lane_boundary(rp[0], rp[1], points[0][2], 0)

        left_lane_x = []
        right_lane_x = []
        left_lane_y = []
        right_lane_y = []

        left_lane_x, right_lane_x, left_lane_y, right_lane_y = self.lane_sampling(points, 2.8, left_boundary, right_boundary, central_curve, thresh, False)
        return left_lane_x, right_lane_x, left_lane_y, right_lane_y


class Road(RoadObject):

    def __init__(self, id, map):
        super(Road, self).__init__(id)
        self.road = map.road.add()
        self.road.id.id = str(id)

    def getID(self):
        return self._id

    def add_section(self, id):
        self.section = self.road.section.add()
        self.section.id.id = str(id)

    def add_lanes_to_section(self, lanes):
        for i in range(len(lanes)):
            self.section.lane_id.add().id = str(lanes[i])

    def add_junction(self, id):
        self.road.junction_id.id = str(id)

    def add_road_boundary(self, points, type, heading):
        edge = self.section.boundary.outer_polygon.edge.add()
        edge.type = type
        segment = edge.curve.segment.add()
        segment.start_position.x, segment.start_position.y = points[0]
        segment.heading = heading
        length = 0
        prevX, prevY = points[0]
        for point in points[1:]:
            p = segment.line_segment.point.add()
            p.x, p.y = point
            length += math.sqrt(math.pow(p.x - prevX, 2) + math.pow(p.y - prevY, 2))
            prevX = p.x
            prevY = p.y
        segment.length = length

    def add(self, lane, section_id, junction_ids, lane_ids, heading):
        self.add_section(section_id)
        for j in junction_ids:
            self.add_junction(j)
        self.add_lanes_to_section(lane_ids)
        lx = lane.left_poly[0][0]
        ly = lane.left_poly[0][1]
        self.add_road_boundary(lane.left_poly, map_road_pb2.BoundaryEdge.LEFT_BOUNDARY, heading)
        rx = lane.right_poly[0][0]
        ry = lane.right_poly[0][1]
        self.add_road_boundary(lane.right_poly, map_road_pb2.BoundaryEdge.RIGHT_BOUNDARY, heading)


class Overlap(RoadObject):

    def __init__(self, id, map):
        super(Overlap, self).__init__(id)
        self.overlap = map.overlap.add()
        self.overlap.id.id = id

    def getID(self):
        return self._id

    def _basic_info_element(self, object, obj_type):
        if obj_type is Lane:
            object.lane_overlap_info.SetInParent()
        elif obj_type is Junction:
            object.junction_overlap_info.SetInParent()

    def _complete_info(self, info, lane, other):
        # Find end points of the overlap
        polygon = Polygon(other.get_polygon())
        central_curve = lane.lane.central_curve.segment[0].line_segment.point
        length = 0
        prev_point = np.array([central_curve[0].x, central_curve[0].y])
        start_s = 0
        end_s = 0
        i = 0
        while not Point(prev_point).within(polygon) and i < len(central_curve):
            curr_point = np.array([central_curve[i].x, central_curve[i].y])
            length += distance(prev_point, curr_point)
            prev_point = curr_point
            i += 1
        if i < len(central_curve):
            start_s = length
            while Point(prev_point).within(polygon) and i < len(central_curve):
                curr_point = np.array([central_curve[i].x, central_curve[i].y])
                length += distance(prev_point, curr_point)
                prev_point = curr_point
                i += 1
            end_s = length

        info.start_s = start_s
        info.end_s = end_s

        # Check if the lane is merging with the other object
        length = 0
        LENGTH_THRESH = 1.0
        in_other_poly = False
        i = 0
        prev_point = np.array([central_curve[0].x, central_curve[0].y])
        while length < LENGTH_THRESH and i < len(central_curve):
            curr_point = np.array([central_curve[i].x, central_curve[i].y])
            length += distance(prev_point, curr_point)
            prev_point = curr_point
            i += 1
            if Point(prev_point).within(polygon):
                in_other_poly = True
                break

        if type(other) is Lane and in_other_poly:
            info.is_merge = True
        else:
            info.is_merge = False

    def add(self, obj1, obj2):
        # Add the overlapping objects
        object1 = self.overlap.object.add()
        object1.id.id = obj1.get_id()
        object2 = self.overlap.object.add()
        object2.id.id = obj2.get_id()
        # Add the specific overlapping objects information
        self._basic_info_element(object1, type(obj1))
        self._basic_info_element(object2, type(obj2))
        # Complete the overlapping objects in case of lanes
        if type(obj1) is Lane:
            self._complete_info(object1.lane_overlap_info, obj1, obj2)
        if type(obj2) is Lane:
            self._complete_info(object2.lane_overlap_info, obj2, obj1)

        # Add the overlap object to the corresponding objects
        obj1.add_overlap(self._id)
        obj2.add_overlap(self._id)


class Junction(RoadObject):

    def __init__(self, id, map):
        super(Junction, self).__init__(id)
        self.junction = map.junction.add()
        self.junction.id.id = id

    def get_id(self):
        return self._id

    def get_polygon(self):
        return self._polygon

    def add_overlap(self, id):
        self.junction.overlap_id.add().id = str(id)

    def add(self, junc_poly):
        self._polygon = np.array(junc_poly)

        # Add the polygon vertices
        for vertex in junc_poly:
            point = self.junction.polygon.point.add()
            point.x, point.y = vertex
