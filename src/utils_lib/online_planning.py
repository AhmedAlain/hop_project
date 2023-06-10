
import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og



def wrap_angle(angle):
    return (angle + (2.0 * np.pi * np.floor((np.pi - angle) / (2.0 * np.pi))))


class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=False):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None
        # map sampling resolution (size of a cell))
        self.resolution = None
        # world position of cell (0, 0) in self.map
        self.origin = None
        # set method has been called
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position
        self.distance = distance
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid

    # Set occupancy map, its resolution and origin.
    def set(self, data, resolution, origin):
        # print("Hello")
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        #-2.770080089569092, -0.07666170597076416
        #[-1.8517 -1.94809]
        # p = self.__position_to_map__([-1.30278, -1.66532])
        # print("p",p)
        # print("self.map[r, c]",self.map[p[0],p[1]])
        # print(self.is_valid([-1.31879, -1.6977]))

    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose):
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        map_coordinates = self.__position_to_map__(pose)
        if map_coordinates == []:
            occupied = True
        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribude).
        else:
            row, col = map_coordinates
            occupied = False
            cell = int(self.distance/self.resolution)

            for r in range(row - cell, row + cell):
                for c in range(col - cell, col + cell):

                    if r < 0 or c < 0 or r >= self.map.shape[0] or c >= self.map.shape[1]:
                        occupied = True
                        continue
                    if self.map[r, c] > 50:
                        occupied = True
                    if self.map[r, c] == -1 and not self.is_unknown_valid:
                        occupied = True
        # print(f"map cord {map_coordinates}, occ{occupied}")
        return not occupied
        # Return True if free, False if occupied and self.is_unknown_valid if unknown.
        
    # def is_valid(self, pose):
    #     # Convert world robot position to map coordinates
    #     map_coordinates = self.__position_to_map__(pose)
    #     occupied = False
    #     if not map_coordinates:
    #         occupied = True
    #     else:
    #         row, col = map_coordinates

    #         # Check if current cell is out of map boundaries or if it's occupied
    #         if row < 0 or col < 0 or row >= self.map.shape[0] or col >= self.map.shape[1]:
    #             occupied = True
    #         elif self.map[row, col] > 50:
    #             occupied = True
    #         elif self.map[row, col] == -1 and not self.is_unknown_valid:
    #             occupied = True

    #     return not occupied

        
    def move_centroide(self, centroide):
        row, col = centroide
        offsets = [(0.3, 0), (0.3, 0.3), (0, 0.3), (-0.3, 0.3), (-0.3, 0), (-0.18, -0.1), (0, -0.18), (0.18, -0.18)]
        for offset in offsets:
            centroide_new = [row + offset[0], col + offset[1]]
            if self.is_valid(centroide_new):
                return centroide_new
        return centroide
    

    def check_path(self, path, step_size=0.05):
        # TODO: Discretize the positions between 2 waypoints with step_size
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True.
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i+1])
            check_segment = np.array([p1[0] - p2[0], p1[1] - p2[1]])
            unit_vector = check_segment / np.sqrt(np.sum(check_segment ** 2))
            dista = math.dist(p1, p2)
            i = 0
            while i <= dista:
                segmant_c = p1 - (unit_vector * i)
                segmant_c[0] = round(segmant_c[0], 4)
                segmant_c[1] = round(segmant_c[1], 4)
                # if 0 <= int(segmant_c[0]) < self.map.shape[0] and 0 <= int(segmant_c[1]) < self.map.shape[1] and self.map[
                #     int(segmant_c[0]), int(segmant_c[1])] == 1:
                if not self.is_valid(segmant_c):
                    return False
                i += step_size
        return True
    
    def smooth_path(self, path, step_size=0.005):
        # Create a new path that starts at the first waypoint and ends at the last waypoint
        new_path = [path[0], path[-1]]

        # Interpolate between the waypoints using line smoothing
        for i in range(len(path) - 2):
            p1 = np.array(path[i])
            p2 = np.array(path[i+1])
            p3 = np.array(path[i+2])
            # Calculate the direction vectors between the waypoints
            v1 = p2 - p1
            v2 = p3 - p2
            # Calculate the angle between the direction vectors
            angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
            # If the angle is greater than a certain threshold, add a new waypoint at the midpoint of the line segment
            if angle > np.pi / 3:
                new_point = (p1 + p2) / 2
                new_path.append(new_point)
            # Add the original waypoint to the new path
            new_path.append(p2)
        # Check the validity of each step in the new path using the is_valid function
        for point in new_path:
            if not self.is_valid(point):
                return None
        return new_path

    """def rdp(self, points, epsilon):
        
        # If there are only two points, return them
        if len(points) <= 2:
            return points
        else:
            # Calculate the perpendicular distance of each point from the line segment
            d_max = 0
            index = 0
            for i in range(1, len(points) - 1):
                d = perpendicular_distance(points[i], points[0], points[-1])
                if d > d_max:
                    index = i
                    d_max = d
            # If the maximum distance is less than epsilon, return the endpoints of the curve
            if d_max <= epsilon:
                return [points[0], points[-1]]
            else:
                # Recursively simplify the left and right segments
                left = self.rdp(points[:index+1], epsilon)
                right = self.rdp(points[index:], epsilon)
                # Combine the results and return them
                return left[:-1] + right"""
            
    def Smooth(self, map, path):
        # Convert path to list of points
        points = [(p[0], p[1]) for p in path]

        # Smooth the path using the Ramer-Douglas-Peucker algorithm
        smoothed_points = self.rdp(points, epsilon=0.1)

        # Convert smoothed points back to a list of positions
        smoothed_path = [[p[0], p[1]] for p in smoothed_points]

        return smoothed_path
    
    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`
        x, y = p[1], p[0]
        col = int((y - self.origin[1]) / self.resolution)
        row = int((x - self.origin[0]) / self.resolution)

        map_coord = [col, row]
        if row < 0 or col < 0 or row >= self.map.shape[0] or col >= self.map.shape[1]:
            return []
        return map_coord
    def add_dicts(self, d1, d2, d3):
        result = {}
        for key, value1, value2 in zip(d1.keys(), d1.values(), d2.values()):
            if key in d3:
                result[key] = value1 + value2 + d3[key]
            else:
                result[key] = value1 + value2
        return result

    def scale(self, dict, range):

        min_val = min(dict.values())
        max_val = max(dict.values())

        for key in dict:
            val = dict[key]
            new_val = ((val - min_val) * (range[1] - range[0]) / (max_val - min_val)) + range[0]
            dict[key] = new_val

        return dict
  
    def simulate_motion(self, v, w, dt, x, y, theta):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        return x, y, theta

    def compute_angle(self, point, goal):
        x, y = point
        goal_x, goal_y = goal

        dx = goal_x - x
        dy = goal_y - y

        angle = math.atan2(dy, dx)

        return angle


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, dominion, max_time=3):

    # TODO: Plan a path from start_p to goal_p inside dominion using the OMPL and the state_validity_checker object. Follow notebook example.
    # some code

    space = ob.RealVectorStateSpace(2)
    space.setBounds(dominion[0], dominion[1])

    space.setLongestValidSegmentFraction(0.001)

    si = ob.SpaceInformation(space)

    si.setStateValidityChecker(
        ob.StateValidityCheckerFn(state_validity_checker.is_valid))
    # create a start state
    start = ob.State(space)
    start[0] = start_p[0]
    start[1] = start_p[1]
    print(start)

    # create a goal state
    goal = ob.State(space)
    goal[0] = goal_p[0]
    goal[1] = goal_p[1]

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))

    # optimizingPlanner = og.RRTstar(si)
    # optimizingPlanner = og.RRTConnect(si)
    optimizingPlanner = og.RRT(si)

    # it represents the maximum length of a motion to be added in the tree of motions.
    optimizingPlanner.setRange(0.5)

    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # print("aaaaaa pose, goal", start_p, goal_p)
    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(max_time)

    pd = ob.PlannerData(si)
    optimizingPlanner.getPlannerData(pd)

    # TODO: if solved fill ret with the points [x, y] in the solution path
    ret = []
    if solved:
        # get the path and transform it to a list
        path = pdef.getSolutionPath()
        # print("Found solution:\n%s" % path)
        ret = []
        for i in path.getStates():
            ret.append((i[0], i[1]))
    else:
        print("No solution found")
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!
    # If the distance is greater than 0.05 (the tolerance distance)
    if len(ret) > 0 and math.dist(ret[-1], goal_p) > 0.05:
        print("Path does not reach goal")
        ret = []
    return ret


# Controller: Given the current position and the goal position, this function computes the desired
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):

    # TODO: Implement a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # Make it sequential to avoid strange curves. First correct orientation and then distance.
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)

    theta_d = math.atan2((goal[1]-current[1]), (goal[0]-current[0]))
    w = Kw * wrap_angle(theta_d - current[2])
    if np.abs(theta_d - current[2]) > np.deg2rad(5):
        v = 0
    else:
        v = Kv * math.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
    return v, w
