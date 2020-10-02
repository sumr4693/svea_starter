"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
"""

import math
import matplotlib.pyplot as plt
from timeout import timeout
show_animation = False


__rediting team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Improved"



class AStarPlanner:

    def __init__(self, grid_shape, reso, rr, origin_x, origin_y):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """  
        self.minx = 0
        self.miny = 0
        self.maxx =  grid_shape[1]
        self.maxy =  grid_shape[0]
        
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.reso = reso
        self.rr = rr
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, c, cost, pind):
            self.x = c[0]  # index of grid
            self.y = c[1]  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)
            
    @timeout(2)
    def planning(self, sx, sy, gx, gy, occupancyGrid, inflate =False):
        """
        A star path search for shorter time

        input
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """


        self.occupancy_grid = occupancyGrid

        nstart = self.Node(self.calc_xyindex(sx,sy), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx,gy), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation: 
                (x,y) = self.grid_index_to_position(current)
                plt.plot(x, y,"xc")
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node( (current.x + self.motion[i][0], current.y + self.motion[i][1]), current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry
        
    @timeout(11)
    def planning_longer(self, sx, sy, gx, gy, occupancyGrid, inflate =False):
        """
        A star path search for longer time

        input
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """


        self.occupancy_grid = occupancyGrid

        nstart = self.Node(self.calc_xyindex(sx,sy), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx,gy), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover 
                (x,y) = self.grid_index_to_position(current)
                plt.plot(x, y,"xc")
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node( (current.x + self.motion[i][0], current.y + self.motion[i][1]), current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)
        return rx, ry
        
    def calc_final_path(self, ngoal, closedset):
        # generate final course
        (x,y) = self.grid_index_to_position(ngoal)
        rx, ry = [x], [y]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            (x,y) = self.grid_index_to_position(n)
            rx.append(x)
            ry.append(y)
            pind = n.pind
        rx.reverse()
        ry.reverse()
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 0.5  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def grid_index_to_position(self, nodee):
        x = nodee.x *self.reso +self.origin_x
        y = nodee.y *self.reso +self.origin_y
        return (x,y)

    def calc_xyindex(self, x, y):
        grid_x = int(round((x - self.origin_x) / self.reso))
        grid_y = int(round((y - self.origin_y) / self.reso))
        return (grid_x, grid_y)

        
    def calc_grid_index(self, nodee):
        return nodee.y * self.maxx + nodee.x

    def verify_node(self, nodee):
        if nodee.x < self.minx:
            return False
        elif nodee.y < self.miny:
            return False
        elif nodee.x >= self.maxx:
            return False
        elif nodee.y >= self.maxy:
            return False
        # Dynamic inflation
        rradius = int(math.ceil(self.rr/self.reso))
        bad = (self.occupancy_grid[ nodee.y-rradius : nodee.y+rradius, nodee.x-rradius:nodee.x+rradius]==1).any()
        if bad:
            return False

        return True
    
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
