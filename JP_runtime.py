import sys
import math
import numpy as np

class Node:
    def __init__(self, c, r, distances):
        self.c          = c
        self.r          = r
        self.parent     = None
        self.given_cost = 0
        self.final_cost = 0
        self.distances  = distances


def equal(node_a, node_b):
    return(node_a.c == node_b.c and node_a.r == node_b.r)


def make_path(end_node):
    previous_node = end_node.parent
    path_to_goal = [end_node]
    while not (previous_node is None):
        path_to_goal.insert(0, previous_node)
        previous_node = previous_node.parent
    return path_to_goal


# utility
directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
cardinal_dir = ["N", "E", "S", "W"]
diagonal_dir = ["NE", "SE", "SW", "NW"]
idx_change = [[0,-1], [1,-1], [1,0], [1, 1], [0,1], [-1,1], [-1,0], [-1,-1]]
idx_change_dir = {key:value for key,value in zip(directions,idx_change)}
dir_by_idx_change ={(key[0],key[1]):value for key,value in zip(idx_change,directions)}

valid_dir_lookup_table = {"S": ["E", "SE", "S","SW", "W"],
                          "SE": ["E", "SE", "S"],
                          "E": ["N", "NE", "E", "SE","S"],
                          "NE": ["N", "NE", "E"],
                          "N": ["W", "NW", "N", "NE","E"],
                          "NW": ["W", "NW", "N"],
                          "W": ["S", "SW", "W", "NW","N"],
                          "SW": ["S", "SW", "W"],
                          "Start": directions}


def create_distances_dict(n, ne, e, se, s, sw, w, nw):
    d_dict = {key: value for key, value in zip(directions, [n, ne, e, se, s, sw, w, nw])}
    return d_dict


def diff_nodes(node_a, node_b):
    dx = node_b.c - node_a.c
    dy = node_b.r - node_a.r
    if dx==0 or dy==0:
        return abs(dx) + abs(dy)
    else:
        return (abs(dx) + abs(dy))/2


def diff_nodes_row(node_a, node_b):
    dy = node_b.r - node_a.r
    return abs(dy)


def diff_nodes_col(node_a, node_b):
    dx = node_b.c - node_a.c
    return abs(dx)


def in_list(node, list):
    return ([node.c,node.r] in list)


def calculate_heuristic(node_a, node_b):
    dx = abs(node_b.c - node_a.c)
    dy = abs(node_b.r - node_a.r)
    distance = dx + dy + (math.sqrt(2)-2)*min(dx, dy)
    return distance


class JPp:
    def __init__(self, map_nodes, start_c, start_r, goal_c, goal_r):

        self.map_nodes = map_nodes
        self.start_c   = start_c
        self.start_r   = start_r
        self.goal_c    = goal_c
        self.goal_r    = goal_r

    def get_travel_dir(self, node):
        parent_node = node.parent
        if parent_node is None:
            for (key, value) in node.distances.items():
                if value >0:
                    return key

        else:
            dx  = int((node.c - parent_node.c)/max(abs(node.c - parent_node.c), 1))
            dy  = int((node.r - parent_node.r)/max(abs(node.r - parent_node.r), 1))
            return dir_by_idx_change[dx, dy]

    def goal_in_exact_card_dir(self, node, dir):
        idx_delta = idx_change_dir[dir]
        node_idx  = [node.c, node.r]
        goal_idx  = [self.goal_c, self.goal_r]
        idx_to_check = idx_delta.index(0)
        idx_to_check_dir = int(abs(idx_to_check-1))
        goal_in_same_horizon = node_idx[idx_to_check] == goal_idx[idx_to_check]
        goal_in_dir          = (goal_idx[idx_to_check_dir]- node_idx[idx_to_check_dir])*idx_delta[idx_to_check_dir]

        return (goal_in_same_horizon and goal_in_dir)

    def goal_in_gen_dir_diagonal(self, node, dir):
        idx_delta = idx_change_dir[dir]
        node_idx = [node.c, node.r]
        goal_idx = [self.goal_c, self.goal_r]

        return (idx_delta[0]*(goal_idx[0] - node_idx[0]) > 0 and idx_delta[1]*(goal_idx[1] - node_idx[1]) > 0)

    def get_node(self,node, dir, n_moves= None):
        if n_moves is None:
            # jump point in dir
            idx_dir = idx_change_dir[dir]
            jump_distance = node.distances[dir]
            node_after_jump = self.map_nodes[node.c + idx_dir[0]*jump_distance][node.r + idx_dir[1]*jump_distance]
            return node_after_jump
        else:
            idx_dir = idx_change_dir[dir]
            node_after_move = self.map_nodes[node.c + idx_dir[0]*n_moves][node.r + idx_dir[1]*n_moves]
            return node_after_move

    def get_f_score(self,colrow):
        return self.map_nodes[colrow[0]][colrow[1]].final_cost

    def JPS_runtime(self):
        open_list   = [[self.start_c,self.start_r]]
        goal_node   = self.map_nodes[self.goal_c][self.goal_r]
        closed_list = []
        nodes_poped = []
        while len(open_list) > 0:
            print("open_list",open_list , file=sys.stderr, flush=True)
            f_scores = list(map(self.get_f_score, open_list))
            idx_to_pop = np.argmin(f_scores)
            if not (type(idx_to_pop) is np.int64):
                idx_to_pop = idx_to_pop[-1]
            curr_node_idx   = open_list.pop(idx_to_pop)
            curr_node       = self.map_nodes[curr_node_idx[0]][curr_node_idx[1]]
            nodes_poped.append(curr_node)
            parent_node     = curr_node.parent

            if equal(curr_node, goal_node):
                # return make_path(curr_node), nodes_poped
                return nodes_poped

            travel_dir = None

            if curr_node.parent is None:
                travel_dir = "Start"
            else:
                travel_dir = self.get_travel_dir(curr_node)

            for direction in valid_dir_lookup_table[travel_dir]:

                new_succesor = None
                given_cost    = 0
                if (direction in cardinal_dir and self.goal_in_exact_card_dir(curr_node, direction)
                   and diff_nodes(curr_node, goal_node) <= abs(curr_node.distances[direction])):

                    new_succesor = goal_node
                    given_cost   = curr_node.given_cost + diff_nodes(curr_node, goal_node)

                elif (direction in diagonal_dir and self.goal_in_gen_dir_diagonal(curr_node,direction) and
                      (diff_nodes_row(curr_node, goal_node) <= abs(curr_node.distances[direction]) or
                      diff_nodes_col(curr_node, goal_node) <= abs(curr_node.distances[direction]))):
                    # create target jump point
                    min_diff = min(diff_nodes_row(curr_node, goal_node), diff_nodes_col(curr_node, goal_node))
                    new_succesor = self.get_node(curr_node, direction, n_moves=min_diff)
                    given_cost   = curr_node.given_cost + math.sqrt(2)*min_diff
                elif (curr_node.distances[direction] > 0):
                    # jump point in this direction
                    new_succesor = self.get_node(curr_node, direction)
                    given_cost   = diff_nodes(curr_node, new_succesor)
                    if direction in diagonal_dir:
                        given_cost *= math.sqrt(2)
                    given_cost += curr_node.given_cost

                if not (new_succesor is None):
                    if not (in_list(new_succesor, open_list) or in_list(new_succesor, closed_list)):

                        new_succesor.parent     = curr_node
                        new_succesor.given_cost = given_cost
                        new_succesor.final_cost = given_cost + calculate_heuristic(new_succesor, goal_node)
                        open_list.insert(0,[new_succesor.c, new_succesor.r])
                    elif given_cost < new_succesor.given_cost:
                        new_succesor.parent = curr_node
                        new_succesor.given_cost = given_cost
                        new_succesor.final_cost = given_cost + calculate_heuristic(new_succesor, goal_node)
                        if not in_list(new_succesor, open_list):
                            # DOUBTS HERE
                            open_list.insert(0, [new_succesor.c, new_succesor.r])
            closed_list.append([curr_node.c, curr_node.r])
        nodes_poped.append("NO PATH")
        return nodes_poped
# Compute the nodes visited by the JPS+ algorithm when performing runtime phase search.

# width: Width of the map
# height: Height of the map


width, height = [int(i) for i in input().split()]

# list of nodes on map, indexed [c][r]
map_nodes = [["#" for r in range(height)] for c in range(width)]

# start_column: coordinate of the starting tile
# start_row: coordinate of the starting tile
# goal_column: coordinate of the goal tile
# goal_row: coordinate of the goal tile
start_column, start_row, goal_column, goal_row = [int(i) for i in input().split()]
_open = int(input())  # number of open tiles on the map
for i in range(_open):
    # column: coordinate of the empty tile described
    # row: coordinate of the empty tile described
    # n: distance to the closest jump point (positive number) or wall (otherwise) going north
    # ne: distance to the closest jump point (positive number) or wall (otherwise) going northeast
    # e: distance to the closest jump point (positive number) or wall (otherwise) going east
    # se: distance to the closest jump point (positive number) or wall (otherwise) going southeast
    # s: distance to the closest jump point (positive number) or wall (otherwise) going south
    # sw: distance to the closest jump point (positive number) or wall (otherwise) going southwest
    # w: distance to the closest jump point (positive number) or wall (otherwise) going west
    # nw: distance to the closest jump point (positive number) or wall (otherwise) going northwest
    column, row, n, ne, e, se, s, sw, w, nw = [int(j) for j in input().split()]
    map_nodes[column][row] = Node(column, row, create_distances_dict(n, ne, e, se, s, sw, w, nw))

# Write an action using print
# To debug: print("Debug messages...", file=sys.stderr, flush=True)

path_finder = JPp(map_nodes, start_column, start_row, goal_column, goal_row)
nodes_explored  = path_finder.JPS_runtime()
n_nodes_explored= 0
# game loop
while True:

    # Write an action using print
    # To debug: print("Debug messages...", file=sys.stderr, flush=True)


    # In order of nodes visited by the JPS+ algorithm, a line containing "nodeColumn nodeRow parentColumn parentRow givenCost".
    current_node = nodes_explored[n_nodes_explored]

    if current_node == "NO PATH":
        print(current_node)
        break

    curr_node_parent = current_node.parent
    if curr_node_parent is None:
        print(f"{current_node.c} {current_node.r} {-1} {-1} {0.00}")
    else:
        print(f"{current_node.c} {current_node.r} {curr_node_parent.c} {curr_node_parent.r} {current_node.given_cost}")
    n_nodes_explored +=1

    if n_nodes_explored == len(nodes_explored):
        break