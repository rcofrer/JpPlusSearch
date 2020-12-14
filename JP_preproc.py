import sys
import math
import numpy as np


# class map:
#     def __init__(self, tiles):
#         self.tiles = tiles
#
directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
cardinal_dir = ["N", "E", "S", "W"]
diagonal_dir = ["NE", "SE", "SW", "NW"]
idx_change = [[0,-1], [1,-1], [1,0], [1, 1], [0,1], [-1,1], [-1,0], [-1,-1]]
idx_change_dir = {key:value for key,value in zip(directions,idx_change)}
dir_by_idx_change ={(key[0],key[1]):value for key,value in zip(idx_change,directions)}


def is_wall(m_tiles, r, c):
    wall = True
    try:
        if m_tiles[r][c] == ".":
            wall = False
    except:
        pass
    return wall

def idx_tile_dir(x,y, dir):
    dx, dy = idx_change_dir[dir]
    return [x + dx, y+ dy]


def get_primary_jp(m_tiles):
    # swipe every cardinal direction
    height = len(m_tiles)
    width  = len(m_tiles[0])
    # print(height,width,file=sys.stderr,flush=True)
    jump_points = [[dict.fromkeys(directions, False) for i in range(height)]for j in range(width)]

    for col in range(width):
        for r in range(height):
            if m_tiles[r][col] == "#":
                for d in diagonal_dir:
                    try:
                        dcol, drow = idx_change_dir[d]
                        if col + dcol < 0 or r + drow < 0:
                            continue
                        idx_neighbor = [col + dcol, r + drow]
                        # m_tiles indexed as row,col
                        neighbor_tile = m_tiles[idx_neighbor[1]][idx_neighbor[0]]
                        # print(idx_neighbor[1], idx_neighbor[0], neighbor_tile, file=sys.stderr, flush=True)
                        if neighbor_tile == ".":
                            if m_tiles[r + drow][col] == "." and m_tiles[r][col + dcol] == ".":
                                jump_points[idx_neighbor[0]][idx_neighbor[1]][dir_by_idx_change[dcol, 0]] = True
                                jump_points[idx_neighbor[0]][idx_neighbor[1]][dir_by_idx_change[0, drow]] = True
                                # print("JP: ",idx_neighbor[1],idx_neighbor[0],dir_by_idx_change[dcol, 0], file =sys.stderr,flush=True )
                                # print("JP: ", idx_neighbor[1], idx_neighbor[0], dir_by_idx_change[0, drow],
                                #       file=sys.stderr, flush=True)
                    except:
                        pass
    return jump_points


def get_distances(m_tiles, jump_pts):
    height = len(m_tiles)
    width = len(m_tiles[0])
    # indexed [col][row]
    distances = [[dict.fromkeys(directions) for i in range(height)] for j in range(width)]

    # left to right sweep
    for r in range(height):
        count = -1
        jump_point_last_seen = False
        for c in range(width):
            if m_tiles[r][c] == "#":
                count = -1
                jump_point_last_seen = False
                distances[c][r]["W"] = 0
                continue
            count +=1
            if jump_point_last_seen:
                distances[c][r]["W"] = count
            else:
                distances[c][r]["W"] = -count
            if jump_pts[c][r]["W"]:
                count = 0
                jump_point_last_seen = True

    # right to left sweep
    for r in range(height):
        count = -1
        jump_point_last_seen = False
        for c in reversed(range(width)):
            if m_tiles[r][c] == "#":
                count = -1
                jump_point_last_seen = False
                distances[c][r]["E"] = 0
                continue
            count +=1

            if jump_point_last_seen:
                distances[c][r]["E"] = count
            else:
                distances[c][r]["E"] = -count
            if jump_pts[c][r]["E"]:
                count = 0
                jump_point_last_seen = True
    # Top to bottom sweep (distances to North dir)
    for c in range(width):
        count = -1
        jump_point_last_seen = False
        for r in range(height):
            if m_tiles[r][c] == "#":
                count = -1
                jump_point_last_seen = False
                distances[c][r]["N"] = 0
                continue
            count +=1

            if jump_point_last_seen:
                distances[c][r]["N"] = count
            else:
                distances[c][r]["N"] = -count
            if jump_pts[c][r]["N"]:
                count = 0
                jump_point_last_seen = True
    # Bottom to top sweep (distances to South dir)
    for c in range(width):
        count = -1
        jump_point_last_seen = False
        for r in reversed(range(height)):
            if m_tiles[r][c] == "#":
                count = -1
                jump_point_last_seen = False
                distances[c][r]["S"] = 0
                continue
            count += 1

            if jump_point_last_seen:
                distances[c][r]["S"] = count
            else:
                distances[c][r]["S"] = -count
            if jump_pts[c][r]["S"]:
                count = 0
                jump_point_last_seen = True

    # Filling diagonal distances

    # Bottom to top
    for r in reversed(range(height)):
        for c in range(width):

            if not is_wall(m_tiles, r, c):
                # SW dir
                if (r == height-1 or c == 0 or is_wall(m_tiles, r+1, c) or
                   is_wall(m_tiles, r, c-1) or is_wall(m_tiles, r+1, c-1)):

                    distances[c][r]["SW"] = 0

                elif (not is_wall(m_tiles, r+1, c) and not is_wall(m_tiles, r, c-1) and
                      (distances[c-1][r+1]["S"] > 0 or distances[c-1][r+1]["W"] > 0)):

                    distances[c][r]["SW"] = 1
                else:
                    jump_distance = distances[c-1][r+1]["SW"]
                    if jump_distance > 0:
                        distances[c][r]["SW"] = 1 + jump_distance
                    else:
                        distances[c][r]["SW"] = -1 + jump_distance

            #SE dir

                if (r == height - 1 or c == width - 1 or is_wall(m_tiles, r + 1, c) or
                   is_wall(m_tiles, r, c + 1) or is_wall(m_tiles, r + 1, c + 1)):

                    distances[c][r]["SE"] = 0

                elif (not is_wall(m_tiles, r+1, c) and not is_wall(m_tiles,r,c+1) and
                      (distances[c+1][r+1]["S"] > 0 or distances[c+1][r+1]["E"]>0)):

                    distances[c][r]["SE"] = 1
                else:
                    jump_distance = distances[c+1][r+1]["SE"]
                    if jump_distance > 0:
                        distances[c][r]["SE"] = 1 + jump_distance
                    else:
                        distances[c][r]["SE"] = -1 + jump_distance

        # Top to bottom
        for r in range(height):
            for c in range(width):

                if not is_wall(m_tiles, r, c):
                    # NW dir
                    if (r == 0 or c == 0 or is_wall(m_tiles, r - 1, c) or
                            is_wall(m_tiles, r, c - 1) or is_wall(m_tiles, r - 1, c - 1)):

                        distances[c][r]["NW"] = 0

                    elif (not is_wall(m_tiles, r - 1, c) and not is_wall(m_tiles, r, c - 1) and
                          (distances[c - 1][r - 1]["N"] > 0 or distances[c - 1][r - 1]["W"] > 0)):

                        distances[c][r]["NW"] = 1
                    else:
                        jump_distance = distances[c - 1][r - 1]["NW"]
                        if jump_distance > 0:
                            distances[c][r]["NW"] = 1 + jump_distance
                        else:
                            distances[c][r]["NW"] = -1 + jump_distance

                    # NE dir

                    if (r == 0 or c == width - 1 or is_wall(m_tiles, r - 1, c) or
                            is_wall(m_tiles, r, c + 1) or is_wall(m_tiles, r - 1, c + 1)):

                        distances[c][r]["NE"] = 0

                    elif (not is_wall(m_tiles, r - 1, c) and not is_wall(m_tiles, r, c + 1) and
                          (distances[c + 1][r - 1]["N"] > 0 or distances[c + 1][r - 1]["E"] > 0)):

                        distances[c][r]["NE"] = 1
                    else:
                        jump_distance = distances[c + 1][r - 1]["NE"]
                        if jump_distance > 0:
                            distances[c][r]["NE"] = 1 + jump_distance
                        else:
                            distances[c][r]["NE"] = -1 + jump_distance
    return distances
# Compute the proper wall / jump point distances, according to the preprocessing phase of the JPS+ algorithm.

# width: Width of the map
# height: Height of the map


map_tiles = []
width, height = [int(i) for i in input().split()]
for i in range(height):
    row = input()  # A single row of the map consisting of passable terrain ('.') and walls ('#')
    map_tiles.append(row)
# Write an action using print
# To debug: print("Debug messages...", file=sys.stderr, flush=True)
jump_points = get_primary_jp(map_tiles)
distance    = get_distances(map_tiles, jump_points)


# For each empty tile of the map, a line containing "column row N NE E SE S SW W NW".
for r in range(height):
    for c in range(width):
        if map_tiles[r][c] == ".":
            print(f"{c} {r} {distance[c][r]['N']} {distance[c][r]['NE']} {distance[c][r]['E']}"
                  f" {distance[c][r]['SE']} {distance[c][r]['S']} {distance[c][r]['SW']}"
                  f" {distance[c][r]['W']} {distance[c][r]['NW']}")


