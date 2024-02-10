#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

# SOKOBAN HEURISTIC

def heur_alternate_1(state):
    
    total_distance = 0
    
    
    boxes = set(state.boxes)
    storage_points = set(state.storage)
    obstacles = set(state.obstacles)
    corners = generate_corners(state.width, state.height)


    UP = Direction("up", (0, -1))
    RIGHT = Direction("right", (1, 0))
    DOWN = Direction("down", (0, 1))
    LEFT = Direction("left", (-1, 0))

    directions = [UP, RIGHT, DOWN, LEFT]
    
    
    potential_deadlocks = set()
    for box_pos in boxes:
        for direction in directions:
            next_pos = direction.move(box_pos)
            if next_pos in obstacles or next_pos in boxes or next_pos in corners:
                # Box would be blocked in this direction
                potential_deadlocks.add(box_pos)
                break

    # dont do next position, look as the state as it is. identify any corners for the boxes for each box, but you
            # cannot precompute
            # but why use the directions then???

    # do an assignment logic: any storage point should be assigned to its nearest box
    # then calculate the distance of each box to its assigned storage point (consider obstacles)
    # consider each storage point and find whether there is a wall attached to it (but why?)
            
    

    # Calculate heuristic value
    for box_pos in boxes:
        min_distance_to_storage = float('inf')
        if box_pos not in storage_points:
            for storage_pos in storage_points:
                if storage_pos not in boxes:
                    distance = calculate_manhattan_distance_with_obstacles(box_pos, storage_pos, obstacles)
                    min_distance_to_storage = min(min_distance_to_storage, distance)
            
            if box_pos in potential_deadlocks:
                total_distance += min_distance_to_storage * 2  # Punish for potential deadlocks
            else:
                total_distance += min_distance_to_storage

    
    
    return total_distance

def heur_alternate_2(state):
    
    total_distance = 0
    
    
    boxes = set(state.boxes)
    storages = set(state.storage)
    obstacles = set(state.obstacles)
    width = state.width
    height = state.height
    

    edge_boxes = set()
    for box in boxes:
        if box[0] == 0 or box[0] == width - 1 or box[1] == 0 or box[1] == height - 1:
            edge_boxes.add(box)
    
    boxes = boxes - edge_boxes
    

    for edge_box in edge_boxes:
        if is_box_deadlocked(edge_box, obstacles, storages, height, width):
            total_distance = 50
        min_distance = float('inf')
        nearest_storage = None
        for storage in storages:
            if storage[0] == edge_box[0] or storage[1] == edge_box[1]:
                # distance = calculate_manhattan_distance(bo, stoarage)
                distance = calculate_manhattan_distance_with_obstacles(edge_box, storage, obstacles)
                if distance < min_distance:
                    min_distance = distance
                    nearest_storage = storage
        if nearest_storage is not None:
            storages.remove(nearest_storage)
        else:
            return float('inf')
        total_distance += min_distance


    for box in boxes:

        if is_box_deadlocked(box, obstacles, storages, height, width):
            total_distance = 50
        
        # there are no deadlocked boxes, assign each box to its nearest storage point
        min_distance = float('inf')
        nearest_storage = None
        for storage in storages:
            # distance = calculate_manhattan_distance(bo, stoarage)
            distance = calculate_manhattan_distance_with_obstacles(box, storage, obstacles)
            if distance < min_distance:
                min_distance = distance
                nearest_storage = storage

        storages.remove(nearest_storage)
        total_distance += min_distance

    return total_distance

def heur_alternate_3(state):
    
    total_distance = 0
    
    
    boxes = set(state.boxes)
    storages = set(state.storage)
    obstacles = set(state.obstacles)
    width = state.width
    height = state.height

    assignments = assign_storage_to_boxes(state) # create dictionary of box to storage point assignments

    for box, (storage, distance) in assignments.items():
        if box is None:
            continue
        if is_box_deadlocked_2(box, obstacles, storage, height, width):
            return float('inf')

        total_distance += distance

    return total_distance


def generate_corners(width, height):
    corners = {(0, 0), (0, height - 1), (width - 1, 0), (width - 1, height - 1)}
    return corners

def calculate_manhattan_distance_with_obstacles(start_pos, end_pos, obstacles):
    
    # Calculate horizontal and vertical distances separately
    horizontal_distance = abs(start_pos[0] - end_pos[0])
    vertical_distance = abs(start_pos[1] - end_pos[1])
    
    # Adjust horizontal distance for obstacles
    for obstacle in obstacles:
        # Check if the obstacle is between start_pos and end_pos horizontally
        if obstacle[1] == start_pos[1] == end_pos[1] and min(start_pos[0], end_pos[0]) < obstacle[0] < max(start_pos[0], end_pos[0]):
            horizontal_distance += 2
    
    # Adjust vertical distance for obstacles
    for obstacle in obstacles:
        # Check if the obstacle is between start_pos and end_pos vertically
        if obstacle[0] == start_pos[0] == end_pos[0] and min(start_pos[1], end_pos[1]) < obstacle[1] < max(start_pos[1], end_pos[1]):
            vertical_distance += 2
    
    return horizontal_distance + vertical_distance

def calculate_manhattan_distance(start_pos, end_pos):
    return abs(start_pos[0] - end_pos[0]) + abs(start_pos[1] - end_pos[1])

def is_box_deadlocked(box_pos, obstacles, storages, height, width):
    bx, by = box_pos # cgeck for storage bc its not infinite # check for 2x2

    blocked_sides = [False, False, False, False]
    # 0 : up, 
    # 1: right, 
    # 2: down, 
    # 3: left

    # check box above
    if by + 1 == height or (bx, by + 1) in obstacles:
        blocked_sides[0] = True
    # check box to the right
    elif bx + 1 == width or (bx + 1, by) in obstacles:
        blocked_sides[1] = True
    # check box below
    elif by - 1 < 0 or (bx, by - 1) in obstacles:
        blocked_sides[2] = True
    # check box to the left
    elif bx - 1 < 0 or (bx - 1, by) in obstacles:
        blocked_sides[3] = True
    
    # check if box is in a corner (corner is defined as at least two adjacent sides being blocked))
    is_in_corner = (blocked_sides[0] and blocked_sides[1]) or \
                    (blocked_sides[1] and blocked_sides[2]) or \
                    (blocked_sides[2] and blocked_sides[3]) or \
                    (blocked_sides[3] and blocked_sides[0])
    
    if is_in_corner: # this checks if box is in a corner (2 or more sides are blocked)
        if (bx, by + 1) in storages: # check above
            return False
        elif (bx + 1, by) in storages:  # check to right
            return False
        elif (bx, by - 1) in storages:  # check below
            return False
        elif (bx - 1, by) in storages:  # check left
            return False 
        return True
    
    if blocked_sides[0] == False and blocked_sides[2] == False and blocked_sides[1] == False and blocked_sides[3] == False: # this is a free box
        return False

    # Handle cases of boxes which are blocked by one side because they are beside the edge of the grid
    if bx == 0 or bx == width - 1:
        for storage in storages:
            if bx == storage[0]:
                return False
    elif by == 0 or by == height - 1:
        for storage in storages:
            if by == storage[1]:
                return False
            

    return True

def is_box_deadlocked_2(box_pos, obstacles, storage, height, width):
    bx, by = box_pos # cgeck for storage bc its not infinite # check for 2x2

    blocked_sides = [False, False, False, False]
    # 0 : up, 
    # 1: right, 
    # 2: down, 
    # 3: left

    # check box above
    if by + 1 == height or (bx, by + 1) in obstacles:
        blocked_sides[0] = True
    # check box to the right
    elif bx + 1 == width or (bx + 1, by) in obstacles:
        blocked_sides[1] = True
    # check box below
    elif by - 1 < 0 or (bx, by - 1) in obstacles:
        blocked_sides[2] = True
    # check box to the left
    elif bx - 1 < 0 or (bx - 1, by) in obstacles:
        blocked_sides[3] = True
    
    # check if box is in a corner (corner is defined as at least two adjacent sides being blocked))
    is_in_corner = (blocked_sides[0] and blocked_sides[1]) or \
                    (blocked_sides[1] and blocked_sides[2]) or \
                    (blocked_sides[2] and blocked_sides[3]) or \
                    (blocked_sides[3] and blocked_sides[0])
    
    if is_in_corner: # this checks if box is in a corner (2 or more sides are blocked)
        if (bx, by + 1) == storage: # check above
            return False
        elif (bx + 1, by) == storage:  # check to right
            return False
        elif (bx, by - 1) == storage:  # check below
            return False
        elif (bx - 1, by) == storage:  # check left
            return False 
        return True
    
    if blocked_sides[0] == False and blocked_sides[2] == False and blocked_sides[1] == False and blocked_sides[3] == False: # this is a free box
        return False

    # Handle cases of boxes which are blocked by one side because they are beside the edge of the grid
    if bx == 0 or bx == width - 1:
        if bx == storage[0]:
            return False
    elif by == 0 or by == height - 1:
        if by == storage[1]:
            return False
            

    return True

def assign_storage_to_boxes(state):
    '''Assign each storage point to its nearest box.'''
    assignments = {}
    remaining_storages = set(state.storage)
    remaining_boxes = set(state.boxes)
    
    
    def f():
        for box in state.boxes:
            is_edge, edge_type = is_at_edge(box, state.width, state.height)
            if is_edge:
                if edge_type in ["left", "right"]:
                    # Look for a storage with the same x coordinate
                    for storage in remaining_storages:
                        if storage[0] == box[0]:  
                            assignments[box] = (storage, calculate_manhattan_distance_with_obstacles(box, storage, state.obstacles))
                            remaining_storages.remove(storage)
                            remaining_boxes.remove(box)
                            break 

                elif edge_type in ["top", "bottom"]:
                    # Look for a storage with the same y coordinate
                    for storage in remaining_storages:
                        if storage[1] == box[1]:  
                            
                            assignments[box] = (storage, calculate_manhattan_distance_with_obstacles(box, storage, state.obstacles))
                            remaining_storages.remove(storage)
                            remaining_boxes.remove(box)
                            break  

    f()

    for storage in remaining_storages:
        min_distance = float('inf')
        nearest_box = None
        
        for box in remaining_boxes:
            distance = calculate_manhattan_distance_with_obstacles(box, storage, state.obstacles)
            if distance < min_distance:
                min_distance = distance
                nearest_box = box
        
        assignments[nearest_box] = (storage, min_distance)
        if nearest_box is not None:
            remaining_boxes.remove(nearest_box)
    
    return assignments

def is_at_edge(pos, width, height):
    if pos[0] == 0:
        edge_type = "left"
    elif pos[0] == width - 1:
        edge_type = "right"
    elif pos[1] == 0:
        edge_type = "bottom"
    elif pos[1] == height - 1:
        edge_type = "top"
    else:
        return False, None  # Not an edge box

    return True, edge_type






### END OF ALTERATE HEURISTICS ###
def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    boxes = state.boxes 
    storage_points = state.storage 

    total_distance = 0 

    for box_pos in boxes: 
        min_distance = math.inf 
        for storage_pos in storage_points:
            distance = abs(box_pos[0] - storage_pos[0]) + abs(box_pos[1] - storage_pos[1])
            if distance < min_distance:
                min_distance = distance
        total_distance += min_distance
    
    return total_distance

def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, timebound, weight):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    # construct a search enginne
    se = SearchEngine('custom', 'default')

    wrapped_fval_function = lambda sN: fval_function(sN, weight)

   
    se.init_search(initial_state, sokoban_goal_state, heur_fn,
                       wrapped_fval_function)

    search_start_time = os.times()[0]
    search_stop_time = search_start_time + timebound

    solution = None
    
    goal_state, stats = se.search(search_stop_time - os.times()[0])
    if goal_state:
        solution = goal_state
            
    
    if not solution:
        return False, stats

    return solution, stats




def iterative_astar(initial_state, heur_fn, weight=10, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    best_solution = None
    best_cost = float('inf')
    current_weight = weight
    se = SearchEngine(strategy='custom', cc_level='default')

    search_start_time = os.times()[0]
    search_stop_time = search_start_time + timebound

    costbound = (float('inf'), float('inf'), float('inf'))

    while os.times()[0] <= search_stop_time:
        se.set_strategy('custom', cc='default')

        wrapped_fval_function = (lambda sN: fval_function(sN, current_weight))

        se.init_search(initial_state, sokoban_goal_state, heur_fn,
                       wrapped_fval_function)

        costbound = (float('inf'), float('inf'), best_cost)

        # Perform Weighted A* search within the remaining timebound
        goal_state, stats = se.search(search_stop_time - os.times()[0], costbound)
        if goal_state:
            # Update the best solution if a solution is found
            best_solution = goal_state
            best_cost = best_solution.gval
            # best_cost = wrapped_fval_function


        # Update the weight for the next iteration (but make sure it's never below 1)
        current_weight = max(current_weight / 2, 1)


        # Break  loop if there are no nodes left to expand
        if not goal_state:
            break

    return best_solution, stats

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    best_solution = None
    best_g_value = float('inf')
    se = SearchEngine(strategy='best_first', cc_level='none')

    search_start_time = os.times()[4]
    search_stop_time = search_start_time + timebound

    while os.times()[4] <= search_stop_time:
        se.set_strategy('best_first', cc='default')

        se.init_search(initial_state, sokoban_goal_state, heur_fn)

        costbound = [best_g_value, float('inf'), float('inf')]

        gbfs_start_time = os.times()[4]

        goal_state, stats = se.search(search_stop_time - gbfs_start_time, costbound)

        if goal_state:
            # Update to the best solution if a solution is found
            best_solution = goal_state
            best_g_value = goal_state.gval

        if not goal_state:
            break

    return best_solution, stats



