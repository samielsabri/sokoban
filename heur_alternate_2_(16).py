def heur_alternate_2(state):
    
    total_distance = 0
    
    
    boxes = set(state.boxes)
    storages = set(state.storage)
    obstacles = set(state.obstacles)
    width = state.width
    height = state.height


    # dont do next position, look as the state as it is. identify any corners for the boxes for each box, but you
            # cannot precompute
            # but why use the directions then???

    # do an assignment logic: any storage point should be assigned to its nearest box
    # then calculate the distance of each box to its assigned storage point (consider obstacles)
    # consider each storage point and find whether there is a wall attached to it (but why?)

    # boxes = boxes - storages # remove boxes that are already in storage points
    # storages = storages - boxes # remove storage points that already have boxes

    for box in boxes:
        # if is_box_deadlocked(box, obstacles, storages, height, width):
        #     total_distance = 50
        
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



## HELPER FUNCTONS ## 

def calculate_manhattan_distance(start_pos, end_pos):
    return abs(start_pos[0] - end_pos[0]) + abs(start_pos[1] - end_pos[1])

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