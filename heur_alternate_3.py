def heur_alternate_3(state):
    
    total_distance = 0
    
    
    boxes = set(state.boxes)
    storages = set(state.storage)
    obstacles = set(state.obstacles)
    width = state.width
    height = state.height

    assignments = assign_storage_to_boxes(state) # create dictionary of box to storage point assignments

    for box, (storage, distance) in assignments.items():
        if is_box_deadlocked(box, obstacles, height, width):
            return float('inf')

        total_distance += distance

    return total_distance


def assign_storage_to_boxes(state):
    '''Assign each storage point to its nearest box.'''
    assignments = {}
    remaining_storages = set(state.storage)
    remaining_boxes = set(state.boxes)
    
    
    def f():
        for box in state.boxes:
            is_edge, edge_type = is_edge_box(box, state.width, state.height)
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

def is_edge_box(box_pos, width, height):
    if box_pos[0] == 0:
        edge_type = "left"
    elif box_pos[0] == width - 1:
        edge_type = "right"
    elif box_pos[1] == 0:
        edge_type = "bottom"
    elif box_pos[1] == height - 1:
        edge_type = "top"
    else:
        return False, None  # Not an edge box

    return True, edge_type


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
