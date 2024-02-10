from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS

def heur_alternate(state):
    
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