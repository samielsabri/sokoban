def heur_alternate_2(state):
    
    total_distance = 0
    
    
    boxes = set(state.boxes)
    storages = set(state.storage)
    obstacles = set(state.obstacles)
    width = state.width
    height = state.height

    edge_storages = set()
    for storage in storages:
        if storage[0] == 0 or storage[0] == width - 1 or storage[1] == 0 or storage[1] == height - 1:
            edge_storages.add(storage)
            storages.remove(storage)
    

    edge_boxes = set()
    for box in boxes:
        if box[0] == 0 or box[0] == width - 1 or box[1] == 0 or box[1] == height - 1:
            edge_boxes.add(box)
            boxes.remove(box)
    

    for edge_box in edge_boxes:
        if is_box_deadlocked(edge_box, obstacles, storages, height, width):
            total_distance = 50
        min_distance = float('inf')
        nearest_storage = None
        for edge_storage in edge_storages:
            if edge_storage[0] == edge_box[0] or edge_storage[1] == edge_box[1]:
                # distance = calculate_manhattan_distance(bo, stoarage)
                distance = calculate_manhattan_distance_with_obstacles(box, edge_storage, obstacles)
                if distance < min_distance:
                    min_distance = distance
                    nearest_storage = edge_storage

        edge_storages.remove(nearest_storage)
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