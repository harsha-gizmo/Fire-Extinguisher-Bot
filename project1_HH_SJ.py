import random
import matplotlib.pyplot as plt
import seaborn as sns
from collections import deque
from queue import PriorityQueue
import copy
import heapq
from concurrent.futures import ProcessPoolExecutor
import time
# Constants
GRID_SIZE = 40
TARGET_OPEN_PERCENTAGE = 0.6
SIMULATION_COUNT = 50
Q_VALUES = [round(i * 0.1, 1) for i in range(1, 11)]  # Generates [0.1, 0.2, ..., 1.0]
max_cores = 20

# Create empty grid layout
def create_grid(size):
    return [['blocked' for _ in range(size)] for _ in range(size)]

# Pick random starting cell in grid
def start_with_initial_cell(grid):
    x, y = random.randint(1, GRID_SIZE - 2), random.randint(1, GRID_SIZE - 2)
    grid[x][y] = 'open'
    return (x, y)

# Gets blocked cells with exactly one open neighbour
def get_candidates_for_opening(grid):
    candidates = []
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if grid[row][col] == 'blocked':
                open_neighbors = sum(
                    grid[nr][nc] == 'open'
                    for nr, nc in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]
                    if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE
                )
                if open_neighbors == 1:
                    candidates.append((row, col))
    return candidates

# Open the candidate cells retrieved in above function iteratively
def open_cells(grid):
    while True:
        candidates = get_candidates_for_opening(grid)
        if not candidates:
            break
        cell_to_open = random.choice(candidates)
        grid[cell_to_open[0]][cell_to_open[1]] = 'open'

def find_dead_ends(grid):
    dead_ends = []
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if grid[row][col] == 'open':
                open_neighbors = sum(
                    grid[nr][nc] == 'open'
                    for nr, nc in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]
                    if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE
                )
                if open_neighbors == 1:
                    dead_ends.append((row, col))
    return dead_ends

def expand_dead_ends(grid, dead_ends):
    selected_dead_ends = random.sample(dead_ends, len(dead_ends) // 2)
    for dead_end in selected_dead_ends:
        row, col = dead_end
        blocked_neighbors = [
            (nr, nc)
            for nr, nc in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and grid[nr][nc] == 'blocked'
        ]
        if blocked_neighbors:
            neighbor_to_open = random.choice(blocked_neighbors)
            grid[neighbor_to_open[0]][neighbor_to_open[1]] = 'open'

# Check if 60 percent of grid open before expanding dead ends
def check_sixty_percent(grid):
    open_count = sum(row.count('open') for row in grid)
    total_cells = GRID_SIZE * GRID_SIZE
    return open_count >= TARGET_OPEN_PERCENTAGE * total_cells

# Wrapper function for creating grid layout
def create_space_vessel_layout():
    while True:
        grid = create_grid(GRID_SIZE)
        start_with_initial_cell(grid)
        open_cells(grid)

        if check_sixty_percent(grid):
            dummy = None
        else:
            print("60 percent not open, trying again")
            continue

        dead_ends = find_dead_ends(grid)
        expand_dead_ends(grid, dead_ends)

        break
                
    return grid

# Spread fire function called iteratively to spread fire to open cells
def spread_fire(grid, fire_cells, q):
    new_grid = copy.deepcopy(grid)
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if grid[row][col] == 'open' and (row, col) not in fire_cells:
                K = sum(1 for nr, nc in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]
                        if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and (nr, nc) in fire_cells)
                spread_probability = 1 - (1 - q) ** K
                if random.random() < spread_probability:
                    new_grid[row][col] = 'fire'
    
    new_fire_cells = fire_cells.union({(row, col) for row in range(GRID_SIZE) for col in range(GRID_SIZE) if new_grid[row][col] == 'fire'})
    return new_grid, new_fire_cells

# Bot1 BFS algorithm to get shortest path to goal avoiding initial fire cell
def bfs_shortest_path(grid, start, goal, fire_cells):
    queue = deque([(start, [start])])
    visited = set([start])

    while queue:
        current, path = queue.popleft()
        if current == goal:
            return path

        for nr, nc in [(current[0]-1, current[1]), (current[0]+1, current[1]), (current[0], current[1]-1), (current[0], current[1]+1)]:
            if (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and
                grid[nr][nc] == 'open' and (nr, nc) not in visited and (nr, nc) not in fire_cells):
                visited.add((nr, nc))
                new_path = path + [(nr, nc)]
                queue.append(((nr, nc), new_path))

    return None  # No path found 

# Bot2 UCS algorithm to get shortest path to goal avoiding current fire cells
def ucs_shortest_path(grid, start, goal, fire_cells):
    queue = PriorityQueue()
    queue.put((0, start))  # (cost, position)
    visited = set()
    parent_map = {start: None}
    cost_so_far = {start: 0}

    while not queue.empty():
        cost, current = queue.get()
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parent_map[current]
            return path[::-1]

        for nr, nc in [(current[0]-1, current[1]), (current[0]+1, current[1]), (current[0], current[1]-1), (current[0], current[1]+1)]:
            if (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and
                grid[nr][nc] == 'open' and (nr, nc) not in visited and (nr, nc) not in fire_cells):
                
                new_cost = cost_so_far[current] + 1
                if (nr, nc) not in cost_so_far or new_cost < cost_so_far[(nr, nc)]:
                    cost_so_far[(nr, nc)] = new_cost
                    parent_map[(nr, nc)] = current
                    # Tie-breaking: add Manhattan distance to goal
                    priority = new_cost + manhattan_distance((nr, nc), goal)
                    queue.put((priority, (nr, nc)))

    return None  # No path found

def get_adjacent_cells(position):
    row, col = position
    return [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]

# BFS Algorithm to check if a solution is possible in a grid layout considering the initial button and bot states
def is_success_possible(grid, start, goal, fire_start):
    queue = deque([start])
    visited = set([start])
    while queue:
        current = queue.popleft()
        if current == goal:
            return True
        for nr, nc in [(current[0]-1, current[1]), (current[0]+1, current[1]), (current[0], current[1]-1), (current[0], current[1]+1)]:
            if (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and
                grid[nr][nc] == 'open' and (nr, nc) not in visited and (nr, nc) != fire_start):
                visited.add((nr, nc))
                queue.append((nr, nc))
    return False

# Heuristic for Bot3 A* Algorithm
def astar_heuristic(a, b, fire_cells, adjacent_fire_cells, avoid_adjacent=True):
    manhattan = manhattan_distance(a, b)
    fire_distance = min((manhattan_distance(a, fire) for fire in fire_cells), default=GRID_SIZE)
    # Only calculate the adjacent fire penalty if avoid_adjacent is True
    if avoid_adjacent:
        adjacent_fire_distance = min((manhattan_distance(a, adj) for adj in adjacent_fire_cells), default=GRID_SIZE)
    else:
        adjacent_fire_distance = GRID_SIZE  # Set to a high value so there's no penalty

    fire_penalty = max(0, 10 - fire_distance) * 3
    firefront_penalty = max(0, 5 - adjacent_fire_distance) * 2
    return manhattan + fire_penalty + firefront_penalty

# Bot3 A* algorithm for Bot3 with heuristic to avoid adjacent cells to fire as well
def astar_shortest_path(grid, start, goal, fire_cells, avoid_adjacent=True):
    queue = PriorityQueue()
    queue.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    
    adjacent_fire_cells = set()
    for fire_cell in fire_cells:
        adjacent_fire_cells.update(get_adjacent_cells(fire_cell))
    adjacent_fire_cells -= fire_cells

    while not queue.empty():
        _, current = queue.get()

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for next_pos in get_adjacent_cells(current):
            if (0 <= next_pos[0] < GRID_SIZE and 0 <= next_pos[1] < GRID_SIZE and
                grid[next_pos[0]][next_pos[1]] == 'open' and next_pos not in fire_cells):
                if avoid_adjacent and next_pos in adjacent_fire_cells:
                    continue  # Avoid adjacent cells if specified
                new_cost = cost_so_far[current] + 1
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + astar_heuristic(next_pos, goal, fire_cells, adjacent_fire_cells, avoid_adjacent)
                    queue.put((priority, next_pos))
                    came_from[next_pos] = current

    return None

def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Bot4 function to predict fire to adjacent cells of current fire cells
def predict_fire_spread(grid, fire_cells, prediction_steps=2):
    predicted_fire = set(fire_cells)
    for _ in range(prediction_steps):
        new_fire = set()
        for cell in predicted_fire:
            for nr, nc in [(cell[0]-1, cell[1]), (cell[0]+1, cell[1]), (cell[0], cell[1]-1), (cell[0], cell[1]+1)]:
                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and grid[nr][nc] == 'open':
                    new_fire.add((nr, nc))
        predicted_fire.update(new_fire)
    return predicted_fire

# Bot4 Risk assessment function
def assess_risk(pos, fire_cells, predicted_fire):
    if not fire_cells:
        return 0  # No risk if there's no fire
    nearest_fire = min(manhattan_distance(pos, fire) for fire in fire_cells.union(predicted_fire))
    return 1 - (nearest_fire / (GRID_SIZE * 2))  # Normalize risk between 0 and 1

# Bot4 - calculate safe paths to goal avoiding fire cells and predicted fire cells using BFS
def calculate_safety_corridor(grid, start, goal, fire_cells, predicted_fire):
    corridor = set()
    queue = [(start, [start])]
    visited = set([start])
    
    while queue:
        current, path = queue.pop(0)
        if current == goal:
            corridor.update(path)
            continue
        
        for nr, nc in [(current[0]-1, current[1]), (current[0]+1, current[1]), (current[0], current[1]-1), (current[0], current[1]+1)]:
            if (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and
                grid[nr][nc] == 'open' and (nr, nc) not in visited and
                (nr, nc) not in fire_cells and (nr, nc) not in predicted_fire):
                new_path = path + [(nr, nc)]
                queue.append(((nr, nc), new_path))
                visited.add((nr, nc))
                
    return corridor

# Bot4 main path finding function
def bot4_pathfinding(grid, start, goal, fire_cells, predicted_fire, safety_corridor):
    def heuristic(a, b):
        manhattan = manhattan_distance(a, b)
        if fire_cells:
            fire_avoidance = min(manhattan_distance(a, fire) for fire in fire_cells.union(predicted_fire)) * 0.5
            corridor_bonus = 5 if a in safety_corridor else 0
            return manhattan - fire_avoidance - corridor_bonus
        return manhattan

    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in [(current[0]-1, current[1]), (current[0]+1, current[1]), (current[0], current[1]-1), (current[0], current[1]+1)]:
            if (0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and
                grid[neighbor[0]][neighbor[1]] == 'open' and neighbor not in fire_cells):
                
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None

# Simulation functions for each bot which calls the Algorithm functions

def run_simulation_bfs(q, possible_cases = False):
    grid = create_space_vessel_layout()
    open_cells_list = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid[r][c] == 'open']
    bot_start = random.choice(open_cells_list)
    button_location = random.choice([cell for cell in open_cells_list if cell != bot_start])
    fire_start = random.choice([cell for cell in open_cells_list if cell != bot_start and cell != button_location])
    if possible_cases:
        while(True):
            if not is_success_possible(grid, bot_start, button_location, fire_start):
                print("Success not possible in layout based on initial positions. Recreating layout ...")
                grid = create_space_vessel_layout()
            else:
                break
    fire_cells = {fire_start}
    grid[fire_start[0]][fire_start[1]] = 'fire'
    current_pos = bot_start
    
    while current_pos != button_location:
        # Plan the shortest path, avoiding current fire cells
        path = bfs_shortest_path(grid, current_pos, button_location, fire_cells)
        
        if not path:
            return False  # No path found

        # Move to the next position in the path
        next_pos = path[1] if len(path) > 1 else current_pos
        current_pos = next_pos

        if current_pos == button_location:
            return True
        # Spread fire after moving
        grid, fire_cells = spread_fire(grid, fire_cells, q)
        if current_pos in fire_cells:
            return False  # Bot got caught in fire

    return True  # Reached the button

def run_simulation_ucs(q, possible_cases = False):
    grid = create_space_vessel_layout()
    open_cells_list = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid[r][c] == 'open']
    bot_start = random.choice(open_cells_list)
    button_location = random.choice([cell for cell in open_cells_list if cell != bot_start])
    fire_start = random.choice([cell for cell in open_cells_list if cell != bot_start and cell != button_location])
    if possible_cases:
        # print("Considering possible cases ...")
        while(True):
            if not is_success_possible(grid, bot_start, button_location, fire_start):
                print("Success not possible in layout based on initial positions. Recreating layout ...")
                grid = create_space_vessel_layout()
            else:
                break
    fire_cells = {fire_start}
    grid[fire_start[0]][fire_start[1]] = 'fire'
    current_pos = bot_start

    while current_pos != button_location:
        path = ucs_shortest_path(grid, current_pos, button_location, fire_cells)
        if not path:
            return False
        
        next_pos = path[1] if len(path) > 1 else current_pos
        
        current_pos = next_pos

        if current_pos == button_location:
            return True

        grid, fire_cells = spread_fire(grid, fire_cells, q)
        if current_pos in fire_cells:
            return False

    return True

def run_simulation_astar(q, possible_cases = False):
    grid = create_space_vessel_layout()
    open_cells_list = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid[r][c] == 'open']
    bot_start = random.choice(open_cells_list)
    button_location = random.choice([cell for cell in open_cells_list if cell != bot_start])
    fire_start = random.choice([cell for cell in open_cells_list if cell != bot_start and cell != button_location])
    if possible_cases:
        # print("Considering possible cases ...")
        while(True):
            if not is_success_possible(grid, bot_start, button_location, fire_start):
                print("Success not possible in layout based on initial positions. Recreating layout ...")
                grid = create_space_vessel_layout()
            else:
                break
    fire_cells = {fire_start}
    grid[fire_start[0]][fire_start[1]] = 'fire'
    current_pos = bot_start

    while current_pos != button_location:
        # Try to find a path avoiding fire cells and adjacent cells
        path = astar_shortest_path(grid, current_pos, button_location, fire_cells, avoid_adjacent=True)
        
        # If no path found, try again avoiding only fire cells
        if not path:
            path = astar_shortest_path(grid, current_pos, button_location, fire_cells, avoid_adjacent=False)
        
        if not path:
            return False  # No path found even when only avoiding fire cells

        next_pos = path[1] if len(path) > 1 else current_pos
        current_pos = next_pos

        if current_pos == button_location:
            return True

        grid, fire_cells = spread_fire(grid, fire_cells, q)
        if current_pos in fire_cells:
            return False
        
    return True

def run_simulation_bot4(q, possible_cases = False):
    grid = create_space_vessel_layout()
    open_cells_list = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid[r][c] == 'open']
    bot_start = random.choice(open_cells_list)
    button_location = random.choice([cell for cell in open_cells_list if cell != bot_start])
    fire_start = random.choice([cell for cell in open_cells_list if cell != bot_start and cell != button_location])
    if possible_cases:
        while(True):
            if not is_success_possible(grid, bot_start, button_location, fire_start):
                print("Success not possible in layout based on initial positions. Recreating layout ...")
                grid = create_space_vessel_layout()
            else:
                break
    fire_cells = {fire_start}
    grid[fire_start[0]][fire_start[1]] = 'fire'
    current_pos = bot_start
    steps = 0

    while current_pos != button_location:
        if steps > GRID_SIZE * 2:  # Prevent infinite loops
            return False

        predicted_fire = predict_fire_spread(grid, fire_cells)
        safety_corridor = calculate_safety_corridor(grid, current_pos, button_location, fire_cells, predicted_fire)
        path = bot4_pathfinding(grid, current_pos, button_location, fire_cells, predicted_fire, safety_corridor)

        if not path:
            return False

        next_pos = path[1] if len(path) > 1 else current_pos
        
        # Assess risk with a specific randomness logic
        risk_level = assess_risk(next_pos, fire_cells, predicted_fire)
        
        if risk_level > 0.7:
            # Introduce randomness: 30% chance to take a risky path
            if random.random() < 0.3:
                next_pos = random.choice(list(safety_corridor)) if safety_corridor else current_pos
            else:
                next_pos = current_pos  # Stay in place if not taking the risky path
        elif risk_level > 0.3:  # Medium risk
            if next_pos not in safety_corridor:
                next_pos = current_pos  # Stay in place if next move is not in safety corridor
        
        current_pos = next_pos
        steps += 1

        if current_pos == button_location:
            return True

        grid, fire_cells = spread_fire(grid, fire_cells, q)
        if current_pos in fire_cells:
            return False

    return True

def main():
    sns.set(style="whitegrid")
    success_rates_bfs = []
    success_rates_ucs = []
    success_rates_astar = []
    success_rates_bot4 = []
    for q in Q_VALUES:
        print(f"Running BFS simulations for q={q}")
        success_count_bfs = 0
        # Using ThreadPoolExecutor to run simulations concurrently
        with ProcessPoolExecutor(max_workers=max_cores) as executor:
            # Submit all the BFS simulation tasks to the thread pool
            futures = [executor.submit(run_simulation_bfs, q) for _ in range(SIMULATION_COUNT)]

            # Manually check completion of each future in a loop
            while futures:
                for future in futures[:]:  # Iterate over a copy of the list
                    if future.done():  # Check if the future is completed
                        if future.result():  # If the simulation was successful
                            success_count_bfs += 1
                        futures.remove(future)  # Remove the future from the list once it's done

                # Sleep for a short time to avoid a busy loop
                time.sleep(2)
        
        success_rate_bfs = success_count_bfs / SIMULATION_COUNT
        success_rates_bfs.append(success_rate_bfs)
        print(f"Success rate for BFS with q={q}: {success_rate_bfs:.2f}")

    for q in Q_VALUES:
        print(f"Running UCS simulations for q={q}")
        success_count_ucs = 0

        # Using ThreadPoolExecutor to run simulations concurrently
        with ProcessPoolExecutor(max_workers=max_cores) as executor:
            # Submit all the BFS simulation tasks to the thread pool
            futures = [executor.submit(run_simulation_ucs, q) for _ in range(SIMULATION_COUNT)]

            # Manually check completion of each future in a loop
            while futures:
                for future in futures[:]:  # Iterate over a copy of the list
                    if future.done():  # Check if the future is completed
                        if future.result():  # If the simulation was successful
                            success_count_ucs += 1
                        futures.remove(future)  # Remove the future from the list once it's done

                # Sleep for a short time to avoid a busy loop
                time.sleep(2)

        success_rate_ucs = success_count_ucs / SIMULATION_COUNT
        success_rates_ucs.append(success_rate_ucs)
        print(f"Success rate for UCS with q={q}: {success_rate_ucs:.2f}")

    for q in Q_VALUES:
        print(f"Running A* simulations for q={q}")
        success_count_astar = 0
        # Using ThreadPoolExecutor to run simulations concurrently
        with ProcessPoolExecutor(max_workers=max_cores) as executor:
            # Submit all the BFS simulation tasks to the thread pool
            futures = [executor.submit(run_simulation_astar, q) for _ in range(SIMULATION_COUNT)]

            # Manually check completion of each future in a loop
            while futures:
                for future in futures[:]:  # Iterate over a copy of the list
                    if future.done():  # Check if the future is completed
                        if future.result():  # If the simulation was successful
                            success_count_astar += 1
                        futures.remove(future)  # Remove the future from the list once it's done

                # Sleep for a short time to avoid a busy loop
                time.sleep(2)

        success_rate_astar = success_count_astar / SIMULATION_COUNT
        success_rates_astar.append(success_rate_astar)
        print(f"Success rate for A* with q={q}: {success_rate_astar:.2f}")

    for q in Q_VALUES:
        print(f"Running Bot4 simulations for q={q}")
        success_count_bot4 = 0
        # Using ThreadPoolExecutor to run simulations concurrently
        with ProcessPoolExecutor(max_workers=max_cores) as executor:
            # Submit all the BFS simulation tasks to the thread pool
            futures = [executor.submit(run_simulation_bot4, q) for _ in range(SIMULATION_COUNT)]

            # Manually check completion of each future in a loop
            while futures:
                for future in futures[:]:  # Iterate over a copy of the list
                    if future.done():  # Check if the future is completed
                        if future.result():  # If the simulation was successful
                            success_count_bot4 += 1
                        futures.remove(future)  # Remove the future from the list once it's done

                # Sleep for a short time to avoid a busy loop
                time.sleep(2)
        
        success_rate_bot4 = success_count_bot4 / SIMULATION_COUNT
        success_rates_bot4.append(success_rate_bot4)
        print(f"Success rate for Bot4 with q={q}: {success_rate_bot4:.2f}")

    plt.figure(figsize=(12, 8))
    plt.plot(Q_VALUES, success_rates_bfs, marker='o', linestyle='-', label='BFS', color='blue')
    plt.plot(Q_VALUES, success_rates_ucs, marker='s', linestyle='-', label='UCS', color='orange')
    plt.plot(Q_VALUES, success_rates_astar, marker='^', linestyle='-', label='A*', color='green')
    plt.plot(Q_VALUES, success_rates_bot4, marker='D', linestyle='-', label='Bot 4', color='red')
    
    plt.title('Success Rate of Algorithms vs Flammability', fontsize=16)
    plt.xlabel('Flammability (q)', fontsize=14)
    plt.ylabel('Average Success Rate', fontsize=14)
    plt.xticks(Q_VALUES)
    plt.ylim(0, 1)
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()