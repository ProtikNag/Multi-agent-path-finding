# import heapq
#
# # Constants for movement in 4 directions (up, down, left, right, wait)
# DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]  # Added (0, 0) for wait action
#
#
# def heuristic(a, b):
#     """Manhattan distance heuristic function."""
#     return abs(a[0] - b[0]) + abs(a[1] - b[1])
#
#
# def is_constrained(agent_id, position, time, constraints):
#     """Check if a given position at a specific time step is constrained for the agent."""
#     for constraint in constraints:
#         if constraint['agent'] == agent_id:
#             if constraint['type'] == 'vertex' and constraint['loc'] == position and constraint['time'] == time:
#                 return True
#             if constraint['type'] == 'edge' and constraint['loc'] == position and constraint['time'] == time:
#                 return True
#     return False
#
#
# def a_star(grid, start, goal, constraints, agent_id):
#     """A* algorithm considering agent-specific constraints and allowing wait actions."""
#     rows, cols = len(grid), len(grid[0])
#     open_list = []
#     heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start, []))  # (f, g, position, path)
#     g_score = {start: 0}
#
#     while open_list:
#         _, current_g, current, path = heapq.heappop(open_list)
#
#         # If reached the goal, return the path
#         if current == goal:
#             return path + [current]
#
#         for direction in DIRECTIONS:
#             neighbor = (current[0] + direction[0], current[1] + direction[1])
#             if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == '.':
#                 tentative_g = current_g + 1
#
#                 # Check if the move violates any constraints
#                 if is_constrained(agent_id, neighbor, tentative_g, constraints):
#                     continue  # Skip this neighbor if it violates constraints
#
#                 if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                     g_score[neighbor] = tentative_g
#                     f_score = tentative_g + heuristic(neighbor, goal)
#                     heapq.heappush(open_list, (f_score, tentative_g, neighbor, path + [current]))
#
#     return None  # No valid path found
#
#
# def detect_collision(path1, path2):
#     """Detect collisions between two paths."""
#     for t in range(max(len(path1), len(path2))):
#         pos1 = path1[t] if t < len(path1) else path1[-1]
#         pos2 = path2[t] if t < len(path2) else path2[-1]
#
#         # Vertex collision
#         if pos1 == pos2:
#             return {'type': 'vertex', 'loc': pos1, 'time': t}
#
#         # Edge collision
#         if t > 0:
#             prev_pos1 = path1[t - 1] if t - 1 < len(path1) else path1[-2]
#             prev_pos2 = path2[t - 1] if t - 1 < len(path2) else path2[-2]
#             if pos1 == prev_pos2 and pos2 == prev_pos1:
#                 return {'type': 'edge', 'loc': [prev_pos1, pos1], 'time': t}
#
#     return None  # No collision detected
#
#
# def add_constraints(collision, agent_id, constraints):
#     """Add new constraints for agents involved in the collision."""
#     if collision['type'] == 'vertex':
#         constraints.append({'agent': agent_id, 'type': 'vertex', 'loc': collision['loc'], 'time': collision['time']})
#     elif collision['type'] == 'edge':
#         constraints.append({'agent': agent_id, 'type': 'edge', 'loc': collision['loc'][0], 'time': collision['time']})
#
#
# def find_solution(grid, agents):
#     """Find a collision-free path for all agents using CBS, allowing waiting."""
#     constraints = []
#     paths_by_agents = []
#
#     # Initial pathfinding for all agents
#     print("Initial paths:")
#     for i, (start, goal) in enumerate(agents):
#         path = a_star(grid, start, goal, constraints, i)
#         paths_by_agents.append(path)
#         print(f"Agent {i + 1}: {path}")
#
#     # Detect collisions and resolve them
#     collision_count = 0
#     while True:
#         # Detect collisions between agents' paths
#         collision = None
#         for i in range(len(paths_by_agents)):
#             for j in range(i + 1, len(paths_by_agents)):
#                 collision = detect_collision(paths_by_agents[i], paths_by_agents[j])
#                 if collision:
#                     collision_count += 1
#                     print(f"Collision detected between Agent {i + 1} and Agent {j + 1} at time {collision['time']}!")
#                     break
#             if collision:
#                 break
#
#         if not collision:
#             break  # No collisions, return paths
#
#         # Add new constraints for the agents involved in the collision
#         add_constraints(collision, i, constraints)
#         add_constraints(collision, j, constraints)
#
#         # Replan for the involved agents
#         for agent_id in [i, j]:
#             start, goal = agents[agent_id]
#             path = a_star(grid, start, goal, constraints, agent_id)
#             paths_by_agents[agent_id] = path
#
#     print(f"\nNumber of collisions detected: {collision_count}")
#     print("\nPaths after resolving collisions (CBS):")
#     for idx, path in enumerate(paths_by_agents):
#         print(f"Agent {idx + 1}: {path}")
#
#
# # Example grid and agents setup
# grid = [
#     ['.', '.', '.', '.', '.'],
#     ['.', '@', '@', '.', '.'],
#     ['.', '.', '.', '.', '.'],
#     ['.', '.', '@', '.', '.'],
#     ['.', '.', '.', '.', '.']
# ]
#
# agents = [((0, 3), (2, 4)), ((0, 4), (3, 1))]
#
# # Run CBS to find paths
# find_solution(grid, agents)


import os
import time
import csv
from tqdm import tqdm
import heapq

# Constants for movement in 4 directions (up, down, left, right, wait)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]  # Added (0, 0) for wait action


def heuristic(a, b):
    """Manhattan distance heuristic function."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def is_constrained(agent_id, position, time, constraints):
    """Check if a given position at a specific time step is constrained for the agent."""
    for constraint in constraints:
        if constraint['agent'] == agent_id:
            if constraint['type'] == 'vertex' and constraint['loc'] == position and constraint['time'] == time:
                return True
            if constraint['type'] == 'edge' and constraint['loc'] == position and constraint['time'] == time:
                return True
    return False


def a_star(grid, start, goal, constraints, agent_id):
    """A* algorithm considering agent-specific constraints and allowing wait actions."""
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start, []))  # (f, g, position, path)
    g_score = {start: 0}

    while open_list:
        _, current_g, current, path = heapq.heappop(open_list)

        # If reached the goal, return the path
        if current == goal:
            return path + [current]

        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == '.':
                tentative_g = current_g + 1

                # Check if the move violates any constraints
                if is_constrained(agent_id, neighbor, tentative_g, constraints):
                    continue  # Skip this neighbor if it violates constraints

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, tentative_g, neighbor, path + [current]))

    return None  # No valid path found


def detect_collision(path1, path2):
    """Detect collisions between two paths."""
    if path1 is None or path2 is None:
        return None  # If one of the paths is None, skip collision detection

    for t in range(max(len(path1), len(path2))):
        pos1 = path1[t] if t < len(path1) else path1[-1]
        pos2 = path2[t] if t < len(path2) else path2[-1]

        # Vertex collision
        if pos1 == pos2:
            return {'type': 'vertex', 'loc': pos1, 'time': t}

        # Edge collision
        if t > 0:
            prev_pos1 = path1[t - 1] if t - 1 < len(path1) else path1[-2]
            prev_pos2 = path2[t - 1] if t - 1 < len(path2) else path2[-2]
            if pos1 == prev_pos2 and pos2 == prev_pos1:
                return {'type': 'edge', 'loc': [prev_pos1, pos1], 'time': t}

    return None  # No collision detected


def add_constraints(collision, agent_id, constraints):
    """Add new constraints for agents involved in the collision."""
    if collision['type'] == 'vertex':
        constraints.append({'agent': agent_id, 'type': 'vertex', 'loc': collision['loc'], 'time': collision['time']})
    elif collision['type'] == 'edge':
        constraints.append({'agent': agent_id, 'type': 'edge', 'loc': collision['loc'][0], 'time': collision['time']})


def find_solution(grid, agents):
    """Find a collision-free path for all agents using CBS, allowing waiting."""
    constraints = []
    paths_by_agents = []
    collision_count = 0

    # Initial pathfinding for all agents
    for i, (start, goal) in enumerate(agents):
        path = a_star(grid, start, goal, constraints, i)
        # If no path is found, the agent will wait at its starting position
        if path is None:
            path = [start] * 100  # Make the agent wait for a large number of timesteps
        paths_by_agents.append(path)

    # Detect collisions and resolve them
    while True:
        # Detect collisions between agents' paths
        collision = None
        for i in range(len(paths_by_agents)):
            for j in range(i + 1, len(paths_by_agents)):
                collision = detect_collision(paths_by_agents[i], paths_by_agents[j])
                if collision:
                    collision_count += 1
                    break
            if collision:
                break

        if not collision:
            break  # No collisions, return paths

        # Add new constraints for the agents involved in the collision
        add_constraints(collision, i, constraints)
        add_constraints(collision, j, constraints)

        # Replan for the involved agents
        for agent_id in [i, j]:
            start, goal = agents[agent_id]
            path = a_star(grid, start, goal, constraints, agent_id)
            if path is None:
                path = [start] * 100  # Make the agent wait if no path found
            paths_by_agents[agent_id] = path

    return collision_count


def read_file(file_path):
    """Read the grid and agent data from the file."""
    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Parse the grid size
    rows, cols = map(int, lines[0].split())

    # Parse the grid
    grid = [list(line.strip().split()) for line in lines[1:rows + 1]]

    # Parse the agents
    num_agents = int(lines[rows + 1].strip())
    agents = []
    for line in lines[rows + 2:rows + 2 + num_agents]:
        sx, sy, gx, gy = map(int, line.split())
        agents.append(((sx, sy), (gx, gy)))

    return grid, agents


def process_files(data_folder, output_file):
    results = []

    for file_name in tqdm(os.listdir(data_folder)):
        if file_name.endswith(".txt"):
            file_path = os.path.join(data_folder, file_name)
            grid, agents = read_file(file_path)
            num_agents = len(agents)

            # Run CBS for all agents and measure time
            start_time = time.time()
            collision_count = find_solution(grid, agents)
            time_taken = time.time() - start_time

            # Record results
            results.append((num_agents, time_taken, collision_count))

    # Write results to CSV file
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Num_Agents", "Average_Time_Taken", "Average_Collisions"])
        for num_agents, time_taken, collision_count in results:
            writer.writerow([num_agents, time_taken, collision_count])


# Main processing
data_folder = "./Data"
output_csv = "agent_collision_results_cbs.csv"

# Process files and gather results
process_files(data_folder, output_csv)

print(f"Results saved to {output_csv}")
