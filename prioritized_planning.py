import heapq
import time
import os
import csv
import random
import numpy as np
from tqdm import tqdm

# Constants for movement in 4 directions (up, down, left, right, wait)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]


def heuristic(a, b):
    """Manhattan distance heuristic function."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star(grid, start, goal, blocked_cells, agent_paths):
    """A* algorithm to find the shortest path, considering blocked cells and proximity to other agents."""
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))  # (f, g, position)
    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, current_g, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == '.' and neighbor not in blocked_cells:
                tentative_g = current_g + 1

                # Proximity penalty: Add a cost if neighbor is close to already planned paths
                proximity_penalty = 0
                for path in agent_paths:
                    if neighbor in path or any(heuristic(neighbor, step) <= 1 for step in path):
                        proximity_penalty += 5  # Adjust the penalty based on how strict you want it

                tentative_g += proximity_penalty

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, tentative_g, neighbor))
                    came_from[neighbor] = current

    return None  # No path found


def simulate_execution_with_collisions(agent_paths):
    """Simulate the execution of all agent paths and count both vertex and edge collisions."""
    time_steps = max(len(path) for path in agent_paths)
    positions = [set() for _ in range(time_steps)]  # Track positions at each time step
    edge_moves = [set() for _ in range(time_steps)]  # Track edge swaps
    collision_count = 0

    for path in agent_paths:
        for t, pos in enumerate(path):
            # Check for vertex collisions
            if pos in positions[t]:
                collision_count += 1
            positions[t].add(pos)

            # Check for edge collisions (agents swapping places between two cells)
            if t > 0:
                prev_pos = path[t - 1]
                move = (prev_pos, pos)
                reverse_move = (pos, prev_pos)
                if reverse_move in edge_moves[t - 1]:
                    collision_count += 1
                edge_moves[t - 1].add(move)

    return collision_count


def prioritized_planning(grid, agents):
    """Prioritized planning: plan paths for agents, check for collisions, and insert wait actions if necessary."""
    agent_paths = []
    blocked_cells = set()

    # Calculate distances for priority assignment
    agents_with_distances = []
    for start, goal in agents:
        distance = heuristic(start, goal)
        agents_with_distances.append((distance, start, goal))

    # Sort agents by the calculated distance (shorter distance gets higher priority)
    agents_with_distances.sort(key=lambda x: x[0])

    # Plan paths in priority order
    for _, start, goal in agents_with_distances:
        # Plan path while considering already blocked cells and agent paths
        path = a_star(grid, start, goal, blocked_cells, agent_paths)

        if path is not None:
            # Check for potential future collisions with previously planned paths
            if simulate_execution_with_collisions([path] + agent_paths) == 0:
                agent_paths.append(path)
                # Add the planned path to blocked cells to avoid collisions for future agents
                blocked_cells.update(path)
            else:
                # Insert a wait action if collision is detected
                path.append(path[-1])  # Add wait at the final position to avoid collision
                agent_paths.append(path)
                blocked_cells.update(path)
        else:
            agent_paths.append([])  # Mark as failure if no path found

    return agent_paths


def read_file(file_path):
    """Read a test file and extract the grid and agent data."""
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


def process_files_with_collisions(data_folder):
    results = []

    for file_name in tqdm(os.listdir(data_folder)):
        if file_name.endswith(".txt"):
            file_path = os.path.join(data_folder, file_name)
            grid, agents = read_file(file_path)
            num_agents = len(agents)

            # Run prioritized planning for all agents and measure time
            start_time = time.time()
            agent_paths = prioritized_planning(grid, agents)
            time_taken = time.time() - start_time

            # Simulate the execution of paths and count collisions
            collision_count = simulate_execution_with_collisions(agent_paths)

            # Record results
            results.append((num_agents, time_taken, collision_count))

    return results


def save_results_to_csv(results, output_file):
    """Save the results into a CSV file."""
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Num_Agents", "Average_Time_Taken", "Average_Collisions"])

        # Aggregate results by number of agents
        aggregated_results = {}
        for num_agents, time_taken, collisions in results:
            if num_agents not in aggregated_results:
                aggregated_results[num_agents] = {"time": [], "collisions": []}
            aggregated_results[num_agents]["time"].append(time_taken)
            aggregated_results[num_agents]["collisions"].append(collisions)

        for num_agents, data in aggregated_results.items():
            avg_time = sum(data["time"]) / len(data["time"])
            avg_collisions = sum(data["collisions"]) / len(data["collisions"])
            writer.writerow([num_agents, avg_time, avg_collisions])


# Main processing
data_folder = "./Data"
output_csv = "agent_collision_results_priority.csv"

# Process files and gather results
results = process_files_with_collisions(data_folder)

# Save the results to CSV
save_results_to_csv(results, output_csv)

print(f"Results saved to {output_csv}")
