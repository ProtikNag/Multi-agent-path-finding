import numpy as np
import random
import os

# Constants
rows, cols = 20, 20  # Grid size
min_agents, max_agents = 2, 50  # Number of agents
blocked_ratio = 0.6  # Percentage of blocked cells

# Create Data Folder if it doesn't exist
os.makedirs("./Data", exist_ok=True)


def generate_grid(rows, cols, blocked_ratio):
    """Generate a grid with blocked and available cells."""
    grid = np.full((rows, cols), '.', dtype=str)
    num_blocked = int(blocked_ratio * rows * cols)
    blocked_positions = random.sample([(i, j) for i in range(rows) for j in range(cols)], num_blocked)

    for i, j in blocked_positions:
        grid[i][j] = '@'

    return grid


def is_valid_path(grid, start, goal):
    """Check if a valid path exists using BFS."""
    from collections import deque

    rows, cols = grid.shape
    queue = deque([start])
    visited = set()
    visited.add(start)

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4 possible moves

    while queue:
        x, y = queue.popleft()

        if (x, y) == goal:
            return True

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited and grid[nx][ny] == '.':
                visited.add((nx, ny))
                queue.append((nx, ny))

    return False


def generate_valid_positions(grid, num_agents):
    """Generate valid start and goal positions for agents."""
    available_positions = [(i, j) for i in range(rows) for j in range(cols) if grid[i][j] == '.']
    agents = []

    while len(agents) < num_agents:
        start, goal = random.sample(available_positions, 2)
        if is_valid_path(grid, start, goal):
            agents.append((start, goal))
            available_positions.remove(start)
            available_positions.remove(goal)

    return agents


def save_to_file(grid, agents, file_path):
    """Save the generated grid and agents data to a file."""
    with open(file_path, 'w') as f:
        f.write(f"{rows} {cols}\n")
        for row in grid:
            f.write(" ".join(row) + "\n")
        f.write(f"{len(agents)}\n")
        for (sx, sy), (gx, gy) in agents:
            f.write(f"{sx} {sy} {gx} {gy}\n")


num_files = 100

for i in range(1, num_files + 1):
    grid = generate_grid(rows, cols, blocked_ratio)
    num_agents = random.randint(min_agents, max_agents)
    agents = generate_valid_positions(grid, num_agents)

    # Create file name based on number of agents
    file_path = os.path.join("Data", f"test_{i}_{num_agents}.txt")
    save_to_file(grid, agents, file_path)
