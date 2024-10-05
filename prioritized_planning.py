import heapq

class PrioritizedPlanning:
    def __init__(self, grid, start_positions, goal_positions):
        self.grid = grid
        self.start_positions = start_positions
        self.goal_positions = goal_positions
        self.num_agents = len(start_positions)

    def heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def get_neighbors(self, node):
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])
            if 0 <= neighbor[0] < len(self.grid) and 0 <= neighbor[1] < len(self.grid[0]):
                neighbors.append(neighbor)
        return neighbors

    def a_star(self, start, goal, constraints):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current_cost, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                new_cost = current_cost + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    # Check if the move is within constraints
                    if not self.is_constrained(current, neighbor, current_cost + 1, constraints):
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, new_cost, neighbor))
                        came_from[neighbor] = current

        return []  # Return empty path if no valid path found

    def is_constrained(self, current, next_node, time_step, constraints):
        # Check vertex and edge constraints
        if (next_node, time_step) in constraints['vertex']:
            return True
        if (current, next_node, time_step) in constraints['edge']:
            return True
        return False

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def plan_paths(self):
        # Assign priorities to agents
        agent_priorities = list(range(self.num_agents))
        paths = []
        constraints = {'vertex': set(), 'edge': set()}

        for agent_id in agent_priorities:
            start = self.start_positions[agent_id]
            goal = self.goal_positions[agent_id]

            path = self.a_star(start, goal, constraints)
            if not path:
                print(f"No path found for agent {agent_id}")
                return None

            # Add new path to the list of paths
            paths.append(path)

            # Update constraints to avoid conflicts for future agents
            for t, node in enumerate(path):
                constraints['vertex'].add((node, t))
                if t > 0:
                    prev_node = path[t - 1]
                    constraints['edge'].add((prev_node, node, t))

        return paths