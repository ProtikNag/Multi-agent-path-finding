import heapq
import copy
from itertools import count

class ConflictBasedSearch:
    def __init__(self, grid, start_positions, goal_positions):
        self.grid = grid
        self.start_positions = start_positions
        self.goal_positions = goal_positions
        self.num_agents = len(start_positions)
        self.counter = count()  # A counter to add unique index for each node in the heap

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

    def a_star(self, start, goal, constraints, agent_id):
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
                time_step = new_cost

                # Check if the move is constrained
                if not self.is_constrained(current, neighbor, time_step, constraints, agent_id):
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, new_cost, neighbor))
                        came_from[neighbor] = current

        return []  # Return empty path if no valid path found

    def is_constrained(self, current, next_node, time_step, constraints, agent_id):
        # Check vertex and edge constraints specific to this agent
        if (agent_id, next_node, time_step) in constraints['vertex']:
            return True
        if (agent_id, current, next_node, time_step) in constraints['edge']:
            return True
        return False

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def find_solution(self):
        # Initial solution by planning independently for each agent
        root_constraints = {'vertex': set(), 'edge': set()}
        root_solution = []
        for agent_id in range(self.num_agents):
            path = self.a_star(self.start_positions[agent_id], self.goal_positions[agent_id], root_constraints, agent_id)
            if not path:
                return None
            root_solution.append(path)

        root_node = {
            'constraints': root_constraints,
            'solution': root_solution,
            'cost': sum(len(path) for path in root_solution)
        }

        open_set = []
        heapq.heappush(open_set, (root_node['cost'], next(self.counter), root_node))

        while open_set:
            _, _, current_node = heapq.heappop(open_set)
            solution = current_node['solution']

            # Check for collisions
            collision = self.get_first_collision(solution)
            if collision is None:
                # No collision found, return solution
                return solution

            # Resolve collision by adding constraints
            agent_a, agent_b, position, time_step = collision
            new_constraints = [
                (agent_a, position, time_step),  # Vertex constraint for agent_a
                (agent_b, position, time_step)   # Vertex constraint for agent_b
            ]

            for new_constraint in new_constraints:
                new_node = {
                    'constraints': copy.deepcopy(current_node['constraints']),
                    'solution': copy.deepcopy(current_node['solution'])
                }
                new_node['constraints']['vertex'].add(new_constraint)

                # Replan for the constrained agent
                agent_to_replan = new_constraint[0]
                new_path = self.a_star(
                    self.start_positions[agent_to_replan],
                    self.goal_positions[agent_to_replan],
                    new_node['constraints'],
                    agent_to_replan
                )

                if new_path:
                    new_node['solution'][agent_to_replan] = new_path
                    new_node['cost'] = sum(len(path) for path in new_node['solution'])
                    heapq.heappush(open_set, (new_node['cost'], next(self.counter), new_node))

        return None  # No solution found

    def get_first_collision(self, solution):
        # Check for vertex and edge collisions
        max_path_length = max(len(path) for path in solution)
        for t in range(max_path_length):
            positions_at_time = {}
            for agent_id, path in enumerate(solution):
                position = path[min(t, len(path) - 1)]  # If the path is finished, agent stays at the goal
                if position in positions_at_time:
                    # Vertex collision detected
                    other_agent_id = positions_at_time[position]
                    return (agent_id, other_agent_id, position, t)
                positions_at_time[position] = agent_id

            # Check for edge collisions
            for agent_id, path in enumerate(solution):
                if t < len(path) - 1:
                    current_pos = path[t]
                    next_pos = path[t + 1]
                    for other_agent_id, other_path in enumerate(solution):
                        if other_agent_id != agent_id and t < len(other_path) - 1:
                            other_current = other_path[t]
                            other_next = other_path[t + 1]
                            if current_pos == other_next and next_pos == other_current:
                                # Edge collision detected
                                return (agent_id, other_agent_id, (current_pos, next_pos), t + 1)

        return None

# Example usage
if __name__ == "__main__":
    grid = [[0 for _ in range(5)] for _ in range(5)]
    start_positions = [(0, 0), (4, 4), (0, 4)]
    goal_positions = [(4, 0), (0, 4), (4, 4)]

    cbs = ConflictBasedSearch(grid, start_positions, goal_positions)
    solution = cbs.find_solution()

    if solution:
        for i, path in enumerate(solution):
            print(f"Agent {i} path: {path}")
    else:
        print("No solution found")

