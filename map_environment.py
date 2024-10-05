import numpy as np
import json

class MapEnvironment:
    def __init__(self, grid_size, num_agents):
        self.grid_size = grid_size
        self.num_agents = num_agents
        self.grid = np.zeros((grid_size, grid_size))

    def generate_synthetic_dataset(self, num_instances):
        dataset = []
        for _ in range(num_instances):
            start_positions = np.random.choice(self.grid_size**2, self.num_agents, replace=False)
            goal_positions = np.random.choice(self.grid_size**2, self.num_agents, replace=False)
            instance = {
                'start_positions': start_positions.tolist(),
                'goal_positions': goal_positions.tolist()
            }
            dataset.append(instance)
        with open('synthetic_mapf_dataset.json', 'w') as f:
            json.dump(dataset, f)

# Example usage
env = MapEnvironment(10, 5)
env.generate_synthetic_dataset(20)
