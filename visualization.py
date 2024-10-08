import pandas as pd
import matplotlib.pyplot as plt

# Load the three CSV files
priority_data = pd.read_csv('agent_collision_results_priority.csv')
a_star_data = pd.read_csv('agent_collision_results_a_star.csv')
cbs_data = pd.read_csv('agent_collision_results_cbs.csv')

# Sort the data by number of agents to ensure a proper comparison
priority_data = priority_data.sort_values('Num_Agents')
a_star_data = a_star_data.sort_values('Num_Agents')
cbs_data = cbs_data.sort_values('Num_Agents')

# Increase font sizes for the plots
font = {'size': 16}

plt.rc('font', **font)

# Create a line plot comparing Average Time Taken across three approaches
plt.figure(figsize=(10, 6))
plt.plot(priority_data['Num_Agents'], priority_data['Average_Time_Taken'], label='Priority Planning', marker='o')
plt.plot(a_star_data['Num_Agents'], a_star_data['Average_Time_Taken'], label='A*', marker='o')
plt.plot(cbs_data['Num_Agents'], cbs_data['Average_Time_Taken'], label='CBS', marker='o')
plt.title('Comparison of Running Time (A*, Priority Planning, CBS)')
plt.xlabel('Number of Agents')
plt.ylabel('Average Time Taken (s)')
plt.legend()
plt.grid(True)
plt.savefig('./Figures/RunningTime_Comparison.pdf')
plt.savefig('./Figures/RunningTime_Comparison.png')
plt.show()

# Create a line plot comparing Average Collisions across three approaches
plt.figure(figsize=(10, 6))
plt.plot(priority_data['Num_Agents'], priority_data['Average_Collisions'], label='Priority Planning', marker='o')
plt.plot(a_star_data['Num_Agents'], a_star_data['Average_Collisions'], label='A*', marker='o')
plt.plot(cbs_data['Num_Agents'], cbs_data['Average_Collisions'], label='CBS', marker='o')
plt.title('Comparison of Average Collisions (A*, Priority Planning, CBS)')
plt.xlabel('Number of Agents')
plt.ylabel('Average Collisions')
plt.legend()
plt.grid(True)
plt.savefig('./Figures/Collisions_Comparison.pdf')
plt.savefig('./Figures/Collisions_Comparison.png')
plt.show()
