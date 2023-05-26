import numpy as np
import matplotlib.pyplot as plt
import pickle
import os

# Load data from this python files directory
print(os.path.dirname(__file__))

with open(os.path.join(os.path.dirname(__file__), "avg_dist_dict.pkl"), "rb") as f:
	avg_dist_dict = pickle.load(f)

with open(os.path.join(os.path.dirname(__file__), "RMSE_dict.pkl"), "rb") as f:
	RMSE_dict = pickle.load(f)

with open(os.path.join(os.path.dirname(__file__), "vertices_dict.pkl"), "rb") as f:
	vertices_dict = pickle.load(f)

with open(os.path.join(os.path.dirname(__file__), "time_dict.pkl"), "rb") as f:
	time_dict = pickle.load(f)

x_ticks = {"poisson_our" : "Poisson + our normals", "poisson_sensor" : "Poisson + sensor normals", "poisson_o3d" : "Poisson + o3d normals", "our" : "Our method"}

# Print time keys as list
print(list(time_dict.keys()))
# Plot average of times
plt.figure()
plt.title("Average computation time (" + str(len(time_dict["poisson_our"])) + " tests)")
# Plot using x_ticks as x ticks from dict keys
plt.bar([x_ticks[key] for key in time_dict.keys()], [np.mean(time_dict[key]) for key in time_dict.keys()])
# Make the plot look nice by giving pastel colors to each bar
for i, patch in enumerate(plt.gca().patches):
	if i % 2 == 0:
		patch.set_facecolor('xkcd:light blue')
	else:
		patch.set_facecolor('xkcd:light green')
plt.xticks(rotation=20)
plt.ylabel("Time (s)")
plt.tight_layout()
# Set size
plt.gcf().set_size_inches(4, 4)
# ylabel is outside image, we add some space
plt.subplots_adjust(left=0.15, right=0.95, top=0.90, bottom=0.22)
plt.savefig(os.path.join(os.path.dirname(__file__), "avg_time.png"), dpi=300)
plt.show()

# Plot average of vertices
plt.figure()
plt.title("Average number of vertices (" + str(len(vertices_dict["poisson_our"])) + " tests)")
# Plot using x_ticks as x ticks from dict keys
plt.bar([x_ticks[key] for key in vertices_dict.keys()], [np.mean(vertices_dict[key]) for key in vertices_dict.keys()])
# Make the plot look nice by giving pastel colors to each bar
for i, patch in enumerate(plt.gca().patches):
	if i % 2 == 0:
		patch.set_facecolor('xkcd:light blue')
	else:
		patch.set_facecolor('xkcd:light green')
plt.xticks(rotation=20)
plt.ylabel("Vertices")
plt.tight_layout()
# Set size
plt.gcf().set_size_inches(4, 4)
# ylabel is outside image, we add some space
plt.subplots_adjust(left=0.15, right=0.95, top=0.90, bottom=0.22)
plt.savefig(os.path.join(os.path.dirname(__file__), "avg_vertices.png"), dpi=300)
plt.show()

# Plot average of RMSE and avg_dist in the same plot
plt.figure()
plt.title("Average RMSE and distance (" + str(len(RMSE_dict["poisson_our"])) + " tests)")
# create grouped bar plot
barWidth = 0.25
# Set position of bar on X axis
r1 = np.arange(len(RMSE_dict.keys()))
r2 = [x + barWidth for x in r1]
# Make the plot look nice by giving pastel colors to each bar
plt.bar(r1, [np.mean(RMSE_dict[key]) for key in RMSE_dict.keys()], color='xkcd:light blue', width=barWidth, edgecolor='white', label='RMSE')
plt.bar(r2, [np.mean(avg_dist_dict[key]) for key in avg_dist_dict.keys()], color='xkcd:light green', width=barWidth, edgecolor='white', label='Avg. dist.')
# Add xticks on the middle of the group bars
plt.xlabel('Method', fontweight='bold')
plt.xticks([r + barWidth/2 for r in range(len(RMSE_dict.keys()))], [x_ticks[key] for key in RMSE_dict.keys()], rotation=20)
# Create legend & Show graphic
plt.legend()
plt.ylabel("RMSE / Avg. dist.")
plt.tight_layout()
# Set size
plt.gcf().set_size_inches(4, 4)
# ylabel is outside image, we add some space
plt.subplots_adjust(left=0.15, right=0.95, top=0.90, bottom=0.22)
plt.savefig(os.path.join(os.path.dirname(__file__), "avg_RMSE_avg_dist.png"), dpi=300)
plt.show()
