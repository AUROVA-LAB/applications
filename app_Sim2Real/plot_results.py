"""Generate a grouped bar chart comparing methods across metrics."""

import numpy as np
import matplotlib.pyplot as plt


# Replace these example values with the results to compare.
methods = ["Sim-to-Real", "Real-to-Real"]
bar_colors = ['tab:blue', 'tab:orange']
# scenes=np.array(["1","2","3","4"])
# metrics = {
# 	"DWT": np.array([[91.11,57.59,65.38,64.22],[79.28,78.64,53.83,53.74]]),
# 	"Time Ratio": np.array([[0.888,0.894,0.916,0.926],[0.991,0.942,1,0.967]]),
# 	"Stops": np.array([[7,8,2,2],[2,3,1,2]]),
# }


# def plot_results(method_names, metric_values):
#     """Plot a grouped bar chart from a metric-name to values mapping."""
#     if not metric_values:
#         raise ValueError("metric_values must contain at least one metric")
#     if any(len(values) != len(method_names) for values in metric_values.values()):
#         raise ValueError("Each metric must have one value per method")

#     metric_names = list(metric_values)
#     values = [np.array(metric_values[name]) for name in metric_names]
#     font = {'size'   : 22}

#     plt.rc('font', **font)

#     fig, ax = plt.subplots(1,3,figsize=(10, 6))
#     for i, value in enumerate(values):
#         ax[i].plot(scenes, value[0],color=bar_colors[0],label=method_names[0])
#         ax[i].plot(scenes, value[1],color=bar_colors[1],label=method_names[1])
        
#         ax[i].spines[['right', 'top']].set_visible(False)
#         fig.tight_layout()
#         ax[i].grid(axis="y", linestyle="--", alpha=0.4)
#     # ax[i].legend()
#     plt.show()


metrics = {
	"DWT": np.array([[63.99,70.38,66.71],[50.11,44.09,49.43]]),
	"Time Ratio": np.array([[0.96,0.98,0.93],[1,0.97,0.99]]),
	"Stops": np.array([[4,3,5],[2,3,2]]),
}


def plot_results(method_names, metric_values):
    """Plot a grouped bar chart from a metric-name to values mapping."""
    if not metric_values:
        raise ValueError("metric_values must contain at least one metric")
    if any(len(values) != len(method_names) for values in metric_values.values()):
        raise ValueError("Each metric must have one value per method")

    metric_names = list(metric_values)
    values = [np.array(metric_values[name]) for name in metric_names]
    font = {'size'   : 22}

    plt.rc('font', **font)

    fig, ax = plt.subplots(1,3,figsize=(10, 6))
    for i, value in enumerate(values):
        print(value)
        bplot = ax[i].boxplot(value.T,
                   patch_artist=True,  # fill with color
                   labels=method_names, medianprops=dict(linestyle='-',linewidth=2.0, color="yellowgreen"))  # will be used to label x-ticks

        # fill with colors
        for patch, color in zip(bplot['boxes'], bar_colors):
            patch.set_facecolor(color)
        # ax[i].legend()
    plt.show()



if __name__ == "__main__":
	plot_results(methods, metrics)
	
