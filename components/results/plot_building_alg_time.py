import plotly.graph_objects as go
import pandas as pd
import numpy as np

# files = {
#     "Plane_10x10x1 (100 Blocks)": [
#         "../simulator/results/plane/simulator_results_plane_1_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_2_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_3_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_4_robots.csv",
#     ],
#     "Plane_20x20x1 (400 Blocks)": [
#         "../simulator/results/plane/simulator_results_plane_4_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_3_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_2_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_1_robots.csv",
#     ],
#     "Plane_50x50x1 (2500 Blocks)": [
#         "../simulator/results/plane/simulator_results_plane_1_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_4_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_2_robots.csv",
#         "../simulator/results/plane/simulator_results_plane_3_robots.csv",
#     ],
# }
files = {
    "Plane_10x10x1 (100 Blocks)": [
        "../simulator/results/plane/simulator_results_plane_10x10x1_1_robots.csv",
        "../simulator/results/plane/simulator_results_plane_10x10x1_2_robots.csv",
        "../simulator/results/plane/simulator_results_plane_10x10x1_3_robots.csv",
        "../simulator/results/plane/simulator_results_plane_10x10x1_4_robots.csv",
    ]
}

files = {
    "Pyramid_10x10x4 (316 Blocks)": [
        "../simulator/results/pyramid/simulator_results_pyramid_10x10x4_1_robots.csv",
        "../simulator/results/pyramid/simulator_results_pyramid_10x10x4_2_robots.csv",
        "../simulator/results/pyramid/simulator_results_pyramid_10x10x4_3_robots.csv",
        "../simulator/results/pyramid/simulator_results_pyramid_10x10x4_4_robots.csv",
    ]
}


def get_building_alg_times(files, fig, name):
    number_of_robots = [x + 1 for x in range(len(files))]
    building_alg_times = []
    for file in files:
        data = pd.read_csv(file, header=0)
        data = data.tail(1).to_dict()
        value = float([x for x in data["Number of timesteps"].values()][0])
        building_alg_times.append(value)

    fig.add_trace(
        go.Scatter(x=number_of_robots, y=building_alg_times, mode="lines", name=name)
    )
    return building_alg_times


def plot_building_alg_times(fig, files, experiment_name):

    for graphname in files:
        get_building_alg_times(files[graphname], fig, graphname)

    fig.update_layout(
        title=f"Time Required to Build Structure",
        template="plotly_dark",
        xaxis_title="Number of robots",
        yaxis_title="Number of timesteps",
        annotations=[
            dict(
                x=0,
                y=1.07,
                showarrow=False,
                text=experiment_name,
                xref="paper",
                yref="paper",
            )
        ],
        showlegend=True,
	xaxis = dict(
      		range=[1,4],  # sets the range of xaxis
    	),
	#yaxis = dict(
      	#	range=[0,65000],  # sets the range of xaxis
    	#),
    )
    fig.update_xaxes(tick0=0, dtick=1.0)

    return fig
    # fig.show()

fig = go.Figure()
plot_building_alg_times(fig, files, experiment_name="")
fig.show()