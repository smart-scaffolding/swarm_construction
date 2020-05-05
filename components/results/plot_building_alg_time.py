import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
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

    trendline = curve_fit(lambda t, a, b: a*np.exp(
        b*t),  np.array(number_of_robots),  np.array(building_alg_times))
    x = np.linspace(1.0, float(len(files)), num=200)
    y = 9.27294187e+04 * np.exp(-4.49696822e-01 * x)
    print(trendline)
    fig.add_trace(
        go.Scatter(x=number_of_robots, y=building_alg_times,
                   mode="markers+text",
                   name=name,
                   line=dict(width=10),
                   marker=dict(size=30,
                               line=dict(
                                   width=2,
                               )
                               ),
                   #    trendline="lowess"
                   ),

    )
    fig.add_trace(
        go.Scatter(x=x, y=y,
                   #    mode="lines+markers+text",
                   name=name,
                   line=dict(width=10),
                   marker=dict(size=18,
                               line=dict(
                                   width=2,
                               )
                               ),
                   #    trendline="lowess"
                   ),

    )
    return building_alg_times


def plot_building_alg_times(fig, files, experiment_name):

    for graphname in files:
        get_building_alg_times(files[graphname], fig, graphname)

    fig.update_layout(
        title=f"Time to Build Pyramid (316 Blocks)",
        # template="plotly_dark",
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
                # titlefont=dict(size=25)
                font=dict(
                    family="Helvetica, monospace",
                    size=16,
                ),
            )
        ],
        showlegend=False,
        xaxis=dict(
            range=[1, 4],  # sets the range of xaxis
            titlefont=dict(size=32),
            tickfont=dict(size=40)
        ),
        yaxis=dict(
            range=[0, 65000],  # sets the range of xaxis
            titlefont=dict(size=32),
            tickfont=dict(size=40)
        ),
        titlefont=dict(size=35, family="Helvetica, monospace",),
        # paper_bgcolor='rgba(0,0,0,0)',
        # plot_bgcolor='rgba(0,0,0,0)',
        width=1200,


    )
    fig.update_xaxes(tick0=0, dtick=1.0)

    return fig
    # fig.show()


fig = go.Figure()
plot_building_alg_times(
    fig, files, experiment_name="")
fig.show()
