import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd


# files = {"Robot 1": "../robot/results/behavior_time_ROBOT_1.csv", "Robot 2": "../robot/results/behavior_time_ROBOT_2.csv"}
# files = {
#     "Robot 1": "../robot/results/behavior_time_ROBOT_1_pyramid_3_robots.csv",
#     "Robot 2": "../robot/results/behavior_time_ROBOT_2_pyramid_3_robots.csv",
# }
# files = {
#     "Robot 1": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_1_robot.csv",
# }
# files = {
#     "Robot 1": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_2_robot.csv",
#     "Robot 2": "../robot/results/behavior_time_ROBOT_2_plane_10x10x1_2_robot.csv",
# }
# files = {
#     "Robot 1": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_3_robot.csv",
#     "Robot 2": "../robot/results/behavior_time_ROBOT_2_plane_10x10x1_3_robot.csv",
#     "Robot 3": "../robot/results/behavior_time_ROBOT_3_plane_10x10x1_3_robot.csv",
# }
files = {
    "Robot 1": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_4_robot.csv",
    "Robot 2": "../robot/results/behavior_time_ROBOT_2_plane_10x10x1_4_robot.csv",
    "Robot 3": "../robot/results/behavior_time_ROBOT_3_plane_10x10x1_4_robot.csv",
    "Robot 4": "../robot/results/behavior_time_ROBOT_4_plane_10x10x1_4_robot.csv",
}


# fig.add_trace(
#     go.Scatterpolar(
#         r=[1, 5, 2, 2, 3], theta=["a", "b", "c", "d", "e"], fill="toself", name="Robot 1"
#     )
# )
#
# fig.add_trace(
#     go.Scatterpolar(
#         r=[5, 1, 1, 4, 3], theta=["a", "b", "c", "d", "e"], fill="toself", name="Robot 2"
#     )
# )


def read_behavior_time_files_helper(filename, graphname, fig, row=1, col=1):
    data = pd.read_csv(filename, header=0)
    data = data.tail(1).to_dict()

    labels = list(data.keys())
    labels.remove("Timestamp")
    values = []
    for label in labels:
        value = float([x for x in data[label].values()][0])
        values.append(value)
    fig.add_trace(
        go.Scatterpolar(r=values, theta=labels, fill="toself", name=graphname,),
        row=row,
        col=col,
    )


def read_behavior_time_files(files, fig, row=1, col=1):
    for graphname in files:
        read_behavior_time_files_helper(files[graphname], graphname, fig, row, col)


def plot_radar_behavior_times(files, fig, row=1, col=1):
    read_behavior_time_files(files, fig, row, col)

    fig.update_layout(
        title="Behavior Time for Robots (seconds)",
        polar=dict(
            radialaxis=dict(visible=True, title="Seconds"),
            angularaxis=dict(rotation=55, direction="clockwise"),
        ),
        polar2=dict(
            radialaxis=dict(visible=True, title="Seconds"),
            angularaxis=dict(rotation=55, direction="clockwise"),
        ),
        polar3=dict(
            radialaxis=dict(visible=True, title="Seconds"),
            angularaxis=dict(rotation=55, direction="clockwise"),
        ),
        polar4=dict(
            radialaxis=dict(visible=True, title="Seconds"),
            angularaxis=dict(rotation=55, direction="clockwise"),
        ),
        showlegend=True,
        template="plotly_dark",
    )
    return fig
    # fig.show()


import dash
import dash_core_components as dcc
import dash_html_components as html

# fig = go.Figure()
# plot_radar_behavior_times(files, fig)
# fig.show()


batch_files = [
    (
        1,
        1,
        {
            "Robot 1 (1 robot)": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_1_robot.csv",
        },
    ),
    (
        1,
        2,
        {
            "Robot 1 (2 robots)": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_2_robot.csv",
            "Robot 2 (2 robots)": "../robot/results/behavior_time_ROBOT_2_plane_10x10x1_2_robot.csv",
        },
    ),
    (
        2,
        1,
        {
            "Robot 1 (3 robots)": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_3_robot.csv",
            "Robot 2 (3 robots)": "../robot/results/behavior_time_ROBOT_2_plane_10x10x1_3_robot.csv",
            "Robot 3 (3 robots)": "../robot/results/behavior_time_ROBOT_3_plane_10x10x1_3_robot.csv",
        },
    ),
    (
        2,
        2,
        {
            "Robot 1 (4 robots)": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_4_robot.csv",
            "Robot 2 (4 robots)": "../robot/results/behavior_time_ROBOT_2_plane_10x10x1_4_robot.csv",
            "Robot 3 (4 robots)": "../robot/results/behavior_time_ROBOT_3_plane_10x10x1_4_robot.csv",
            "Robot 4 (4 robots)": "../robot/results/behavior_time_ROBOT_4_plane_10x10x1_4_robot.csv",
        },
    ),
]


fig = make_subplots(
    rows=2,
    cols=2,
    specs=[
        [{"type": "scatterpolar"}, {"type": "scatterpolar"}],
        [{"type": "scatterpolar"}, {"type": "scatterpolar"}],
    ],
    subplot_titles=("1 Robot", "2 Robots", "3 Robots", "4 Robots"),
)
for row, col, batch in batch_files:
    plot_radar_behavior_times(batch, fig, row, col)
fig.show()


# app = dash.Dash()
# app.layout = html.Div(
#     [html.Div([dcc.Graph(figure=fig)]), html.Div([dcc.Graph(figure=fig)])]
# )
#
# app.run_server(debug=True)
