import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from components.results.plot_behavior_times import plot_radar_behavior_times
from components.results.plot_building_alg_time import plot_building_alg_times

radar_files = [
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


radar = make_subplots(
    rows=2,
    cols=2,
    specs=[
        [{"type": "scatterpolar"}, {"type": "scatterpolar"}],
        [{"type": "scatterpolar"}, {"type": "scatterpolar"}],
    ],
    subplot_titles=("1 Robot", "2 Robots", "3 Robots", "4 Robots"),
)
for row, col, batch in radar_files:
    plot_radar_behavior_times(batch, radar, row, col)



building_alg_files = {
    "Robot 1": "../robot/results/behavior_time_ROBOT_1_plane_10x10x1_4_robot.csv",
    "Robot 2": "../robot/results/behavior_time_ROBOT_2_plane_10x10x1_4_robot.csv",
    "Robot 3": "../robot/results/behavior_time_ROBOT_3_plane_10x10x1_4_robot.csv",
    "Robot 4": "../robot/results/behavior_time_ROBOT_4_plane_10x10x1_4_robot.csv",
}

building_alg_fig = go.Figure()

plot_building_alg_times(building_alg_fig, building_alg_files, experiment_name="")


app = dash.Dash()
app.layout = html.Div(
    [html.Div([dcc.Graph(figure=radar)]), html.Div([dcc.Graph(figure=building_alg_fig)])]
)

app.run_server(debug=True)