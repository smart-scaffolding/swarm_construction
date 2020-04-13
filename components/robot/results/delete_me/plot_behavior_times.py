import plotly.graph_objects as go
import pandas as pd


files = {"Robot 1": "behavior_time_ROBOT_1.csv", "Robot 2": "behavior_time_ROBOT_2.csv"}


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


def read_behavior_time_files_helper(filename, graphname, fig):
    data = pd.read_csv(filename, header=0)
    data = data.tail(1).to_dict()

    labels = list(data.keys())
    labels.remove("Timestamp")
    values = []
    for label in labels:
        value = float([x for x in data[label].values()][0])
        values.append(value)
    fig.add_trace(go.Scatterpolar(r=values, theta=labels, fill="toself", name=graphname,))


def read_behavior_time_files(files, fig):
    for graphname in files:
        read_behavior_time_files_helper(files[graphname], graphname, fig)


def plot_radar_behavior_times(files):
    fig = go.Figure()
    read_behavior_time_files(files, fig)

    fig.update_layout(
        title="Behavior Time for Robots",
        polar=dict(radialaxis=dict(visible=True, range=[0, 5], title="Seconds"),),
        showlegend=True,
        template="plotly_dark",
    )
    # return fig
    fig.show()


# import dash
# import dash_core_components as dcc
# import dash_html_components as html
fig = plot_radar_behavior_times(files)
#
# app = dash.Dash()
# app.layout = html.Div(
#     [
#         dcc.Graph(figure=fig)
#         ]
#     )
#
# app.run_server(debug=True)
