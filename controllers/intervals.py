# Plotly Dash modules
import dash_core_components as dcc
import dash_html_components as html

dashboard_intervals = html.Div([
	# TODO: Possibly combine fast and medium intervals into single interval timer
	dcc.Interval(
		id='interval-fast',
		# Too short an interval causes issues as not all plots can be updated before the next callback
		interval=0.1 * 1000,    # Every tenth of a second
		n_intervals=0
	),
	dcc.Interval(
		id='interval-medium',
		interval=0.2 * 1000,    # Every fifth of a second
		n_intervals=0
	),
	dcc.Interval(
		id='interval-slow',
		interval=60 * 1000,     # Every minute
		n_intervals=0
	)
])
