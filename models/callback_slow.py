# Plotly Dash modules
from dash.dependencies import Input, Output
import plotly.graph_objs as go

# MiRo dashboard modules
from app import app
import constants as con

# MiRo interface modules
from models.basic_functions import miro_ros_interface as mri

# Initialise MiRo clients
miro_core = mri.MiRoCore()


@app.callback(
	Output('circadian-graph', 'figure'),
	[Input('interval-slow', 'n_intervals')]
)
def callback_slow(_):
	# Initialise output data dictionary
	output = {}

	# Circadian graph
	if miro_core.time is not None:
		circ_input = miro_core.time
	else:
		circ_input = 0

	# # TODO: Make circadian clock display leading zeroes
	# # TODO: Update clock to have two hands and show accurate time
	# circ_hrs = range(0, 24)
	# # circ_hrs = ['{:02d}'.format(item) for item in range(0, 24)]
	#
	# # Set clock hand width and length
	# hand_width = 2
	# hand_length = 0.9

	# TODO: Disable polar plot zoom
	# circ_data = [
	# 	go.Scatterpolar(
	# 		fill='toself',
	# 		fillcolor='steelblue',
	# 		marker={
	# 			'line': {
	# 				'color': 'black',
	# 				'width': 0.5
	# 			}
	# 		},
	# 		mode='lines',
	# 		name='Time',
	# 		r=[0, 0.1, hand_length, 0.1, 0],
	# 		theta=[
	# 			0,
	# 			circ_hrs[circ_input - hand_width],
	# 			circ_hrs[circ_input],
	# 			circ_hrs[circ_input + hand_width],
	# 			0
	# 		]
	# 	)
	# ]

	circ_axis = {
		'fixedrange'    : True,
		'linewidth'     : 0,
		'mirror'        : True,
		'range'         : [0, 1],
		'showgrid'      : False,
		'showticklabels': False,
		'zeroline'      : False,
	}

	circ_layout = go.Layout(
		images=[{
			'opacity': 1,
			'sizing' : 'contain',
			'sizex'  : 1,
			'sizey'  : 1,
			'source' : con.ASSET_PATH + 'clock_' + str(circ_input) + '.png',
			'x'      : 0.5,
			'y'      : 0.5,
			'xanchor': 'center',
			'yanchor': 'middle'
		}],
		margin={
			'b': 0,
			'l': 0,
			'r': 0,
			't': 0
		},
		xaxis=circ_axis,
		yaxis=circ_axis,
		# polar={
		# 	'angularaxis': {
		# 		'categoryarray': circ_hrs,
		# 		'direction'    : 'clockwise',
		# 		'nticks'       : 8,
		# 		'period'       : 15,
		# 		'rotation'     : 270,
		# 		'showgrid'     : False,
		# 		'type'         : 'category',
		# 	},
		# 	'radialaxis' : {
		# 		'range'     : [0, 1],
		# 		'visible'   : False
		# 	},
		# },
		showlegend=False
	)

	output['circadian-graph'] = {
		# 'data'  : circ_data,
		'data'  : None,
		'layout': circ_layout
	}

	# Return all outputs
	return output['circadian-graph']