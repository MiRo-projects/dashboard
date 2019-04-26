#!/home/dbuxton/mdk/share/python/miro2/dashboard/.venv/bin/python
# -*- coding: utf-8 -*-

# Import Dash components
import dash
import dash_daq as daq
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go

# Import system and ROS components
import rospy
import numpy as np
import miro_interface as mi

##########
# Define line and arrow widths
H_WIDTH = '7px'
V_WIDTH = '5px'
V_HEIGHT = '30px'
A_WIDTH = '10px'
A_HEIGHT = '20px'

##########
# Define custom CSS
css = {
	'arrow_down': {
		'border-left' : A_WIDTH + ' solid white',
		'border-right': A_WIDTH + ' solid white',
		'border-top'  : A_HEIGHT + ' solid black',
		'height'      : 0,
		'margin'      : 'auto',
		'position'    : 'relative',
		'bottom'      : '20px',
		'width'       : 0
	},
	'arrow_left': {
		'border-bottom': A_WIDTH + ' solid white',
		'border-right' : A_HEIGHT + ' solid black',
		'border-top'   : A_WIDTH + ' solid white',
		'float'        : 'left',
		'height'       : 0,
		'margin-top'   : '-12px',
		'width'        : 0
	},
	'arrow_right': {
		'border-bottom': A_WIDTH + ' solid white',
		'border-left'  : A_HEIGHT + ' solid black',
		'border-top'   : A_WIDTH + ' solid white',
		'float'        : 'right',
		'height'       : 0,
		'margin-top'   : '-12px',
		'width'        : 0
	},
	'arrow_right_clear': {
		'border-bottom': A_WIDTH + ' solid transparent',
		'border-left'  : A_HEIGHT + ' solid transparent',
		'border-top'   : A_WIDTH + ' solid transparent',
		'float'        : 'right',
		'height'       : 0,
		'margin-top'   : '-12px',
		'width'        : 0
	},
	'arrow_up': {
		'border-bottom': A_HEIGHT + ' solid black',
		'border-left'  : A_WIDTH + ' solid white',
		'border-right' : A_WIDTH + ' solid white',
		'height'       : 0,
		'margin'       : 'auto',
		'width'        : 0
	},
	'bar': {
		'border'    : '0px',
		'margin'    : '0px',
		'padding'   : '5px',
		'text-align': 'center'
	},
	'line_horizontal': {
		'background-color': 'black',
		'border-bottom'   : '1px white solid',
		'border-top'      : '1px white solid',
		'float'           : 'right',
		'height'          : H_WIDTH,
		'width'           : '100%',
		'margin-top'      : '25px',
	},
	'line_horizontal_clear': {
		'border-bottom'   : '1px white transparent',
		'border-top'      : '1px white transparent',
		'float'           : 'right',
		'height'          : H_WIDTH,
		'width'           : '100%',
		'margin-top'      : '25px',
	},
	'line_horizontal_clear_left': {
		'background-color': 'white',
		'border-right'    : '5px black solid',
		'height'          : '6px',
		'width'           : '52%',
		'position'        : 'absolute',
		'right'           : '48%',
		'top'             : 25
	},
	'line_vertical': {
		'background-color': 'black',
		'height'          : '100%',
		'width'           : V_WIDTH,
		'margin'          : 'auto',
		'min-height'      : V_HEIGHT
	},
}

##########
# Define affect faces
affect_faces = {
	'0.0': {
		'0.0': 'assets/face_frowning.png',
		'0.3': 'assets/face_crying.png',
		'0.6': 'assets/face_crying_loud.png',
		'0.9': 'assets/face_crying_loud.png',
	},
	'0.2': {
		'0.0': 'assets/face_pensive.png',
		'0.3': 'assets/face_frowning_slight.png',
		'0.6': 'assets/face_anguished.png',
		'0.9': 'assets/face_anguished.png',
	},
	'0.4': {
		'0.0': 'assets/face_expressionless.png',
		'0.3': 'assets/face_neutral.png',
		'0.6': 'assets/face_open_mouth.png',
		'0.9': 'assets/face_open_mouth.png',
	},
	'0.6': {
		'0.0': 'assets/face_relieved.png',
		'0.3': 'assets/face_smiling_slight.png',
		'0.6': 'assets/face_grinning.png',
		'0.9': 'assets/face_grinning.png',
	},
	'0.8': {
		'0.0': 'assets/face_smiling.png',
		'0.3': 'assets/face_smiling_eyes.png',
		'0.6': 'assets/face_grinning_eyes.png',
		'0.9': 'assets/face_grinning_eyes.png',
	},
}

sleep_faces = {
	'0.00': 'assets/face_sleeping.png',
	'0.25': 'assets/face_sleepy.png',
	'0.50': 'assets/face_no_mouth.png',
	'0.75': 'assets/face_no_mouth.png',
}

##########
# Define dashboard items
dashboard_alerts = {
	'ball': dbc.Alert(
		"⚽",
		id='ball-alert',
		color='info',
		is_open=False,
		style={
			'font-size' : 'x-large',
			'margin'    : '0px',
			'text-align': 'center'
		}
	),
	'ball_large': dbc.Alert(
		"⚽",
		id='ball-alert-large',
		color='info',
		is_open=False,
		style={
			'font-size' : 'x-large',
			'margin'    : '0px',
			'text-align': 'center'
		}
	),
	'face': dbc.Alert(
		"😀",
		id='face-alert',
		color='success',
		is_open=False,
		style={
			'font-size' : 'x-large',
			'margin'    : '0px',
			'text-align': 'center'
		}
	),
	'face_large': dbc.Alert(
		"😀",
		id='face-alert-large',
		color='success',
		is_open=False,
		style={
			'font-size' : 'x-large',
			'margin'    : '0px',
			'text-align': 'center'
		}
	),
}

dashboard_graphs = {
	# TODO: Add a larger modal including BG model information
	'action': dcc.Graph(
		id='action-graph',
		config={'displayModeBar': False},
		style={
			'height': '150px',
			'width' : '100%',
		}
	),
	'affect': dcc.Graph(
		id='affect-graph',
		# 'Animate' property is incompatible with changing images
		# animate=True,
		config={'displayModeBar': False},
		style={
			# FIXME: Ideally this shouldn't be a hardcoded value
			'height': '400px',
			'width' : '100%',
		}
	),
	'affect_large': dcc.Graph(
		id='affect-graph-large',
		# 'Animate' property is incompatible with changing images
		# animate=True,
		config={'displayModeBar': False},
		style={
			'height': '500px',
			'width' : '500px',
		}
	),
	'aural': dcc.Graph(
		id='aural-graph',
		config={'displayModeBar': False},
		style={'width': '100%'}
	),
	'aural_large': dcc.Graph(
		id='aural-graph-large',
		config={'displayModeBar': False},
		style={'width': '100%'}
	),
	'cameras': dcc.Graph(
		id='camera-graph',
		config={'displayModeBar': False},
		style={'width': '100%'}
	),
	'cameras_large': dcc.Graph(
		id='camera-graph-large',
		config={'displayModeBar': False},
		style={'width': '100%'}
	),
	'circadian': dcc.Graph(
		id='circadian-graph',
		animate=True,
		config={'displayModeBar': False},
		style={
			'height': '100%',
			'width' : '100%',
		}
	),
	'sleep_large': dcc.Graph(
		id='sleep-graph-large',
		# 'Animate' property is incompatible with changing images
		# animate=True,
		config={'displayModeBar': False},
		style={
			'height': '500px',
			'width' : '500px',
		}
	),
}

dashboard_intervals = html.Div([
	# TODO: Move all update callbacks into a single interval check
	dcc.Interval(
		id='interval-fast',
		interval=0.9 * 1000,
		# interval=0.25 * 1000,
		n_intervals=0
	),
	dcc.Interval(
		id='interval-medium',
		interval=1 * 1000,
		n_intervals=0
	),
	dcc.Interval(
		id='interval-slow',
		interval=60 * 1000,
		n_intervals=0
	)
])

dashboard_tools = {
	'cam_toggle': daq.BooleanSwitch(
		id='cam-toggle',
		label='Visual attention',
		labelPosition='bottom',
	),
	'cam_toggle_large': daq.BooleanSwitch(
		id='cam-toggle-large',
		label='Visual attention',
		labelPosition='bottom',
	),
	'affect_button': dbc.Button(
		'+',
		id='affect-modal-open',
		color='info',
		size='sm',
		style={'float': 'right'}
	),
	'action_button': dbc.Button(
		'+',
		id='action-modal-open',
		color='info',
		size='sm',
		style={'float': 'right'}
	),
	'spatial_button': dbc.Button(
		'+',
		id='spatial-modal-open',
		color='info',
		size='sm',
		style={'float': 'right'}
	),
}

dashboard_modals = html.Div([
	dbc.Modal(
		[
			dbc.ModalHeader('Spatial attention'),
			dbc.ModalBody([
				dashboard_graphs['aural_large'],
				dashboard_graphs['cameras_large'],
				dashboard_tools['cam_toggle_large'],
				dashboard_alerts['ball_large'],
				dashboard_alerts['face_large'],
			]),
			dbc.ModalFooter([
				dbc.Button(
					'Close',
					id='spatial-modal-close',
					className='ml-auto'
				)
			]),
		],
		id='spatial-modal',
		centered=True,
		size='xl'
	),
	dbc.Modal(
		[
			dbc.ModalHeader('Affect'),
			dbc.ModalBody([
				dbc.Table(
					[
						html.Tr([
							html.Td(dashboard_graphs['affect_large']),
							html.Td(dashboard_graphs['sleep_large'])
						])
					],
					borderless=True
				)
			]),
			dbc.ModalFooter([
				dbc.Button(
					'Close',
					id='affect-modal-close',
					className='ml-auto'
				)
			]),

		],
		id='affect-modal',
		centered=True,
		size='xl'
	)
])

# As arrows comprise multiple lines spread across multiple rows and columns,
# and each tooltip requires a unique ID, tooltip messages must be repeated to occur along the entire arrow
# Instead, tooltips currently only appear for arrow terminators
dashboard_tooltips = html.Div([
	# Row 1
	dbc.Tooltip(
		'Motor pattern output',
		target='tooltip-top-sum'
	),

	# Row 3
	dbc.Tooltip(
		'Predicted state',
		target='tooltip-motor-action',
	),
	dbc.Tooltip(
		'Spatial priority',
		target='tooltip-spatial-action',
	),
	dbc.Tooltip(
		'Action success / failure',
		target='tooltip-action-affect',
	),
	dbc.Tooltip(
		'Downstream effects on affective state',
		target='tooltip-top-affect',
	),

	# Row 4
	dbc.Tooltip(
		'Spatial bias and inhibition of return',
		target='tooltip-top-spatial',
	),

	# Row 5
	dbc.Tooltip(
		'Current state',
		target='tooltip-sensory-spatial',
	),
	dbc.Tooltip(
		'Base arousal',
		target='tooltip-circadian-affect',
	),
	dbc.Tooltip(
		'Physical state',
		target='tooltip-bottom-affect',
	),

	# Row 6
	dbc.Tooltip(
		'Current pose',
		target='tooltip-sensory-motor',
	),
	dbc.Tooltip(
		'Ears, eyelids, lights, tail, and vocalisation',
		target='tooltip-expression',
	),

	# Row 7
	dbc.Tooltip(
		'Motor efferent',
		target='tooltip-motor-bottom',
	),
	dbc.Tooltip(
		'Motor afferent',
		target='tooltip-bottom-sensory',
	),
	dbc.Tooltip(
		'Light',
		target='tooltip-bottom-circadian',
	),

])

##########
# Define dashboard rows
dashboard_rows = {
	'Row_top': dbc.Row(
		dbc.Col(
			dbc.Alert(
				'⬆ To P3 ⬆',
				color='dark',
				style=css['bar']
			)
		),
		no_gutters=True
	),
	'Row_1': dbc.Row(
		[
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				id='tooltip-top-sum',
				width={
					'size'  : 1,
					'offset': 3
				},
			),
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				width={
					'size'  : 1,
					'offset': 3
				}
			),
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 1
				}
			),
		],
		no_gutters=True
	),
	'Row_2': dbc.Row(
		[
			dbc.Col(
				[
					dbc.Alert(
						'Many up– and downstream connections omitted for clarity',
						color='info'
					)
				],
				width={
					'size'  : 1,
					'offset': 1
				},
			),
			dbc.Col(
				[
					dbc.Card(
						dbc.CardBody(
							dbc.CardText('Σ'),
							style={
								'font-size'  : 'xx-large',
								'font-weight': 'bolder',
								'text-align' : 'center'
							}
						),
						color='light'
					),
					html.Div(style=css['line_vertical']),
				],
				width={
					'size'  : 1,
					'offset': 1
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_left']),
					html.Div(style=css['line_vertical']),
				],
				width={
					'size'  : 1,
					'offset': 0
				},
			),
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader(
							'Action selection',
							# dashboard_tools['action_button']
						),
						dbc.CardBody(dashboard_graphs['action'])
					],
					color='warning',
					outline=True,
				),
				width={
					'size'  : 5,
					'offset': 0
				}
			),
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 1
				}
			),
		],
		no_gutters=True
	),
	'Row_3': dbc.Row(
		[
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 3
				}
			),
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				id='tooltip-motor-action',
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				id='tooltip-spatial-action',
				width={
					'size'  : 1,
					'offset': 1
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				id='tooltip-action-affect',
				width={
					'size'  : 1,
					'offset': 1
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				id='tooltip-top-affect',
				width={
					'size'  : 1,
					'offset': 1
				}
			)
		],
		no_gutters=True
	),
	'Row_4': dbc.Row(
		[
			dbc.Col(
				dbc.Card([
					dbc.CardHeader('Environment'),
					dbc.CardBody(dbc.CardImg(src='/assets/miro_logo.png'))
				]),
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
				],
				width={
					'size'  : 1,
					'offset': 0
				},
			),
			dbc.Col(
				[
					dbc.Card(
						[
							dbc.CardBody([
								dbc.CardText(
									'Motor reafferent',
									style={'font-weight': 'bold'}
								),
								dbc.CardText(html.Hr()),
								dbc.CardText(
									'Noise filter',
									style={'font-weight': 'bold'}
								),
							]),
							dbc.CardFooter(
								'➡ Self-activity reports',
								style={'font-size': 'x-small'}
							)
						],
						color='light'
					),
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_vertical']),
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				html.Div(
					[
						html.Div(style=css['line_horizontal_clear_left']),
						html.Div(style=css['line_horizontal']),
						html.Div(style=css['arrow_right_clear']),
						html.Div(style=css['line_horizontal']),
						html.Div(style=css['arrow_right_clear']),
						html.Div(style=css['line_horizontal']),
						html.Div(style=css['arrow_right_clear']),
						html.Div(style=css['line_vertical']),
					]
				),
				width={
					'size'  : 1,
					'offset': 0
				},
			),
			dbc.Col(
				[
					html.Div(style=css['line_horizontal']),
					html.Div(
						style=css['arrow_right'],
						id='tooltip-top-spatial',
					),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
					# For some reason the standard vertical line results in a 1px offset here
					# html.Div(style=css['line_vertical']),
					html.Div(style={
						'background-color': 'black',
						'height'          : '100%',
						'width'           : V_WIDTH,
						'margin-left'     : '49%',
						'min-height'      : V_HEIGHT
					})
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader(
							[
								'Spatial attention',
								dashboard_tools['spatial_button']
							]
						),
						dbc.CardBody(
							[
								dashboard_graphs['aural'],
								dashboard_graphs['cameras'],
								dashboard_alerts['ball'],
								dashboard_alerts['face'],
							]
						),
						dbc.CardFooter(dashboard_tools['cam_toggle'])
					],
					color='primary',
					outline=True,
					# Some cards are forced to 100% height so that arrows always connect cleanly
					style={'height': '100%'}
				),
				width={
					'size'  : 3,
					'offset': 0
				}
			),
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader([
							'Affect',
							dashboard_tools['affect_button']
						]),
						dbc.CardBody(dashboard_graphs['affect'])
					],
					color='success',
					outline=True,
					style={'height': '100%'}
				),
				width={
					'size'  : 3,
					'offset': 0
				}
			)
		],
		no_gutters=True
	),
	'Row_5': dbc.Row(
		[
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				width={
					'size'  : 1,
					'offset': 3
				}
			),
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 1
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				id='tooltip-sensory-spatial',
				width={
					'size'  : 1,
					'offset': 1
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				id='tooltip-circadian-affect',
				width={
					'size'  : 1,
					'offset': 1
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				id='tooltip-bottom-affect',
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),
		],
		no_gutters=True
	),
	'Row_6': dbc.Row(
		[
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader('Body model (Motor)'),
						dbc.CardBody(
							dbc.CardImg(
								# TODO: Get a more square-proportioned MiRo image
								src='assets/miro_picture.jpg',
								style={'width': '120px'}
							),
							style={'text-align': 'center'}
						),
						dbc.CardFooter(
							'➡ Self-activity reports',
							style={'font-size': 'x-small'}
						)
					],
					style={'height': '100%'}
				),
				width={
					'size'  : 3,
					'offset': 3
				}
			),
			dbc.Col(
				[
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_left']),
				],
				id='tooltip-sensory-motor',
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader('Body model (Sensory)'),
						dbc.CardBody(
							dbc.CardImg(src='assets/miro_picture.jpg'),
							style={'text-align': 'center'}
						),
					],
					style={'height': '100%'}
				),
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader('Circadian rhythm'),
						dbc.CardBody(dashboard_graphs['circadian'])
					],
					# color='secondary',
					# outline=True,
					style={'height': '100%'}
				),
				width={
					'size'  : 2,
					'offset': 0
				}
			),
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader('Expression'),
						dbc.CardBody(
							# TODO: Can probably do better than a dog face for this card
							dbc.CardImg(src='assets/express_dog.png'),
							id='tooltip-expression'
						),
						dbc.CardFooter(
							'➡ Self-activity reports',
							style={'font-size': 'x-small'}
						)
					],
					style={'height': '100%'}
				),
				width={
					'size'  : 1,
					'offset': 0
				}
			),
		],
		no_gutters=True
	),
	'Row_7': dbc.Row(
		[
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				id='tooltip-motor-bottom',
				width={
					'size'  : 1,
					'offset': 4
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				id='tooltip-bottom-sensory',
				width={
					'size'  : 1,
					'offset': 2
				}
			),
			dbc.Col(
				[
					html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				id='tooltip-bottom-circadian',
				width={
					'size'  : 1,
					'offset': 1
				}
			),
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col([
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),
		],
		no_gutters=True
	),
	'Row_btm': dbc.Row(
		dbc.Col(
			dbc.Alert(
				'⬇ To P1 ⬇',
				color='dark',
				style=css['bar']
			)
		),
		no_gutters=True
	)
}

##########
# Define dashboard layout
# See other included themes: https://bootswatch.com
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.MINTY])

# CSS modification needed to remove corner 'undo' button
app.css.append_css({'external_url': 'assets/stylesheet.css'})

# TODO: Find out how to remove 'undo' button in the corner
app.layout = html.Div([
	dashboard_rows['Row_top'],
	dashboard_rows['Row_1'],
	dashboard_rows['Row_2'],
	dashboard_rows['Row_3'],
	dashboard_rows['Row_4'],
	dashboard_rows['Row_5'],
	dashboard_rows['Row_6'],
	dashboard_rows['Row_7'],
	dashboard_rows['Row_btm'],
	dashboard_modals,
	dashboard_tooltips,
	dashboard_intervals
])


##########
# Define dashboard callbacks
@app.callback(
	[Output('ball-alert', 'is_open'), Output('ball-alert-large', 'is_open')],
	[Input('interval-fast', 'n_intervals')]
)
def alert_ball(_):
	if (len(miro_ros_data.core_detect_ball_l.data) > 1) or (len(miro_ros_data.core_detect_ball_r.data) > 1):
		return True, True
	else:
		return False, False


@app.callback(
	[Output('face-alert', 'is_open'), Output('face-alert-large', 'is_open')],
	[Input('interval-fast', 'n_intervals')]
)
def alert_face(_):
	# This is totally untested and assumes the same data structure as ball detection. No idea if it works!
	if (len(miro_ros_data.core_detect_face_l.data) > 1) or (len(miro_ros_data.core_detect_face_r.data) > 1):
		return True, True
	else:
		return False, False


@app.callback(
	Output('affect-modal', 'is_open'),
	[Input('affect-modal-open', 'n_clicks'), Input('affect-modal-close', 'n_clicks')],
	[State('affect-modal', 'is_open')]
)
def modal_affect(n1, n2, is_open):
	if n1 or n2:
		return not is_open

	return is_open


@app.callback(
	Output('spatial-modal', 'is_open'),
	[Input('spatial-modal-open', 'n_clicks'), Input('spatial-modal-close', 'n_clicks')],
	[State('spatial-modal', 'is_open')]
)
def modal_spatial(n1, n2, is_open):
	if n1 or n2:
		return not is_open

	return is_open


@app.callback(Output('action-graph', 'figure'), [Input('interval-fast', 'n_intervals')])
def update_action(_):

	if (miro_ros_data.selection_priority is not None) and (miro_ros_data.selection_inhibition is not None):
		action_inhibition = np.array(miro_ros_data.selection_inhibition.data)
		# Priority is made negative so it appears to the left of the bar chart
		action_priority = np.array([-x for x in miro_ros_data.selection_priority.data])
	else:
		action_inhibition = [0]
		action_priority = [0]

	# TODO: Extract this list automatically from demo code
	action_list = [
		'Mull',
		'Halt',
		'Orient',
		'Approach',
		'Avert',
		'Flee',
		'Retreat'
	]

	layout = go.Layout(
		bargap=0.1,
		barmode='overlay',
		margin={
			'b': 40,
			'l': 60,
			'r': 0,
			't': 0
		},
		xaxis={
			'fixedrange': True,
			'range'     : [-1, 1],
			'ticktext'  : [1, 0.5, 0, 0.5, 1],
			'tickvals'  : [-1, -0.5, 0, 0.5, 1],
			'title'     : 'Salience'
		},
		yaxis={'fixedrange': True},
	)

	data = [
		go.Bar(
			name='Input',
			orientation='h',
			hoverinfo='name+text+y',
			# Format input label to three decimal places
			hovertext=np.round(-action_priority, decimals=3),
			marker={'color': 'mediumseagreen'},
			text=action_list,
			x=action_priority,
			y=action_list,
		),
		go.Bar(
			hoverinfo='none',
			name='Output',
			orientation='h',
			marker={'color': 'silver'},
			x=action_inhibition,
			y=action_list,
		)
	]

	return {
		'data'  : data,
		'layout': layout
	}


@app.callback(
	[
		Output('affect-graph', 'figure'),
		Output('affect-graph-large', 'figure'),
		Output('sleep-graph-large', 'figure')
	],
	[Input('interval-fast', 'n_intervals')]
)
def update_affect(_):

	# Affect axes are the same irrespective of data
	affect_xaxis = {
		'fixedrange'    : True,
		'linewidth'     : 0.5,
		'mirror'        : True,
		'range'         : [0, 1],
		'showgrid'      : False,
		'showticklabels': False,
		'title'         : 'Valence',
		'zeroline'      : False,
	}

	affect_yaxis = {
		'fixedrange'    : True,
		'linewidth'     : 0.5,
		'mirror'        : True,
		'range'         : [0, 1],
		'showgrid'      : False,
		'showticklabels': False,
		'title'         : 'Arousal',
		'zeroline'      : False,
	}

	# Minor changes for sleep graph
	sleep_xaxis = affect_xaxis.copy()
	sleep_yaxis = affect_yaxis.copy()
	sleep_xaxis['title'] = 'Wakefulness'
	sleep_yaxis['title'] = 'Pressure'

	# Layout margins are slightly different without data
	null_layout = go.Layout(
		margin={
			'b': 20,
			'l': 20,
			'r': 5,
			't': 20
		},
		xaxis=affect_xaxis,
		yaxis=affect_yaxis
	)

	# Update affect
	affect_data = miro_ros_data.core_affect
	if affect_data is not None:
		data = {
			'emotion': go.Scatter(
				x=np.array(affect_data.emotion.valence),
				y=np.array(affect_data.emotion.arousal),
				name='Emotion',
				mode='markers',
				opacity=0.7,
				marker={
					'color': 'steelblue',
					'size' : 15,
					'line' : {
						'width': 0.5,
						'color': 'black'
					}
				}
			),

			'mood'   : go.Scatter(
				x=np.array(affect_data.mood.valence),
				y=np.array(affect_data.mood.arousal),
				name='Mood',
				mode='markers',
				opacity=0.7,
				marker={
					'color': 'seagreen',
					'size' : 15,
					'line' : {
						'width': 0.5,
						'color': 'black'
					}
				}
			),

			'sleep'  : go.Scatter(
				x=np.array(affect_data.sleep.wakefulness),
				y=np.array(affect_data.sleep.pressure),
				name='Sleep',
				mode='markers',
				opacity=0.7,
				marker={
					'color': 'salmon',
					'size' : 15,
					'line' : {
						'width': 0.5,
						'color': 'black'
					}
				}
			)
		}

		# Get the appropriate face from the 'faces' dictionary based on current mood values
		for x in np.arange(0, 1, 0.2):
			for y in np.arange(0, 1, 0.3):
				if (x < data['mood'].x <= x + 0.2) and (y < data['mood'].y <= y + 0.3):
					# Round the results to nearest 0.1 to prevent floating point errors; inaccurate but unimportant
					affect_face = affect_faces['{0:.1f}'.format(x)]['{0:.1f}'.format(y)]

		for x in np.arange(0, 1, 0.25):
			if x < data['sleep'].x <= x + 0.25:
				sleep_face = sleep_faces['{0:.2f}'.format(x)]

		# Layout includes background face image and graph legend
		affect_layout = go.Layout(
			images=[{
				'layer'  : 'below',
				'opacity': 0.8,
				'sizing' : 'contain',
				'sizex'  : 0.3,
				'sizey'  : 0.3,
				'source' : affect_face,
				'x'      : 0.5,
				'y'      : 0.5,
				'xanchor': 'center',
				'yanchor': 'middle'
			}],
			legend={
				'orientation': 'h',
				'x'          : 0.5,
				'xanchor'    : 'center',
				'y'          : 1.01,
				'yanchor'    : 'bottom',
			},
			margin={
				'b': 20,
				'l': 20,
				'r': 20,
				't': 20
			},
			showlegend=True,
			xaxis=affect_xaxis,
			yaxis=affect_yaxis
		)

		sleep_layout = go.Layout(affect_layout)
		sleep_layout['xaxis'] = sleep_xaxis
		sleep_layout['yaxis'] = sleep_yaxis
		# TODO: It should be possible to just modify the 'source' attribute but I don't know how
		sleep_layout['images'] = [{
			'layer'  : 'below',
			'opacity': 0.8,
			'sizing' : 'contain',
			'sizex'  : 0.3,
			'sizey'  : 0.3,
			'source' : sleep_face,
			'x'      : 0.5,
			'y'      : 0.5,
			'xanchor': 'center',
			'yanchor': 'middle'
		}]

		affect_figure = {
			'data'  : [
				data['emotion'],
				data['mood'],
				data['sleep']
			],
			'layout': affect_layout
		}

		affect_figure_large = {
			'data'  : [
				data['emotion'],
				data['mood'],
			],
			'layout': affect_layout
		}

		sleep_figure_large = {
			'data'  : [data['sleep']],
			'layout': sleep_layout
		}

		return affect_figure, affect_figure_large, sleep_figure_large

	else:
		return {'layout': null_layout}, {'layout': null_layout}, {'layout': null_layout}


@app.callback(
	[Output('aural-graph', 'figure'), Output('aural-graph-large', 'figure')],
	[Input('interval-medium', 'n_intervals')]
)
def update_aural(_):
	priw = miro_ros_data.core_priw

	# Needs to be updated manually if plot width changes; value includes margins
	p_height = 60
	p_height_large = 80

	# Set image properties
	if priw is not None:
		priw_image = [{
			'layer'  : 'below',
			'opacity': 1,
			'sizing' : 'stretch',
			'sizex'  : 1,
			'sizey'  : 1,
			'source' : priw,
			'x'      : 0,
			'y'      : 0,
			'xref'   : 'paper',
			'yref'   : 'paper',
			'yanchor': 'bottom'
		}]
	else:
		priw_image = []

	layout = go.Layout(
		height=p_height,
		margin={
			'b': 0,
			'l': 0,
			'r': 0,
			't': 30
		},
		shapes=[
			{
				'line': {
					'color': 'silver',
					'dash' : 'dot',
					'width': 1,
				},
				'type': 'line',
				'x0'  : 0.5,
				'x1'  : 0.5,
				'xref': 'paper',
				'y0'  : 0,
				'y1'  : 1,
				'yref': 'paper'
			}
		],
		images=priw_image,
		title={
			'pad': {
				'b': 10,
				'l': 0,
				'r': 0,
				't': 0
			},
			'text'   : 'Aural',
			'yanchor': 'bottom',
			'y'      : 1,
			'yref'   : 'paper'
		},
		xaxis={
			'fixedrange': True,
			'visible'   : False
		},
		yaxis={
			'fixedrange': True,
			'visible'   : False
		}
	)

	# go.Layout creates a specific type of dict that can't be copied using dict()
	layout_large = go.Layout(layout)
	layout_large['height'] = p_height_large

	return {'layout': layout}, {'layout': layout_large}


@app.callback(
	[Output('camera-graph', 'figure'), Output('camera-graph-large', 'figure')],
	[Input('interval-medium', 'n_intervals'), Input('cam-toggle', 'on')]
)
def update_cameras(_, toggle):
	# Get camera data
	caml = miro_ros_data.sensors_caml
	camr = miro_ros_data.sensors_camr

	pril = miro_ros_data.core_pril
	prir = miro_ros_data.core_prir

	# Needs to be updated manually if plot width changes; value includes margins
	cam_height = 185
	cam_height_large = 380

	# Set camera image properties
	caml_image = {
		'layer'  : 'below',
		'opacity': 1,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,           # Overridden by 'constrain' property but must still be set
		'source' : caml,
		'x'      : 0,
		'xanchor': 'left',
		'xref'   : 'paper',
		'y': 0,
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	camr_image = {
		'layer'  : 'below',
		'opacity': 1,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,
		'source' : camr,
		'x'      : 1,
		'xanchor': 'right',
		'xref'   : 'paper',
		'y': 0,
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	pril_image = {
		'layer'  : 'above',
		'opacity': 0.5,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,
		'source' : pril,
		'x'      : 0,
		'xanchor': 'left',
		'xref'   : 'paper',
		'y': 0,
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	prir_image = {
		'layer'  : 'above',
		'opacity': 0.5,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,
		'source' : prir,
		'x'      : 1,
		'xanchor': 'right',
		'xref'   : 'paper',
		'y': 0,
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	# Show vision with attention overlay, vision alone, or nothing
	if (caml is not None) and (camr is not None):
		if toggle:
			cam_images = [
				caml_image,
				camr_image,
				pril_image,
				prir_image
			]
		else:
			cam_images = [
				caml_image,
				camr_image
			]
	else:
		cam_images = []

	layout = go.Layout(
		height=cam_height,
		images=cam_images,
		margin={
			'b': 10,
			'l': 0,
			'r': 0,
			't': 60
		},
		shapes=[{
			'line': {
				'color': 'black',
				'dash' : 'dot',
				'width': 1,
			},
			'type': 'line',
			'x0'  : 0.5,
			'x1'  : 0.5,
			'xref': 'paper',
			'y0'  : 0,
			'y1'  : 1,
			'yref': 'paper'
		}],
		title={
			'pad'    : {
				'b': 10,
				'l': 0,
				'r': 0,
				't': 0
			},
			'text'   : 'Visual',
			'yanchor': 'bottom',
			'y'      : 1,
			'yref'   : 'paper'
		},
		xaxis={
			'fixedrange': True,
			'visible'   : False
		},
		yaxis={
			'fixedrange': True,
			'visible'   : False
		}
	)

	# go.Layout creates a specific type of dict that can't be copied using dict()
	layout_large = go.Layout(layout)
	layout_large['height'] = cam_height_large

	return {'layout': layout}, {'layout': layout_large}


@app.callback(Output('circadian-graph', 'figure'), [Input('interval-slow', 'n_intervals')])
def update_clock_graph(_):

	if miro_ros_data.core_time.data is not None:
		circ_data = miro_ros_data.core_time.data
	else:
		circ_data = 0

	# TODO: Make circadian clock display leading zeroes
	circ_hrs = range(0, 24)
	# circ_hrs = ['{:02d}'.format(item) for item in range(0, 24)]

	# Set clock hand width and length
	hand_width = 2
	hand_length = 0.9

	# TODO: Find out how to disable polar plot zoom

	data = [
		go.Scatterpolar(
			fill='toself',
			fillcolor='steelblue',
			marker={
				'line': {
					'color': 'black',
					'width': 0.5
				}
			},
			mode='lines',
			r=[0, 0.1, hand_length, 0.1, 0],
			theta=[
				0,
				circ_hrs[circ_data - hand_width],
				circ_hrs[circ_data],
				circ_hrs[circ_data + hand_width],
				0
			]
		)
	]

	layout = go.Layout(
		margin={
			'b': 20,
			'l': 10,
			'r': 10,
			't': 20
		},
		polar={
			'angularaxis': {
				'categoryarray': circ_hrs,
				'direction'    : 'clockwise',
				'nticks'       : 8,
				'period'       : 15,
				'rotation'     : 270,
				'showgrid'     : False,
				'type'         : 'category',
			},
			'radialaxis' : {
				'range'     : [0, 1],
				'visible'   : False
			},
		},
		showlegend=False
	)

	return {
		'data'  : data,
		'layout': layout
	}


if __name__ == '__main__':
	# Initialise a new ROS node
	# 'disable_rostime' must be True to work in Pycharm
	rospy.init_node("dash_listener", anonymous=True, disable_rostime=True)

	# Initialise MiRo client
	miro_ros_data = mi.MiroClient()

	# # This is only to suppress warnings TEMPORARILY
	# app.config['suppress_callback_exceptions'] = True

	# Hot reloading seems to cause "IOError: [Errno 11] Resource temporarily unavailable" errors
	app.run_server(debug=False)
