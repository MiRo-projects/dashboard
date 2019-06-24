#!/usr/bin/python
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
import miro_ros_interface as mri

##########
# Define line and arrow dimensions
A_HEIGHT = 20
A_WIDTH = A_HEIGHT / 2
A_VERT_OFFSET = -13
H_WIDTH = 9
L_BORDER = 2
L_COLOUR = '#7b8a8b'        # Matches 'dark' colour from Flatly theme
L_VERT_OFFSET = 25
V_WIDTH = H_WIDTH - (L_BORDER * 2)
V_HEIGHT = 30

##########
# Define custom CSS for lines and arrows
css = {
	'arrow_down': {
		'border-left' : str(A_WIDTH) + 'px solid white',
		'border-right': str(A_WIDTH) + 'px solid white',
		'border-top'  : str(A_HEIGHT) + 'px solid ' + L_COLOUR,
		'height'      : 0,
		'margin'      : 'auto',
		'position'    : 'relative',
		'bottom'      : str(A_HEIGHT) + 'px',
		'width'       : 0
	},
	'arrow_left': {
		'border-bottom': str(A_WIDTH) + 'px solid white',
		'border-right' : str(A_HEIGHT) + 'px solid ' + L_COLOUR,
		'border-top'   : str(A_WIDTH) + 'px solid white',
		'float'        : 'left',
		'height'       : 0,
		'margin-top'   : str(A_VERT_OFFSET) + 'px',
		'width'        : 0
	},
	'arrow_right': {
		'border-bottom': str(A_WIDTH) + 'px solid white',
		'border-left'  : str(A_HEIGHT) + 'px solid ' + L_COLOUR,
		'border-top'   : str(A_WIDTH) + 'px solid white',
		'float'        : 'right',
		'height'       : 0,
		'margin-top'   : str(A_VERT_OFFSET) + 'px',
		'width'        : 0
	},
	'arrow_right_clear': {
		'border-bottom': str(A_WIDTH) + 'px solid transparent',
		'border-left'  : str(A_HEIGHT) + 'px solid transparent',
		'border-top'   : str(A_WIDTH) + 'px solid transparent',
		'float'        : 'right',
		'height'       : 0,
		'margin-top'   : str(A_VERT_OFFSET) + 'px',
		'width'        : 0
	},
	'arrow_up': {
		'border-bottom': str(A_HEIGHT) + 'px solid ' + L_COLOUR,
		'border-left'  : str(A_WIDTH) + 'px solid white',
		'border-right' : str(A_WIDTH) + 'px solid white',
		'height'       : 0,
		'margin'       : 'auto',
		'width'        : 0
	},
	'line_horizontal': {
		'background-color': L_COLOUR,
		'border-bottom'   : str(L_BORDER) + 'px white solid',
		'border-top'      : str(L_BORDER) + 'px white solid',
		'float'           : 'right',
		'height'          : str(H_WIDTH) + 'px',
		'width'           : '100%',
		'margin-top'      : str(L_VERT_OFFSET) + 'px',
	},
	'line_horizontal_clear': {
		'border-bottom'   : str(L_BORDER) + 'px white transparent',
		'border-top'      : str(L_BORDER) + 'px white transparent',
		'float'           : 'right',
		'height'          : str(H_WIDTH) + 'px',
		'width'           : '100%',
		'margin-top'      : str(L_VERT_OFFSET) + 'px',
	},
	'line_horizontal_clear_left': {
		'background-color': 'white',
		'border-right'    : str(V_WIDTH) + 'px ' + L_COLOUR + ' solid',
		'height'          : str(H_WIDTH - L_BORDER) + 'px',
		'width'           : '52%',
		'position'        : 'absolute',
		'right'           : '48%',
		'top'             : str(L_VERT_OFFSET) + 'px'
	},
	'line_vertical': {
		'background-color': L_COLOUR,
		'height'          : '100%',
		'width'           : str(V_WIDTH) + 'px',
		'margin'          : 'auto',
		'min-height'      : str(V_HEIGHT) + 'px',
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
		className='m-0 large text-center',
		is_open=False,
	),
	'ball_large': dbc.Alert(
		"⚽",
		id='ball-alert-large',
		color='info',
		className='m-0 large text-center',
		is_open=False,
	),
	'face': dbc.Alert(
		"😀",
		id='face-alert',
		color='success',
		className='m-0 large text-center',
		is_open=False,
	),
	'face_large': dbc.Alert(
		"😀",
		id='face-alert-large',
		color='success',
		className='m-0 large text-center',
		is_open=False,
	),
	'intro': dbc.Alert(
		[
			html.H4('MiRo Dashboard', className='alert-heading'),
			html.P('This visual representation of MiRo\'s \"cognitive architecture\" reveals some of the '
			       'data and processes driving the robot\'s behaviour.'),
			html.P('Note component connections, observe what happens to each plot as you interact with '
			       'MiRo, and click any of the \'＋\' buttons for more information.')
		],
		className='mx-5 shadow-lg',
		color='primary',
	),
	'connections': dbc.Alert(
		'Many up– and downstream connections are omitted for clarity',
		className='mx-5 shadow small',
		color='light',
	),
}

dashboard_graphs = {
	'action': dcc.Graph(
		id='action-graph',
		config={'displayModeBar': False},
		style={
			'height': '150px',
			'width' : '100%',
		}
	),
	'action_large': dcc.Graph(
		id='action-graph-large',
		config={'displayModeBar': False},
		style={
			'height': '300px',
			'width' : '100%',
		}
	),
	'affect': dcc.Graph(
		id='affect-graph',
		# 'Animate' property is incompatible with changing background images
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
	dcc.Interval(
		id='interval-fast',
		# Too short an interval causes issues as not all plots can be updated before the next callback
		interval=0.5 * 1000,    # Every half-second
		n_intervals=0
	),
	dcc.Interval(
		id='interval-medium',
		interval=1 * 1000,      # Every second
		n_intervals=0
	),
	dcc.Interval(
		id='interval-slow',
		interval=60 * 1000,     # Every minute
		n_intervals=0
	)
])

dashboard_tools = {
	# TODO: Add a callback so the status of both toggles is synchronised
	'cam_toggle': daq.BooleanSwitch(
		color='#18BC9C',        # Matches the Flatly theme 'success' colour used in the attention header
		id='cam-toggle',
		label='Visual attention',
		labelPosition='bottom',
		style={'color': 'black'}
	),
	'cam_toggle_large': daq.BooleanSwitch(
		id='cam-toggle-large',
		label='Visual attention',
		labelPosition='bottom',
	),
	'action_button': dbc.Button(
		'＋',
		id='action-modal-open',
		color='light',
		size='sm',
		style={'float': 'right'}
	),
	'affect_button': dbc.Button(
		'＋',
		id='affect-modal-open',
		color='light',
		size='sm',
		style={'float': 'right'}
	),
	'circadian_button': dbc.Button(
		'＋',
		id='circadian-modal-open',
		color='light',
		size='sm',
		style={'float': 'right'}
	),
	'spatial_button': dbc.Button(
		'＋',
		id='spatial-modal-open',
		color='light',
		size='sm',
		style={'float': 'right'}
	),
}

dashboard_tabs = {
	'action_graph' : dbc.Tab(dashboard_graphs['action_large'], label='Live data'),
	'action_info': dbc.Tab(
		[
			dbc.Alert(
				'Action selection is a fundamental process for all animal life. To successfully navigate the world '
				'and complete goals such as finding food or evading predators, animals must choose at every moment '
				'which action to perform from countless possibilities.',
				color='info',
				className='mt-2'
			),
			dbc.CardDeck([
				dbc.Card(
					[
						dbc.CardHeader('Mammals', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'A collection of brain structures called the **basal ganglia** are commonly thought to '
								'be responsible for action selection in mammals. The basal ganglia continually inhibit '
								'motor centres to prevent unwanted movements, and inputs from many brain areas '
								'corresponding to \'requests\' to perform various actions co-operate and compete to '
								'determine which motor regions are disinhibited to allow an action to be selected.'
							),
							dcc.Markdown(
								'The basal ganglia are also responsible for many related functions, such as linking '
								'several actions together into a learned sequence (e.g. pressing the brakes in a car) '
								'and forming habitual responses to specific stimuli (e.g. switching on the lights when '
								'entering a dark room).'
							)
						]),
						dbc.CardFooter(
							html.A(
								dbc.Button('More information', color='success'),
								href='http://www.scholarpedia.org/article/Basal_ganglia',
								target='_blank'
							)
						)
					],
					className='shadow-sm'
				),
				dbc.Card(
					[
						dbc.CardHeader('MiRo', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'Unlike a real animal which can perform any action its body will physically allow, '
								'MiRo can only choose from a set of seven possible actions, the parameters of which '
								'are largely predefined. MiRo is unable to learn new actions, form sequences of '
								'actions, or associate actions with positive or negative outcomes.'
							),
							dcc.Markdown(
								'MiRo cannot set long-term goals or anticipate future situations and thus selects '
								'actions purely as a reaction to the current situation. The process by which actions '
								'are selected is also greatly simplified compared to living mammals.'
							)
						])
					],
					className='shadow-sm'
				),
			])
		],
		label='Information'
	),
	'affect_graph': dbc.Tab(
		dbc.Table(
			html.Tr([
				html.Td(dashboard_graphs['affect_large']),
				html.Td(dashboard_graphs['sleep_large'])
			]),
			borderless=True
		),
		label='Live data'
	),
	'affect_info' : dbc.Tab(
		[
			dbc.Alert(
				'Emotions arise from complex neurological, biological, and physiological interactions, though the '
				'neural changes and the conscious perception of those changes should be considered as separate '
				'phenomena. Affective responses provide rapid and vital information on how best to deal with '
				'environmental stimuli.',
				color='info',
				className='mt-2'
			),
			dbc.CardDeck([
				dbc.Card(
					[
						dbc.CardHeader('Mammals', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'Mammals have a rich and complex affective response to the world around them, '
								'generated by the interactions of multiple brain regions including the **amygdala** '
								'and regions of **cortex** as well as the endocrine system.'
							),
							dcc.Markdown(
								'Emotional responses guide attention, drive learning, and influence behaviour and are '
								'therefore enormously important to an animal\'s well-being and success. For example, '
								'fear of predators preserves safety by inducing fleeing, and disgust preserves health '
								'by inducing the avoidance of poor food.'
							)
						]),
						dbc.CardFooter(
							html.A(
								dbc.Button('More information', color='success'),
								href='http://www.scholarpedia.org/article/Emotion',
								target='_blank'
							)
						)
					],
					className='shadow-sm'
				),
				dbc.Card(
					[
						dbc.CardHeader('MiRo', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'MiRo\'s entire emotional state is represented by two numbers representing valence '
								'(i.e. good/bad) and arousal (or intensity). These values are influenced by internal '
								'and external events (MiRo will be \'happy\' if you pet him and \'sad\' if you shake '
								'him), but are not associated with specific people, events, or places and will not '
								'influence MiRo\'s long-term behaviour.'
							),
							dcc.Markdown(
								'MiRo\'s emotions are driven entirely by his current situation rather than memories of '
								'previous encounters or expectations of the future, and there is no scope for '
								'representing emotions that do not fit on a strictly good/bad continuum such as anger, '
								'confusion, curiosity, or disgust.'
							)
						])
					],
					className='shadow-sm'
				),
			])
		],
		label='Information'
	),
	'circadian_info': dbc.Tab(
		[
			# TODO: Circadian information
			dbc.Alert(
				'Circadian rhythms regulate many important biological processes such as hunger and sleep, and are '
				'maintained through exposure to periodically varying stimuli such as light or heat',
				color='info',
				className='mt-2'
			),
			dbc.CardDeck([
				dbc.Card(
					[
						dbc.CardHeader('Mammals', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'Circadian rhythms are the result of activity in the **hypothalamus**, which drives '
								'hormonal changes that induce feelings of sleepiness or wakefulness. These periodic '
								'changes in behaviour help ensure animals are active or asleep at appropriate times '
							)
						]),
						dbc.CardFooter(
							html.A(
								dbc.Button('More information', color='success'),
								href='http://www.scholarpedia.org/article/Models_of_hypothalamus#Circadian_Rhythm_Generation',
								target='_blank'
							)
						)
					],
					className='shadow-sm'
				),
				dbc.Card(
					[
						dbc.CardHeader('MiRo', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'MiRo exhibits a simple circadian rhythm driven by light as well as his internal '
								'clock, but rather than causing motivational changes and sleep–seeking behaviour, MiRo '
								'is forcibly sent to sleep at a certain point of the cycle. MiRo\'s circadian '
								'rhythm is greatly accelerated compared to mammals, and MiRo has no hormonal '
								'system or behavioural patterns to regulate.'
							)
						])
					],
					className='shadow-sm'
				),
			])
		],
		label='Information'
	),
	'spatial_graph': dbc.Tab(
		[
			dashboard_graphs['aural_large'],
			dashboard_graphs['cameras_large'],
			dashboard_tools['cam_toggle_large'],
			dashboard_alerts['ball_large'],
			dashboard_alerts['face_large'],
		],
		label='Live data'
	),
	'spatial_info'  : dbc.Tab(
		[
			dbc.Alert(
				'Animals have constant access to an enormous amount of sensory data, much of which will not be '
				'important or relevant to its current goal. Attention is the process by which a subset of this data is '
				'selected as useful and subjected to enhanced processing and integration.',
				color='info',
				className='mt-2'
			),
			dbc.CardDeck([
				dbc.Card(
					[
						dbc.CardHeader('Mammals', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'Attention in mammals is a complex process involving the **thalamus** and the '
								'**superior colliculus**, as well as some regions of **cortex**. It is often '
								'considered to comprise the aspects of orienting, filtering, and searching, and can be '
								'focused on internal or external events, as well as being split between several '
								'different sources (albeit poorly).'
							),
							dcc.Markdown(
								'Attention is closely related to action selection as well as consciousness. Often, the '
								'focus of attention (e.g. food) and the associated action (e.g. eating) are so closely '
								'linked that it is unclear whether it is the action or the attentional locus being '
								'selected.'
							)
						]),
						dbc.CardFooter(
							html.A(
								dbc.Button('More information', color='success'),
								href='http://www.scholarpedia.org/article/Attention',
								target='_blank'
							)
						)
					],
					className='shadow-sm'
				),
				dbc.Card(
					[
						dbc.CardHeader('MiRo', className='font-weight-bold'),
						dbc.CardBody([
							dcc.Markdown(
								'MiRo\'s attentional modal is highly simplified, consisting of a \'salience map\' that '
								'determines how much interest to show any point of his visual field. The factors '
								'controlling the map\'s values are predetermined; MiRo cannot learn or decide to pay '
								'more or less attention to different features. MiRo also cannot direct his attention '
								'inwards, or indeed exert any control over his attention at all.'
							),
							dcc.Markdown(
								'MiRo\'s attentional system includes only the \'orienting\' aspect of attention. MiRo '
								'can approach items of interest, but cannot focus on these items by filtering out '
								'distractions or actively searching for interesting things.'
							)
						])
					],
					className='shadow-sm'
				),
			])
		],
		label='Information'
	)
}

dashboard_modals = html.Div([
	dbc.Modal(
		[
			dbc.ModalHeader('Action selection'),
			dbc.ModalBody(
				dbc.Tabs([
					dashboard_tabs['action_graph'],
					dashboard_tabs['action_info']
				])
			),
			dbc.ModalFooter(
				dbc.Button(
					'Close',
					id='action-modal-close',
					color='danger',
					className='ml-auto'
				)
			),
		],
		id='action-modal',
		centered=True,
		size='xl'
	),
	dbc.Modal(
		[
			dbc.ModalHeader('Affect'),
			dbc.ModalBody(
				dbc.Tabs([
					dashboard_tabs['affect_graph'],
					dashboard_tabs['affect_info']
				])
			),
			dbc.ModalFooter(
				dbc.Button(
					'Close',
					id='affect-modal-close',
					color='danger',
					className='ml-auto'
				)
			),
		],
		id='affect-modal',
		centered=True,
		size='xl'
	),
	dbc.Modal(
		[
			dbc.ModalHeader('Circadian rhythm'),
			dbc.ModalBody(dashboard_tabs['circadian_info']),
			dbc.ModalFooter(
				dbc.Button(
					'Close',
					id='circadian-modal-close',
					color='danger',
					className='ml-auto'
				)
			),
		],
		id='circadian-modal',
		centered=True,
		size='xl'
	),
	dbc.Modal(
		[
			dbc.ModalHeader('Spatial attention'),
			dbc.ModalBody(
				dbc.Tabs([
					dashboard_tabs['spatial_graph'],
					dashboard_tabs['spatial_info']
				])
			),
			dbc.ModalFooter(
				dbc.Button(
					'Close',
					id='spatial-modal-close',
					color='danger',
					className='ml-auto'
				)
			),
		],
		id='spatial-modal',
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
		'Perception',
		target='tooltip-environment-filter',
	),
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
		'Touch, sound, light, tilt',
		target='tooltip-bottom-affect',
	),

	# Row 6
	dbc.Tooltip(
		'Current pose',
		target='tooltip-sensory-motor',
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
	# TOP
	'Row_top': dbc.Row(
		dbc.Col(
			dbc.Alert(
				'⬆ To higher functions ⬆',
				color='dark',
				className='my-0 py-0 text-center'
			)
		),
		no_gutters=True
	),

	# Row 1
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

	# Row 2
	'Row_2': dbc.Row(
		[
			dbc.Col(
				dashboard_alerts['intro'],
				width={
					'size'  : 3,
					'offset': 0
				},
			),
			dbc.Col(
				[
					dbc.Card(
						dbc.CardBody('Sum of current motor output and selected action'),
						color='light',
					),
					html.Div(style=css['line_vertical']),
				],
				width={
					'size'  : 1,
					'offset': 0
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
							[
								'Action selection',
								dashboard_tools['action_button'],
							],
							className='bg-warning font-weight-bold lead'
						),
						dbc.CardBody(dashboard_graphs['action'])
					],
					color='warning',
					inverse=True,
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

	# Row 3
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

	# Row 4
	'Row_4': dbc.Row(
		[
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader('Environment'),
						dbc.CardImg(
							src='/assets/icon_park.png',
							bottom=True
						)
					],
					color='light',
					className='ml-1',
				),
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
					html.Div(
						style=css['arrow_right'],
						id='tooltip-environment-filter'
					)
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
							dbc.CardBody('Motor reafferent noise filter'),
							dbc.CardFooter(
								'➡ Self-activity reports',
								style={'font-size': 'x-small'}
							),
						],
						color='light'
					)
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
						id='tooltip-top-spatial'
					),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
					html.Div(style=css['line_vertical']),
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
							],
							className='bg-success font-weight-bold lead'
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
					color='success',
					inverse=True,
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
						dbc.CardHeader(
							[
								'Affect',
								dashboard_tools['affect_button']
							],
							className='bg-info font-weight-bold lead'
						),
						dbc.CardBody(dashboard_graphs['affect'])
					],
					color='info',
					className='mx-1',
					inverse=True,
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

	# Row 5
	'Row_5': dbc.Row(
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

	# Row 6
	'Row_6': dbc.Row(
		[
			dbc.Col(
				html.Div(
					[
						html.Div(style=css['line_horizontal_clear_left']),
						html.Div(style=css['line_horizontal']),
						html.Div(style=css['line_vertical']),
					]
				),
				width={
					'size'  : 1,
					'offset': 3
				},
			),
			dbc.Col(
				[
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),
			dbc.Col(
				[
					dbc.Card(
						[
							dbc.CardBody('Motor body model'),
							dbc.CardFooter(
								'➡ Self-activity reports',
								style={'font-size': 'x-small'}
							)
						],
						color='light'
					),
					html.Div(style=css['line_vertical'])
				],
				width={
					'size'  : 1,
					'offset': 0
				},
				style={'overflow': 'hidden'}
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
				[
					dbc.Card(
						[dbc.CardBody('Sensory body model')],
						color='light'
					),
					html.Div(
						style=css['arrow_up'],
						id='tooltip-bottom-sensory'
					),
					html.Div(style=css['line_vertical'])
				],
				width={
					'size'  : 1,
					'offset': 0
				},
				# Necessary to keep stretched arrows hidden
				style={'overflow': 'hidden'}
			),
			dbc.Col(
				dbc.Card(
					[
						dbc.CardHeader(
							[
								'Circadian rhythm',
								dashboard_tools['circadian_button']
							],
							className='bg-primary font-weight-bold lead'
						),
						dbc.CardBody(dashboard_graphs['circadian'])
					],
					color='primary',
					className='mx-1',
					inverse=True,
					outline=True,
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
				[
					dbc.Card(
						[
							dbc.CardHeader('Expression'),
							dbc.CardBody(
								dbc.ListGroup(
									[
										dbc.ListGroupItem([
											'Ears',
											dbc.CardImg(
												src='assets/express_ear.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Eyelids',
											dbc.CardImg(
												src='assets/express_eye.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Lights',
											dbc.CardImg(
												src='assets/express_lights.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Tail',
											dbc.CardImg(
												src='assets/express_dog.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Vocalisation',
											dbc.CardImg(
												src='assets/express_speaker.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
									],
									className='small',
									flush=True,
								),
								className='border-0 m-0 p-0'
							),
							dbc.CardFooter(
								'➡ Self-activity reports',
								style={'font-size': 'x-small'}
							)
						],
						color='light',
						className='mx-1',
					),
					html.Div(style=css['line_vertical'])
				],
				width={
					'size'  : 1,
					'offset': 0
				},
				style={'overflow': 'hidden'}
			),
		],
		no_gutters=True
	),

	# Row 7
	'Row_7': dbc.Row(
		[
			dbc.Col(
				dashboard_alerts['connections'],
				width={
					'size'  : 3,
					'offset': 0
				},
			),
			dbc.Col(
				[
					html.Div(style=css['line_vertical']),
					html.Div(style=css['arrow_down']),
				],
				id='tooltip-motor-bottom',
				width={
					'size'  : 1,
					'offset': 2
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

	# BOTTOM
	'Row_btm': dbc.Row(
		dbc.Col(
			dbc.Alert(
				'⬇ To lower functions ⬇',
				color='dark',
				className='my-0 py-0 text-center'
			)
		),
		no_gutters=True
	)
}

##########
# Define dashboard layout
# See other included themes: https://bootswatch.com
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.FLATLY])

app.title = 'MiRo Dashboard'

# CSS modification needed to remove corner 'undo' button
app.css.append_css({'external_url': 'assets/stylesheet.css'})

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
	[
		Output('ball-alert', 'is_open'),
		Output('ball-alert-large', 'is_open'),
		Output('face-alert', 'is_open'),
		Output('face-alert-large', 'is_open'),
		Output('action-graph', 'figure'),
		Output('action-graph-large', 'figure'),
		Output('affect-graph', 'figure'),
		Output('affect-graph-large', 'figure'),
		Output('sleep-graph-large', 'figure')
	],
	[Input('interval-fast', 'n_intervals')]
)
def callback_fast(_):
	# Initialise output data dictionary
	output = {}

	# Ball alert
	if (miro_ros_data.core_detect_ball_l is not None) and (miro_ros_data.core_detect_ball_r is not None):
		if (len(miro_ros_data.core_detect_ball_l.data) > 1) or (len(miro_ros_data.core_detect_ball_r.data) > 1):
			output['ball-alert'] = True
			output['ball-alert-large'] = True
		else:
			output['ball-alert'] = False
			output['ball-alert-large'] = False

	# Face alert
	if (miro_ros_data.core_detect_face_l is not None) and (miro_ros_data.core_detect_face_r is not None):
		if (len(miro_ros_data.core_detect_face_l.data) > 1) or (len(miro_ros_data.core_detect_face_r.data) > 1):
			output['face-alert'] = True
			output['face-alert-large'] = True
		else:
			output['face-alert'] = False
			output['face-alert-large'] = False

	# Action graph
	if (miro_ros_data.selection_priority is not None) and (miro_ros_data.selection_inhibition is not None):
		action_inhibition = np.array(miro_ros_data.selection_inhibition.data)
		# Priority is made negative so it appears to the left of the bar chart
		action_priority = np.array([-x for x in miro_ros_data.selection_priority.data])
	else:
		action_inhibition = [0]
		action_priority = [0]

	# TODO: Extract this list automatically
	action_list = [
		'Mull',
		'Halt',
		'Orient',
		'Approach',
		'Avert',
		'Flee',
		'Retreat'
	]

	action_layout = go.Layout(
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
		yaxis={'fixedrange': True}
	)

	action_data = [
		go.Bar(
			hoverinfo='text+y',
			# Format input label to three decimal places
			hovertext=np.round(-action_priority, decimals=3),
			marker={'color': '#F39C12'},    # Match header colour
			name='Input',
			orientation='h',
			x=action_priority,
			y=action_list,
		),
		go.Bar(
			hoverinfo='none',
			marker={'color': '#95a5a6'},    # Match Flatly theme grey
			name='Output',
			orientation='h',
			x=action_inhibition,
			y=action_list,
		)
	]

	output['action-graph'] = {
		'data'  : action_data,
		'layout': action_layout
	}
	output['action-graph-large'] = {
		'data'  : action_data,
		'layout': action_layout
	}

	# Affect graphs
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
	affect_layout_null = go.Layout(
		margin={
			'b': 30,
			'l': 20,
			'r': 5,
			't': 0
		},
		xaxis=affect_xaxis,
		yaxis=affect_yaxis
	)

	# Update affect
	affect_input = miro_ros_data.core_affect
	if affect_input is not None:
		affect_data = {
			'emotion': go.Scatter(
				# TODO: Make hovertext show both X and Y values together
				marker={
					'color': 'steelblue',
					'size' : 15,
					'line' : {
						'width': 0.5,
						'color': 'black'
					}
				},
				mode='markers',
				name='Emotion',
				opacity=0.7,
				x=np.array(np.round(affect_input.emotion.valence, decimals=3)),
				y=np.array(np.round(affect_input.emotion.arousal, decimals=3)),
			),
			'mood'   : go.Scatter(
				marker={
					'color': 'seagreen',
					'size' : 15,
					'line' : {
						'width': 0.5,
						'color': 'black'
					}
				},
				mode='markers',
				name='Mood',
				opacity=0.7,
				x=np.array(np.round(affect_input.mood.valence, decimals=3)),
				y=np.array(np.round(affect_input.mood.arousal, decimals=3)),
			),
			'sleep'  : go.Scatter(
				marker={
					'color': 'salmon',
					'size' : 15,
					'line' : {
						'width': 0.5,
						'color': 'black'
					}
				},
				mode='markers',
				name='Wakefulness',
				opacity=0.7,
				x=np.array(np.round(affect_input.sleep.wakefulness, decimals=3)),
				y=np.array(np.round(affect_input.sleep.pressure, decimals=3)),
			)
		}

		# Get the appropriate face from the 'faces' dictionary based on current mood values
		for x in np.arange(0, 1, 0.2):
			for y in np.arange(0, 1, 0.3):
				if (x < affect_data['mood'].x <= x + 0.2) and (y < affect_data['mood'].y <= y + 0.3):
					# Round the results to nearest 0.1 to prevent floating point errors; inaccurate but unimportant
					affect_face = affect_faces['{0:.1f}'.format(x)]['{0:.1f}'.format(y)]

		for x in np.arange(0, 1, 0.25):
			if x < affect_data['sleep'].x <= x + 0.25:
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
				'b': 30,
				'l': 20,
				'r': 5,
				't': 0
			},
			showlegend=True,
			xaxis=affect_xaxis,
			yaxis=affect_yaxis
		)

		sleep_layout = go.Layout(affect_layout)
		sleep_layout['xaxis'] = sleep_xaxis
		sleep_layout['yaxis'] = sleep_yaxis
		# TODO: If possible, just modify the 'source' attribute
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
				affect_data['emotion'],
				affect_data['mood'],
				affect_data['sleep']
			],
			'layout': affect_layout
		}

		affect_figure_large = {
			'data'  : [
				affect_data['emotion'],
				affect_data['mood'],
			],
			'layout': affect_layout
		}

		sleep_figure_large = {
			'data'  : [affect_data['sleep']],
			'layout': sleep_layout
		}

		output['affect-graph'] = affect_figure
		output['affect-graph-large'] = affect_figure_large
		output['sleep-graph-large'] = sleep_figure_large

	else:
		output['affect-graph'] = {'layout': affect_layout_null}
		output['affect-graph-large'] = {'layout': affect_layout_null}
		output['sleep-graph-large'] = {'layout': affect_layout_null}

	# Return all outputs
	return \
		output['ball-alert'], \
		output['ball-alert-large'], \
		output['face-alert'], \
		output['face-alert-large'], \
		output['action-graph'], \
		output['action-graph-large'], \
		output['affect-graph'], \
		output['affect-graph-large'], \
		output['sleep-graph-large']


@app.callback(
	[
		Output('aural-graph', 'figure'),
		Output('aural-graph-large', 'figure'),
		Output('camera-graph', 'figure'),
		Output('camera-graph-large', 'figure')
	],
	[
		Input('interval-medium', 'n_intervals'),
		Input('cam-toggle', 'on'),
		Input('cam-toggle-large', 'on')
	]
)
def callback_medium(_, toggle, toggle_large):
	# Initialise output data dictionary
	output = {}

	# Aural graph
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

	aural_layout = go.Layout(
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
	aural_layout_large = go.Layout(aural_layout)
	aural_layout_large['height'] = p_height_large

	output['aural-graph'] = {'layout': aural_layout}
	output['aural-graph-large'] = {'layout': aural_layout_large}

	# Camera graphs
	# Get camera data
	caml = miro_ros_data.sensors_caml
	camr = miro_ros_data.sensors_camr

	pril = miro_ros_data.core_pril
	prir = miro_ros_data.core_prir

	# Needs to be updated manually if plot width changes; value includes margins
	cam_height = 190
	cam_height_large = 380

	# Set camera image properties
	caml_image = {
		'layer'  : 'below',
		'opacity': 1,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,       # Overridden by 'constrain' property but must still be set
		'source' : caml,
		'x'      : 0,
		'xanchor': 'left',
		'xref'   : 'paper',
		'y'      : 0,
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
		'y'      : 0,
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
		'y'      : 0,
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
		'y'      : 0,
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	# Show vision with attention overlay, vision alone, or nothing
	if (caml is not None) and (camr is not None):
		if toggle or toggle_large:
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

	camera_layout = go.Layout(
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
	camera_layout_large = go.Layout(camera_layout)
	camera_layout_large['height'] = cam_height_large

	output['camera-graph'] = {'layout': camera_layout}
	output['camera-graph-large'] = {'layout': camera_layout_large}

	# Return all outputs
	return \
		output['aural-graph'], \
		output['aural-graph-large'], \
		output['camera-graph'], \
		output['camera-graph-large']


@app.callback(
	Output('circadian-graph', 'figure'),
	[Input('interval-slow', 'n_intervals')]
)
def callback_slow(_):
	# Initialise output data dictionary
	output = {}

	# Circadian graph
	if miro_ros_data.core_time.data is not None:
		circ_input = miro_ros_data.core_time.data
	else:
		circ_input = 0

	# TODO: Make circadian clock display leading zeroes
	circ_hrs = range(0, 24)
	# circ_hrs = ['{:02d}'.format(item) for item in range(0, 24)]

	# Set clock hand width and length
	hand_width = 2
	hand_length = 0.9

	# TODO: Disable polar plot zoom
	circ_data = [
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
			name='Time',
			r=[0, 0.1, hand_length, 0.1, 0],
			theta=[
				0,
				circ_hrs[circ_input - hand_width],
				circ_hrs[circ_input],
				circ_hrs[circ_input + hand_width],
				0
			]
		)
	]

	circ_layout = go.Layout(
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

	output['circadian-graph'] = {
		'data'  : circ_data,
		'layout': circ_layout
	}

	# Return all outputs
	return output['circadian-graph']


@app.callback(
	Output('action-modal', 'is_open'),
	[Input('action-modal-open', 'n_clicks'), Input('action-modal-close', 'n_clicks')],
	[State('action-modal', 'is_open')]
)
def modal_action(n1, n2, is_open):
	if n1 or n2:
		return not is_open

	return is_open


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
	Output('circadian-modal', 'is_open'),
	[Input('circadian-modal-open', 'n_clicks'), Input('circadian-modal-close', 'n_clicks')],
	[State('circadian-modal', 'is_open')]
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


if __name__ == '__main__':
	# Initialise a new ROS node
	# 'disable_rostime' must be True to work in Pycharm
	rospy.init_node("dash_listener", anonymous=True, disable_rostime=True)

	# Initialise MiRo client
	miro_ros_data = mri.MiroClient()

	# This is only to suppress warnings TEMPORARILY
	# app.config['suppress_callback_exceptions'] = True

	# Hot reloading seems to cause "IOError: [Errno 11] Resource temporarily unavailable" errors
	app.run_server(debug=False)