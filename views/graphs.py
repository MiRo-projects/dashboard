# Plotly Dash modules
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html

# MiRo dashboard modules
import dashboard_constants as con

dashboard_graphs = {
	'action'          : dcc.Graph(
		id='action-graph',
		config={'displayModeBar': False},
		style={
			'height': '150px',
			'width' : '100%',
		}
	),
	'action_large'    : dcc.Graph(
		id='action-graph-large',
		config={'displayModeBar': False},
		style={
			'height': '300px',
			'width' : '100%',
		}
	),
	'affect'          : dcc.Graph(
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
	'affect_large'    : dcc.Graph(
		id='affect-graph-large',
		# 'Animate' property is incompatible with changing background images
		# animate=True,
		config={'displayModeBar': False},
		style={
			'height': '500px',
			'width' : '500px',
		}
	),
	'aural'           : dcc.Graph(
		id='aural-graph',
		config={'displayModeBar': False},
		style={'width': '100%'}
	),
	'aural_large'     : dcc.Graph(
		id='aural-graph-large',
		config={'displayModeBar': False},
		style={'width': '100%'}
	),
	# 'cameras': dcc.Graph(
	# 	id='camera-graph',
	# 	config={'displayModeBar': False},
	# 	style={'width': '100%'}
	# ),
	# 'cameras_large': dcc.Graph(
	# 	id='camera-graph-large',
	# 	config={'displayModeBar': False},
	# 	style={'width': '100%'}
	# ),
	# TODO: Add aural graph here as well
	'cameras_large'   : dbc.CardBody(
		[
			# html.H6(
			# 	'Visual',
			# 	className='card-subtitle',
			# 	style={
			# 		'color'     : 'black',
			# 		'text-align': 'center'
			# 	}
			# ),
			html.Div(
				[
					html.Img(
						id='camera-img-left-large',
						style={
							'height': con.CAM_HEIGHT_LARGE,
							'width' : con.CAM_WIDTH_LARGE
						}
					),
					html.Img(
						id='camera-img-right-large',
						style={
							'height': con.CAM_HEIGHT_LARGE,
							'width' : con.CAM_WIDTH_LARGE
						}
					),
					html.Div(
						[
							html.Img(
								id='camera-pri-left-large',
								style={
									'height' : con.CAM_HEIGHT_LARGE,
									'width'  : con.CAM_WIDTH_LARGE,
									'opacity': con.PRI_OPACITY,
								}
							),
							html.Img(
								id='camera-pri-right-large',
								style={
									'height' : con.CAM_HEIGHT_LARGE,
									'width'  : con.CAM_WIDTH_LARGE,
									'opacity': con.PRI_OPACITY,
								}
							),
						],
						style={
							'float'   : 'left',
							'position': 'absolute',
							'left'    : '0px',
							'top'     : '0px',
							'z-index' : '2'
						}
					)
				],
				# Necessary for attention image to overlay vision
				style={'position': 'relative'}
			),
		],
	),
	'cameras'         : dbc.CardBody(
		[
			html.H6(
				'Aural',
				className='card-subtitle',
				style={
					'color'     : 'black',
					'text-align': 'center'
				}
			),
			html.Img(
				id='audio-pri-wide',
				style={
					'height': con.PRIW_HEIGHT,
					'width' : con.PRIW_WIDTH,
				}
			),
			html.P(''),
			html.H6(
				'Visual',
				className='card-subtitle',
				style={
					'color'     : 'black',
					'text-align': 'center'
				}
			),
			html.Div(
				[
					html.Img(
						id='camera-img-left',
						style={
							'height': con.CAM_HEIGHT,
							'width' : con.CAM_WIDTH
						}
					),
					html.Img(
						id='camera-img-right',
						style={
							'height': con.CAM_HEIGHT,
							'width' : con.CAM_WIDTH
						}
					),
					html.Div(
						[
							html.Img(
								id='camera-pri-left',
								style={
									'height' : con.CAM_HEIGHT,
									'width'  : con.CAM_WIDTH,
									'opacity': con.PRI_OPACITY,
								}
							),
							html.Img(
								id='camera-pri-right',
								style={
									'height' : con.CAM_HEIGHT,
									'width'  : con.CAM_WIDTH,
									'opacity': con.PRI_OPACITY,
								}
							),
						],
						style={
							'float'   : 'left',
							'position': 'absolute',
							'left'    : '0px',
							'top'     : '0px',
							'z-index' : '2'
						}
					)
				],
				# Necessary for attention image to overlay vision
				style={'position': 'relative'}
			),
		],
	),

	'circadian'       : dcc.Graph(
		id='circadian-graph',
		# 'Animate' property is incompatible with changing background images
		# animate=True,
		config={'displayModeBar': False},
		style={
			'height': '100%',
			'width' : '100%',
		}
	),
	'motivation'      : dcc.Graph(
		id='motivation-graph',
		config={'displayModeBar': False},
		style={
			'height': '250px',
			'width' : '100%'
		}
	),
	'motivation_large': dcc.Graph(
		id='motivation-graph-large',
		# 'Animate' property is incompatible with changing images
		# animate=True,
		config={'displayModeBar': False},
		style={
			'height': '500px',
			'width' : '100%',
		}
	),
	'sleep_large'     : dcc.Graph(
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
