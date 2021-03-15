# Plotly Dash modules
import dash_bootstrap_components as dbc
import dash_html_components as html

# MiRo dashboard modules
import dashboard_constants as con
from views.css import css
from views.alerts import dashboard_alerts
from views.graphs import dashboard_graphs
from controllers.tools import dashboard_tools

dashboard_rows = {
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

	'Row_1'  : dbc.Row(
		[
			# Column 4
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

			# Column 5
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),

			# Column 6
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

			# Column 10
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

			# Column 12
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

	'Row_2'  : dbc.Row(
		[
			# Column 1-3
			dbc.Col(
				dashboard_alerts['intro'],
				width={
					'size'  : 3,
					'offset': 0
				},
			),

			# Column 4
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

			# Column 5
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

			# Column 6-10
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

			# Column 12
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

	'Row_3'  : dbc.Row(
		[
			# Column 4
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 3
				}
			),

			# Column 5
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),

			# Column 6
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

			# Column 8
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

			# Column 10
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

			# Column 12
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

	'Row_4'  : dbc.Row(
		[
			# Column 1
			dbc.Col(
				[
					html.Div('ENVIRONMENT', style=css['bar_left']),
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right'], ),
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
				],
				# dbc.Card(
				# 	[
				# 		dbc.CardHeader('Environment'),
				# 		dbc.CardImg(
				# 			src=con.ASSET_PATH + 'icon_park.png',
				# 			bottom=True
				# 		)
				# 	],
				# 	color='light',
				# 	className='ml-1',
				# ),
				width={
					'size'  : 1,
					'offset': 0
				}
			),

			# Column 2-3
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
					),
					html.Div(style=css['line_horizontal_clear']),
					dbc.Card(
						[
							dbc.CardHeader(
								[
									'Motivation',
									dashboard_tools['motivation_button']
								],
								className='bg-danger font-weight-bold lead'
							),
							dbc.CardBody(dashboard_graphs['motivation']),
							dbc.CardFooter(
								'➡ Internal drives',
								style={
									'color'    : 'black',
									'font-size': 'x-small'
								}
							),
						],
						color='danger',
						inverse=True,
						outline=True,
					),
				],
				width={
					'size'  : 2,
					'offset': 0
				}
			),

			# Column 4
			dbc.Col(
				[
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
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

			# Column 5
			dbc.Col(
				html.Div(
					[
						html.Div(style=css['line_horizontal_clear_left']),
						html.Div(style=css['line_horizontal']),
						html.Div(style=css['arrow_right_clear']),
						html.Div(style=css['line_horizontal']),
						html.Div(style=css['arrow_right_clear']),
						html.Div(style=css['line_horizontal_clear']),
						html.Div(style=css['arrow_right_clear']),
						html.Div(style=css['line_horizontal_clear']),
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

			# Column 6
			dbc.Col(
				[
					html.Div(style=css['line_horizontal']),
					html.Div(
						style=css['arrow_right'],
						id='tooltip-top-spatial'
					),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal_clear']),
					html.Div(style=css['arrow_right_clear']),
					html.Div(style=css['line_horizontal']),
					html.Div(style=css['arrow_right']),
					html.Div(style=css['line_vertical']),
				],
				width={
					'size'  : 1,
					'offset': 0
				}
			),

			# Column 7-9
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
						# dbc.CardBody(
						# 	[
						# dashboard_graphs['aural'],
						# dashboard_graphs['cameras'],
						dashboard_graphs['cameras'],
						# dashboard_alerts['ball'],
						# dashboard_alerts['face'],
						# 	]
						# ),
						# TODO: Remove camera toggle from small mode to reduce vertical space
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

			# Column 10-12
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

	'Row_5'  : dbc.Row(
		[
			# Column 4
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 3
				}
			),

			# Column 6
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 1
				}
			),

			# Column 8
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

			# Column 10
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

			# Column 11
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

			# Column 12
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

	'Row_6'  : dbc.Row(
		[
			# Column 4
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

			# Column 5
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

			# Column 6
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

			# Column 7
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

			# Column 8
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

			# Column 10
			dbc.Col(
				[
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
						style={'height': '60%'}
					),
					html.Div(
						style=css['arrow_up'],
						id='tooltip-bottom-circadian',
					),
					html.Div(style=css['line_vertical']),
				],
				width={
					'size'  : 1,
					'offset': 1
				},
				# Necessary to keep stretched arrows hidden
				style={'overflow': 'hidden'}
			),

			# Column 11
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),

			# Column 12
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
												src=con.ASSET_PATH + 'express_ear.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Eyelids',
											dbc.CardImg(
												src=con.ASSET_PATH + 'express_eye.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Lights',
											dbc.CardImg(
												src=con.ASSET_PATH + 'express_lights.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Tail',
											dbc.CardImg(
												src=con.ASSET_PATH + 'express_dog.png',
												className='float-right',
												style={'width': '20px'}
											),
										]),
										dbc.ListGroupItem([
											'Vocalisation',
											dbc.CardImg(
												src=con.ASSET_PATH + 'express_speaker.png',
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

	'Row_7'  : dbc.Row(
		[
			# Column 1-3
			dbc.Col(
				dashboard_alerts['connections'],
				width={
					'size'  : 3,
					'offset': 0
				},
			),

			# Column 6
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

			# Column 8
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 1
				}
			),

			# Column 10
			dbc.Col(
				[
					# html.Div(style=css['arrow_up']),
					html.Div(style=css['line_vertical']),
				],
				# id='tooltip-bottom-circadian',
				width={
					'size'  : 1,
					'offset': 1
				}
			),

			# Column 11
			dbc.Col(
				html.Div(style=css['line_vertical']),
				width={
					'size'  : 1,
					'offset': 0
				}
			),

			# Column 12
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
