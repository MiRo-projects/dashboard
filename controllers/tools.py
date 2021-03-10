# Plotly Dash modules
import dash_bootstrap_components as dbc
import dash_daq as daq


dashboard_tools = {
	# TODO: Add a callback so the status of both toggles is synchronised
	'cam_toggle': daq.BooleanSwitch(
		# Matches the Flatly theme 'success' colour used in the attention header
		color='#18BC9C',
		id='cam-toggle',
		label='Visual attention',
		labelPosition='bottom',
		style={'color': 'black'}
	),
	'cam_toggle_large': daq.BooleanSwitch(
		color='#18BC9C',
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
	'motivation_button': dbc.Button(
		'＋',
		id='motivation-modal-open',
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
