# Plotly Dash modules
from dash.dependencies import Input, Output

# MiRo dashboard modules
from app import app
import constants as con

# MiRo interface modules
from models.basic_functions import miro_ros_interface as mri

# Other modules
import base64
import cv2
from io import BytesIO

# Initialise MiRo clients
miro_core = mri.MiRoCore()
miro_perception = mri.MiRoPerception()


@app.callback(
	[
		Output('audio-pri-wide', 'src'),
		Output('camera-img-left', 'src'),
		Output('camera-img-right', 'src'),
		Output('camera-pri-left', 'src'),
		Output('camera-pri-right', 'src'),
		Output('camera-img-left-large', 'src'),
		Output('camera-img-right-large', 'src'),
		Output('camera-pri-left-large', 'src'),
		Output('camera-pri-right-large', 'src'),
	],
	[
		Input('interval-medium', 'n_intervals'),
		Input('cam-toggle', 'on'),
		Input('cam-toggle-large', 'on')
	]
)
def callback_medium(_, toggle, toggle_large):
	def process_frame(frame, scale):
		# Create base64 URI from image: https://stackoverflow.com/questions/16065694/is-it-possible-to-create-encoded-base64-url-from-image-object
		# frame_buffer = BytesIO()
		# frame_sml = frame.resize(tuple(dim / scale for dim in frame.size))
		# frame_sml.save(frame_buffer, format='PNG')
		# frame_b64 = base64.b64encode(frame_buffer.getvalue())

		retval, im = cv2.imencode('.png', frame)
		data = base64.b64encode(im)

		return 'data:image/png;base64,{}'.format(data)
		# return 'data:image/png;base64,{}'.format(frame_b64)

	if miro_perception.caml is not None:
		caml = miro_perception.caml
		camr = miro_perception.camr
		pril = miro_core.pril
		prir = miro_core.prir
		priw = miro_core.priw

		caml_image = process_frame(caml, con.CAM_SCALE)
		camr_image = process_frame(camr, con.CAM_SCALE)

		if pril is not None and (toggle or toggle_large):
			pril_image = process_frame(pril, 1)
			prir_image = process_frame(prir, 1)
		else:
			pril_image = None
			prir_image = None

		# TODO: Change to EAF method
		if priw is not None:
			priw_image = process_frame(priw, 1)
		else:
			priw_image = con.ASSET_PATH + 'test_priw.png'

	else:
		# Show test patterns
		caml_image = con.ASSET_PATH + 'test_cam_sml.png'
		camr_image = con.ASSET_PATH + 'test_cam_sml.png'
		pril_image = None
		prir_image = None

	# Return all outputs
	return \
		priw_image, \
		caml_image, \
		camr_image, \
		pril_image, \
		prir_image, \
		caml_image, \
		camr_image, \
		pril_image, \
		prir_image
