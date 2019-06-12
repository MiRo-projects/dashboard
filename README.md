# MiRo dashboard
Web dashboard displaying internal MiRo-E data and processes

## Installation
The MiRo dashboard assumes you already have the latest [MiRo MDK](http://labs.consequentialrobotics.com/miro-e/software/) installed, either on your computer to run in simulation or on a physical MiRo robot.

You will also need to install `dash`, `dash-daq`, and `dash-bootstrap-components` for the web frontend, and `opencv-python-headless` and `Pillow` for image processing. It's assumed you already have MDK prerequisites such as `rospy` installed.

Clone the dashboard folder into the `mdk/share/python/miro2/` folder and run app.py to start the dashboard. The dashboard will be available at [localhost:8050](http://localhost:8050).

## Links
* [Dash](https://dash.plot.ly)
* [Dash Bootstrap components](https://dash-bootstrap-components.opensource.faculty.ai)
* [Plotly graphing library](https://plot.ly/python/)
* [Pillow](https://pillow.readthedocs.io/)

### Acknowledgements
Dashboard design and code is based on the original [MiRo GUI](https://github.com/hamidehkerdegari/graphical_interface) by Hamideh Kerdegari.
