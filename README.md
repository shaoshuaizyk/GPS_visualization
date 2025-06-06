# GPS_visualization

Run `pip install -r requirements.txt` to install the required library.

Connect the GPS via the serial port. Run the visualization python code
`python visualize_gps.py --lat ${YOUR START LATITUDE ON THE MAP} --lon ${YOUR START LONGITUDE ON THE MAP}` to start the visualization. While the program is running, it provides a visualization of the trajectory and the altitude. You can also see the satellite number and moving speed information. When the visualization plot is closed, you need to input the end latitude and longitude position on the map as well.
The nmea log and the final trajectory will be stored.

Other parameters:

`--baud ${BAUD_RATE}` sets the baud rate, default 115200.

`--max_points ${MAX_VISUALIZE_POINTS}` sets the number of the recent GPS points in the visualization animation, default 1200.

`--sim` sets if running the simulation mode.