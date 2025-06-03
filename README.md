The purpose of these codes is to generate point clouds from a SICK MS connected to a raspberry pi(5)
on a cart. The cart is pushed through and environment (crop plot for ag. applications) while scans
are being collected. 

This is a wrapper for https://github.com/SICKAG/ScanSegmentAPI. Clone this into a directory and 
place this repo in the same directory. The environment is managed by poetry. 

Also connected to the pi is a pico, wired to an optical rotary encoder and intertial measurement unit (IMU).
The rotary encoder allows the cart to obtain a sense of distance traveled, which corresponds to the Z axis 
in the resulting point cloud:

  Z: front to back
  X: right
  Y: up

This unorthodox point cloud system is used because the LIDAR is mounted horizontally. 

Data collection is handled by gui.py. This runs pico data and lidar collection on seperate threads. 
Scans are started and stopped using the s key. q quits the gui. The gui saves IMU data alongside raw 
lidar data to seperate csvs. Scan names are predetermined and placed in a csv given to the gui before
scannning. Additional plots (ADL) are made if the user exceeds the list of premade scan names. 

Parsing the raw lidar data is handled by parser.py. This allows viewing of point cloud data in cartesian format 
rather than the lidar's phi, theta, distance. Further parsing is handled by subparser.py, for 
applications where plots contain multiple crops or subplots. Parser.py splits scans of crop rows into 
individual plots based on the scan file names. This is done by assessing whether points are on the 
left or right of the cart and the z value of the points. 

  As of 6/3/2025 parser.py has trouble handling ADL -- L/R splitting.
  subparser.py does not take ADL as input. 

I have begun to dabble with some post processing point cloud operations in voxelize.py.


