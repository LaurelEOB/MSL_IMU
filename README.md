# MSL_IMU
Arduino code for external IMU project for CSU MSL

My project was to make an external IMU (Internal Measurement Unit) that would take euler angle measurements for the HAMMR (High-frequency Airborne Microwave and Millimeter-wave Radiometer) instrument.

HAMMR is a radiometer that MSL (The Microwave Systems Laboratory) built to demonstrate wet path delay measurements (Wet path delay is an error caused by humidity in the air between the surface and the radiometer.)  To do this, it was designed to be mounted on the underbelly of a Twin Otter plane pointing down toward the ground during tests. 
After that demonstration mission, MSL was contacted by an Aerospace company called FIRST RF. They contracted MSL to show that a radiometer could detect supercooled liquid water in clouds.
MSL used HAMMR to show this. But to do this, HAMMR would have to be set on its side to be able to observe clouds in front of it. HAMMR’s internal IMU (used for orientation) wasn’t designed for this type of rotation. It was designed to point down at the ground and take scans as the plane flew, not be set on its side to take scans as it is pivoted up or down. When they tested HAMMR with this type of rotation, the Euler angle measurements came back with random spikes and other weird behavior. To be able to use HAMMR in this way, MSL needed an external IMU that would be able to take Euler angle measurements while HAMMR is doing scans on its side.
