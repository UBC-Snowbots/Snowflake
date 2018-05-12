# Localisation Testing
Documents how we tested for the EKF for IGVC 2018

# Installation and Setup (Software)

1. Run 'cd ~/Snowflake && git checkout master'
2. Run 'cd ~/Snowflake/setup_scripts && ./setup_udev_rules.sh'
  - If the script fails make sure that ./install_dependencies.sh successfully completed before.
3. Compile everything by running 'catkin_make'.

# Installation and Setup (Hardware)

1. Plug the encoder and IMU into the USB hub.
2. Place your laptop on the tower platform.
3. Plug the USB hub into your laptop.
4. Plug the GPS into your laptop on a separate port from the USB hub.
5. Place the GPS antenna at the peak of the tower.
6. Move the test cart to an area with good GPS signal

# Collecting rosbags for testing

1. Open up 4 terminals (or equivalent).
2. Run 'roscore' on the first terminal.
2. Run 'cd ~/Snowflake && source devel/setup.sh' on the second terminal.
3. Launch the localisation stack by running 'roslaunch localisation_igvc localisation.launch'.
  - This will launch the EKF, GPS, and IMU.
4. Check that GPS has fix by running 'rostopic echo /gps/navsatfix' on the third terminal.
  - If there's an output and the header is constantly changing, then the GPS has a fix.
  - Make sure the GPS is on it's own port in the laptop to ensure the best fix.
5. Create a physical course with cones.
  - Make sure you know the measurements of the course so you can analyse it properly later.
6. Start collecting the rosbag by running 'rosbag record -a' on the fourth terminal.
7. Move the test cart through the physical course.
8. Stop rosbag collection by pressing 'CTRL + C' while the fourth terminal is in focus.
9. Repeat for as many courses as necessary, but make sure you remember what each rosbag file was recording.

# Analysing the rosbags

1. Run 'rostopic echo -b file.bag -p /topic > file.csv'
2. Open the csv file with LibreOffice.
3. Create a scatterplot of the outputted x and y positions from the generated csv file.
4. Compare the scatterplot with the expected output (What the physical course was).
5. Save the csv as an odf file for future reference.



