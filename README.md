# Ouster-PCAP-to-Pointcloud
Converts PCAP files generated using a Ouster OS-32 lidar using MATLAB or the included stand alone MATLAB App.
The lidar needs to be rotated around its X axis with the connector pointing down. The closer the x axis is aligned to the motor axis the better your results will be. The rotation needs to be very smooth and consistant. I have found the planetary gearsets not as smooth as what is required. A harmonic gearbox in the 30 to 50 to 1 ratio works well with either a 0.9 or 1.8 degree stepper motor. A rotation time of 2 to 4 minutes produces a dense cloud. 

This is based on the work by Jason Bula and his velodyne_tls Matlab script. https://github.com/jason-bula/velodyne_tls
I have modified it to work with Ouster lidar devices and created a stand alone application for it so you do not need Matlab if you use the installer. You will need to download the Matlab 2022a Runtime which is freely avalible at Mathworks: https://www.mathworks.com/products/compiler/matlab-runtime.html
The Stand alone was written to run on a Windows 64 machine and must be installed to C:\Users\<your_username>\TLS_Ouster or it will not work. You can change this diretory in the MATLAB files and recompile if you would like to to be installed somewhere else.
The settings file will be installed in the application sub folder.
If you would like to run this on linux you can edit the .m and .mplapp files. There are a few lines that will need to be uncommented. And then comment out the lines for Windows. These are related to the file save and reading locations. All the code other than that is universal to Windows Linux or Mac. 
You will need the Lidar Toolbox if running the code in Matlab. Otherwise just install the app and be happy.

Detailed instructions on the settings and calibration of the unit are at https://github.com/jason-bula/velodyne_tls


![TLS_Ouster_screen](https://github.com/Rotoslider/Ouster-PCAP-to-Pointcloud/assets/15005663/306b04ce-8181-4143-b341-cfd641331a80)

The software for the scanner can be found here: https://github.com/Rotoslider/TLS_Pie which will need updated to capture Ouster Pcap files. 
The Ouster should be rotated slowly and precisely along the X axis. A rotation time of 4 minutes will produce a very dense pointcloud. The connector should be pointing down. 

If the angle is below 45 degrees or above 360 it will now throw a warning and adjust it to 360. 
![TLS_Ouster_angle_error window](https://github.com/Rotoslider/Ouster-PCAP-to-Pointcloud/assets/15005663/3829d5aa-9e7a-4d07-b3c5-19d832094ae3)

If the duration is set longer than the scan time in seconds, minus the delay time it will now automatically replace it with the max usable time and issue a warning. 
![TLS_Ouster_time_error_window](https://github.com/Rotoslider/Ouster-PCAP-to-Pointcloud/assets/15005663/d02321b3-80f7-4c96-a9c0-d3d3c6276845)

At the end of proccessing a window will open to display the final point cloud. If its the middle band it will show it. If its the first and last or all bands it will show you the merged cloud. It this window you can zoom pan and rotate the cloud for inspection. This is a subsampled preview of the full cloud saved in the results folder.

![TLS_Ouster_Merged_output viewer](https://github.com/Rotoslider/Ouster-PCAP-to-Pointcloud/assets/15005663/2a787bb8-5de9-48c3-889c-b94b50225d9f)
