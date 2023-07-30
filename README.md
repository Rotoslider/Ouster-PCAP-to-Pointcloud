# Ouster-PCAP-to-Pointcloud
Converts PCAP files generated using a Ouster OS-32 lidar using a stand alone Matlab script

This is based on the work by Jason Bula and his velodyne_tls Matlab script. https://github.com/jason-bula/velodyne_tls
I have modified it to work with Ouster lidar devices and created a stand alone application for it so you do not need Matlab if you use the installer. You will need to download the Matlab 2022a Runtime which is freely avalible at Mathworks: https://www.mathworks.com/products/compiler/matlab-runtime.html
The Stand alone was written to run on a Windows 64 machine and must be installed to C:\TLS_Ouster or it will not work. You can change this diretory in the MATLAB files and recompile if you would like to to be installed somewhere else. 

Detailed instructions on the settings and calibration of the unit are at https://github.com/jason-bula/velodyne_tls

![desktop_app](https://user-images.githubusercontent.com/15005663/234740916-b61a77a4-608c-4cb5-b7fd-e41293aaf593.png)

The software for the scanner can be found here: https://github.com/Rotoslider/TLS_Pie which will need updated to capture Ouster Pcap files. 
The Ouster should be rotated slowly and precisely along the X axis. A rotation time of 4 minutes will produce a very dense pointcloud. The connector should be pointing down. 
