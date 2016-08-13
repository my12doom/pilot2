#Frames of Reference
This page gives an overview of the different frames used in the pilot firmware.
#Coordinate Systems

##Geographic Coordinate System
Represents position on earth with a longitude and latitude value (Geographic_coordinate_system). Additionally the altitude may may be included. The altitude can be expressed as distance from the earth center or as altitude above the mean sea level. All in all, this gives a spherical coordinate system.
![coordinate](https://pixhawk.org/lib/exe/fetch.php?tok=0b3ee3&media=http%3A%2F%2Fupload.wikimedia.org%2Fwikipedia%2Fcommons%2Fthumb%2F6%2F62%2FLatitude_and_Longitude_of_the_Earth.svg%2F652px-Latitude_and_Longitude_of_the_Earth.svg.png)
##NED Coordinate System

The x axis is aligned with the vector to the north pole (tangent to meridians)  

The y axis points to the east side (tangent to parallels)  

The z axis points to the center of the earth  

This is a classical cartesian coordinate system where the 3 axes are orthogonal to each other.  

See North east down for details.
#Frames

##Inertial frame
A fixed frame. For most low range MAV applications an earth-fixed NED frame at the starting point or aligned with the room.
ECEF - Earth-Centered, Earth-Fixed frame

Cartesian coordinate system at the center of the earth. The positive z axis goes through the north pole. The x and y axes are on the equatorial plane.

##Body Fixed Frame
Attached to the aircraft.  

The x axis points in forward (defined by geometry and not by movement) direction. (= roll axis)  

The y axis points to the right (geometrically) (= pitch axis)  

The z axis points downwards (geometrically) (= yaw axis)  

##body NED Frame
Attached to the aircraft.  

The x axis points towards the north pole.  

The y axis points to the east  

The z axis points downwards to the center of the earth  

##Transformation between frames
Read this excellent tech report on transformations to become familiar with quaternion and matrix based representations.  

#Euler Angles  

Usually a conversion between a earth fixed “ground” frame and the body fixed “in-air” frame is described via Euler-Angles.  
There are multiple conventions of the Euler angles. In aerospace engineering the standard are the Tait–Bryan angles.   
The rotation order for the Tait-Bryan angles is   ![zyx](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/imge68c69ae0c88ba06ed9c94b02dde7a15.png)  (see the figure):  

rotation of ![x](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/img535b15667b86f1b118010d4c218fecb9.png)around  X (yaw)  

rotation of ![y](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/imgb35e24d8a08c0ab01195f2ad2a78fab7.png) around Y (pitch)  

rotation of ![z](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/img5e16cba094787c1a10e568c61c63a5fe.png) around Z (roll)  

![rotation](https://pixhawk.org/lib/exe/fetch.php?tok=de1983&media=http%3A%2F%2Fmrechte.free.fr%2Fpx4%2FLagewinkel-Drehung2.png)  

This gives the following transformation matrix:

![rotation matrix](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/img370b7b538e045463b478370f80ec238e.png)  
transforms a vector from the inertial frame to the body fixed frame: . Or in other words  describes  in the body frame .  
 
See the German Wikipedia article for details:[WIKI URL][1]  

Boundaries
--------------------------

In order to avoid ambiguities the following limits are used:  

![theta](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/imgca216094d11d752ea37d852fd0bd3bb5.png)  

![fin](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/img8788e69d4828770129160ebf916c32c8.png)  

![fine](https://pixhawk.org/lib/exe/fetch.php?media=wiki:latex:/img1ec8194a0a6f561e9c3ff500cc37596d.png)  



Quaternion Representation
---------------------------
[Rotation Averages][2]  

[RMatlab Code][3]
[1]:http://de.wikipedia.org/wiki/Eulersche_Winkel#Luftfahrtnorm_.28DIN_9300.29_.28Yaw-Pitch-Roll.2C_Z.2C_Y.E2.80.99.2C_X.E2.80.99.E2.80.99.29 "Google"
[2]:http://www.soest.hawaii.edu/wessel/courses/gg711/pdf/Gramkow_2001_JMIV.pdf
[3]:http://www.mathworks.com/matlabcentral/fileexchange/40098-tolgabirdal-averaging-quaternions
