Frames of Reference

This page gives an overview of the different frames used in the PX4 firmware.
Coordinate Systems

Geographic Coordinate System

Represents position on earth with a longitude and latitude value (Geographic_coordinate_system). Additionally the altitude may may be included. The altitude can be expressed as distance from the earth center or as altitude above the mean sea level. All in all, this gives a spherical coordinate system.

NED Coordinate System

The x axis is aligned with the vector to the north pole (tangent to meridians).
The y axis points to the east side (tangent to parallels)
The z axis points to the center of the earth
This is a classical cartesian coordinate system where the 3 axes are orthogonal to each other.
See North east down for details.
Frames

Inertial frame

A fixed frame. For most low range MAV applications an earth-fixed NED frame at the starting point or aligned with the room.
ECEF - Earth-Centered, Earth-Fixed frame

Cartesian coordinate system at the center of the earth. The positive z axis goes through the north pole. The x and y axes are on the equatorial plane.
Body Fixed Frame

Attached to the aircraft.
The x axis points in forward (defined by geometry and not by movement) direction. (= roll axis)
The y axis points to the right (geometrically) (= pitch axis)
The z axis points downwards (geometrically) (= yaw axis)
Body NED Frame

Attached to the aircraft.
The x axis points towards the north pole.
Th y axis points to the east
The z axis points downwards to the center of the earth
Transformation between frames

Read this excellent tech report on transformations to become familiar with quaternion and matrix based representations.
Euler Angles

Usually a conversion between a earth fixed “ground” frame and the body fixed “in-air” frame is described via Euler-Angles. There are multiple conventions of the Euler angles. In aerospace engineering the standard are the Tait–Bryan angles. The rotation order for the Tait-Bryan angles is  (see the figure):
rotation of  around  (yaw)
rotation of  around Y' (pitch)
rotation of  around  (roll)

This gives the following transformation matrix:

 transforms a vector from the inertial frame to the body fixed frame: . Or in other words  describes  in the body frame .
See the German Wikipedia article for details:http://de.wikipedia.org/wiki/Eulersche_Winkel#Luftfahrtnorm_.28DIN_9300.29_.28Yaw-Pitch-Roll.2C_Z.2C_Y.E2.80.99.2C_X.E2.80.99.E2.80.99.29
Boundaries

In order to avoid ambiguities the following limits are used:



Quaternion Representation

Rotation Averages
Matlab Code
