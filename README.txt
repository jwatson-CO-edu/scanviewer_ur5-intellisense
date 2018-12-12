#############################
README.txt
James Watson, 2018 December
Building, Running, and Usage directions for Project submission
#############################

=== Build Instructions ===

In the root directory for the project:
source build_HW.sh

Dependencies:
	* CMake version 3.0+
	* C++11
    * SDL2

___ End Build ___


=== Run Instructions ===

In the root directory for the project:
./final
where '${N}' is the correct HW number

___ End Run ___


=== Motivation ===

The purpose of this project is to display real 3D camera data as textured meshes.  The depth image from a 3D camera provides 
a wealth of information, but is fuzzy.  The goal is use the filtered point cloud and the original flat image data to 
reconstruct a representation of 3D objects with small, manageable point clouds that still retains rich and informative visual information. 

* Demonstrates *
1. Correct application of textures to an arbitrary mesh by fusing 2D and 3D data
2. Selection of 3D objects (lab frame) from 2D mouse interaction (window frame)
3. Smooth shading of an arbitrary surface
4. Nested reference frames for a character model

___ End Motivate ___


=== Usage Instructions ===

~~ Keys ~~

~ Mesh Selection & Control ~

* This scene has several filtered 3D camera scans pre-loaded *
NOTE: When a scan is toggled ON, the robot will automatically move to the pose the real robot was in when the scan data was captured
[1] Toggle Scan 1
[2] Toggle Scan 2
[3] Toggle Scan 3
[4] Toggle Scan 4
[h] Toggle smooth shading for meshes

~ Camera Control ~

[Keypad 0] : __ Wide shot for robot motion
[Keypad 1] : __ Tight shot for mesh characteristics and selection
[Arrow Up/Dn] : Pitch the plot on a horizontal axis parallel to the screen
[Arrow Rt/Lf] : Yaw the plot about lab Z axis
[PgDn/PgUp] : _ Zoom in and out
[0] : _________ Set view angles to default
NOTE: Camera always faces scan center-ish

~ Robot Control ~

[Shift] && [1-6] : Select robot joint to edit , Joints are numbered 1 to 6 - proximal to distal
[+/-] : _________  Increment / Decrement the angle of the joint selected above

~ Light Position ~

[m] : ______ Toggles auto light movement
 [   ]  : __ Lower/rise light (orbit elevation)
 '   \  : __ Move negative / positive theta (orbit angle)
[Home/End] : Move the light source farther from / closer to the center of the scene

~ Light Properties ~

[a/A] : Decrease/increase ambient light from the orbiting light
[d/D] : Decrease/increase diffuse light from the orbiting light
[s/S] : Decrease/increase specular light from the orbiting light
[n/N] : Decrease/increase shininess of all objects except the orbiting light


~ Other ~

[Esc] : _______ Close program 

___ End Usage ___


=== Dev Time ===

???? 80 hours ???? 
	* Conversion to SDL2
	* Setting up apparatus
    * Camera problems
    * Data formatting problems
    * Geometry for taking meshes in and
    * Incorporating Delaunay triangulation by P. Bourke
    * Cleaning the resulting mesh
    ----------
    * Converting mouse space to lab space
    * Smooth lighting for captured mesh
    * Troubleshooting a lighting problem that turned everything blue (Including text!)
    * SDL Cursor
    * Decouple from VSYNC
    * Geometry for ray-mesh collision
    * AABB Collision
    * Winding number + Mesh Collision
    * Display of selected mesh AABB
    * Display mesh data

___ End Time ___
