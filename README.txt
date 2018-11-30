#############################
README.txt
James Watson, 2018 November
Building, Running, and Usage directions for Project submission
#############################

=== Build Instructions ===

In the root directory for the project:
source build_HW.sh

The script will:
	1. Create a 'build' directory within the present directory, if it does not exist.  Then cd to 'build'
	2. Run: 'cmake ../' 
	3. Run : make -j4 , then cd to the top program directory
	4. Output the program in the top program directory, named "HW${N}" where '${N}' is the correct HW number

Dependencies:
	* CMake version 3.0+
	* C++11
    * SDL2

___ End Build ___


=== Run Instructions ===

In the root directory for the project:
./scanviewer
where '${N}' is the correct HW number

___ End Run ___


=== Remaining Work ===

1. Texture the point cloud meshes with the infrared images gathered from the camera
2. Display the robot in the pose in which each shot was taken ( animate? )
3. Display the 2D infrared image as a billboard so that the user can visualize the robot's point of view

___ End Work ___


=== Usage Instructions ===

~~ Keys ~~

~ This scene has several 3D camera scans pre-loaded ~
[1] Toggle Scan 1
[2] Toggle Scan 2
[3] Toggle Scan 3
[4] Toggle Scan 4

~ Camera Control ~

[Arrow Up/Dn] : Pitch the plot on a horizontal axis parallel to the screen
[Arrow Rt/Lf] : Yaw the plot about lab Z axis
[PgDn/PgUp] : _ Zoom in and out
[0] : _________ Set view angles to default
NOTE: Camera always faces scan center-ish

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

30 hours ????
	* Conversion to SDL2
	* Setting up apparatus
    * Camera problems
    * Data formatting problems
    * Geometry for taking meshes in and
    * Incorporating Delaunay triangulation by P. Bourke
    * Cleaning the resulting mesh

___ End Time ___
