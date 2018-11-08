#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
HEADER_TEMPLATE.h
James Watson , YYYY MONTHNAME
A ONE-LINE DESRIPTION OF THE FILE

Template Version: 2018-07-16
***********/

#ifndef DH_ROBOT_H // This pattern is to prevent symbols to be loaded multiple times
#define DH_ROBOT_H // from multiple imports

// ~~ Includes ~~
// ~ LIBNAME_i ~
// ~ Local ~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/
#include "MathGeo.h"
#include "OGL_utils.h"

// ~~ Shortcuts and Aliases ~~

// ~~ Constants ~~
float BASEHEIGHT = 0.017;
vec3e URGREY{ 117/255.0 , 125/255.0 , 130/255.0 };
vec3e URBLCK{  50/255.0 ,  50/255.0 ,  50/255.0 };
vec3e URMETL{ 224/255.0 , 224/255.0 , 224/255.0 };


// === Classes and Structs =================================================================================================================

// == struct DH_Parameters ==
struct DH_Parameters{
	std::vector<float> alpha; 
	std::vector<float> a; 
	std::vector<float> d; 
	std::vector<float> theta; 
};

// __ End DH __

// == class RobotLink ==

class RobotLink{
public:
		
	// ~~ Con/Destructors ~~
	RobotLink( float pTheta , const vec3e& pOrigin , 
			   float pD_dist , float pA_dist , const vec3e& pNextRotnAxis , float pNextRotnAngl , 
			   void (*pDrawFunc)(const DH_Parameters& DH) );

	// ~~ Configuration ~~
	void add_distal( RobotLink* link );
	bool is_leaf();

	// ~~ Robot Motion ~~
	void set_theta( float pTheta );
	
	// ~~ Drawing ~~
	void draw();

protected:
	// ~ Proximal Joint ~
	float /* ----------- */ theta;
	// ~ Relative Location ~
	vec3e /* ----------- */ origin;
	// ~ Distal Joint ~
	float /* ----------- */ d_dist;
	float /* ----------- */ a_dist;
	vec3e /* ----------- */ nextRotnAxis;
	float /* ----------- */ nextRotnAngl;
	// ~ Distal Links ~
	std::vector<RobotLink*> distalLinks;
	// ~ Rendering ~
	void /* ------------ */ (*drawFunc)();
	// ~ Bookkeeping ~
	uint /* ------------ */ index;
};

// __ End RobotLink __


// == class UR5_OGL ==

class UR5_OGL{
public:

	UR5_OGL( const vec3e& baseOrigin );

	// ~~ Members ~~

	// ~ Joint State ~
	std::vector<float> q    = {   0.0 ,	 0.0 ,	0.0 ,  0.0 ,  0.0 ,	 0.0 }; // deg
	std::vector<float> qDot = {  30.0 , 30.0 , 30.0 , 30.0 , 30.0 , 30.0 }; // deg/sec

	// ~ Pose ~
	vec3e basePos; // Position of the base , Lab Frame

protected:

	// ~ Links ~
	RobotLink* Link1 = nullptr;
	RobotLink* Link2 = nullptr;
	RobotLink* Link3 = nullptr;
	RobotLink* Link4 = nullptr;
	RobotLink* Link5 = nullptr;
	RobotLink* Link6 = nullptr;
};

// __ End UR5_OGL __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================



// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

