#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
DH_Robot.h
James Watson , 2018 November
Represents the UR5 Robot with simple shapes

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


// === Classes and Structs =================================================================================================================

// == struct DH_Parameters ==
struct DH_Parameters{
    std::vector<float> alpha; 
    std::vector<float> a; 
    std::vector<float> d; 
    std::vector<float> theta; 
};

DH_Parameters copy_dh_params( const DH_Parameters& origParams ); // Copy DH parameters into a new structure

bool dh_populated( const DH_Parameters& origParams ); // Are there params and are they of equal length?

// __ End DH __

// == class RobotLink ==

class RobotLink{
    // Represents a single rigid link in a serial manipulator
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    // ~~ Con/Destructors ~~
    RobotLink( float pTheta , const vec3e& pOrigin , 
               float pD_dist , float pA_dist , const vec3e& pNextRotnAxis , float pNextRotnAngl , 
               void (*pDrawFunc)(const DH_Parameters& DH) );

    ~RobotLink();

    // ~~ Configuration ~~
    void add_distal( RobotLink* link ); // Add a child link to this link
    uint get_num_distal(); // Return the number of distal links attached to this link
    bool is_leaf(); // Return true if there are no links distal to this, Otherwise return false

    // ~~ Robot Motion ~~
    void set_theta( float pTheta );
    
    // ~~ Drawing ~~
    bool has_draw_func();
    void draw( const DH_Parameters& DH );

    // ~~ Member Vars ~~
    bool enableLinkDraw = true;

protected:
    // ~ Proximal Joint ~
    float theta = 0.0;
    // ~ Relative Location ~
    vec3e origin;
    // ~ Distal Joint ~
    float d_dist = 0.0;
    float a_dist = 0.0;
    vec3e nextRotnAxis;
    float nextRotnAngl = 0.0;
    // ~ Distal Links ~
    std::vector<RobotLink*> distalLinks;
    // ~ Rendering ~
    void (*drawFunc)(const DH_Parameters& DH);
    // ~ Bookkeeping ~
    uint index = 0;
};

// __ End RobotLink __


// == class UR5_OGL ==

class UR5_OGL{
    // Represents the UR5
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ~~ Functions ~~

    // ~ Con/Destructors ~
    UR5_OGL( const vec3e& baseOrigin , const DH_Parameters& params );
    
    ~UR5_OGL(); // Delete all links , Distal-->Proximal (Order important!)

    // ~ Rendering ~
    void draw(); // Render the robot

    // ~~ Members ~~

    // ~ Joint State ~
    std::vector<float> q    = {   0.0 ,	 0.0 ,	0.0 ,  0.0 ,  0.0 ,	 0.0 }; // deg
    std::vector<float> qDot = {  30.0 , 30.0 , 30.0 , 30.0 , 30.0 , 30.0 }; // deg/sec

    void set_joint_state( const std::vector<float>& qNu );

    // ~ Pose ~
    vec3e basePos; // Position of the base , Lab Frame

protected:
    DH_Parameters params;
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

