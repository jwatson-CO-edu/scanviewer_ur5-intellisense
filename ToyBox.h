#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
ToyBox.h
James Watson , 2018 October
Objects and Critters to render

Template Version: 2018-07-16
***********/

#ifndef TOYBOX_H // This pattern is to prevent symbols to be loaded multiple times
#define TOYBOX_H // from multiple imports

// ~~ Includes ~~
// ~ LIBNAME_i ~
// ~ Local ~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/
#include "MathGeo.h"
#include "OGL_utils.h"

// ~~ Shortcuts and Aliases ~~

// ~~ Constants ~~
const typeF MESHSIZE   = 0.005;
const typeF FOVHORZDEG = 69.4;
const typeF FOVVERTDEG = 42.5;

// === Classes and Structs =================================================================================================================

// == class PegBlock ==

class PegBlock{
    // A LEGO brick for space children
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PegBlock( float minSide , float maxSide , const matXe& bbox );
    
    void draw();

protected:
    vec3e center;
    float th;
    float ph;
    float side; // This is the scale
    std::vector<float> pegLen;
    vec3e fillColor;
    vec3e lineColor;
};

// __ End PegBlock __


// == class Icosahedron_OGL ==

class Icosahedron_OGL{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ~~ Functions ~~
    
    // ~ Con/Destructors ~
    Icosahedron_OGL( float rad , const vec3e& cntr , const vec3e& colr , float shiny );

    // ~ Setters ~
    void set_th_ph( float thNu , float phNu ); // ---- Set theta and phi to 'thNu' and 'phNu'
    void turn_th_ph( float thIncr , float phIncr ); // Increment theta and phi by 'thIncr' and 'phIncr'
    void set_emission_color( const vec3e& emitClr );
    void set_emission_intensity( float intnsty );
    
    // ~ Textures ~
    // Pick a patch from a (big) texture and assign it to each face
    void assign_face_textures_randomly( uint txtrHandle , float patchRad , uint xDim , uint yDim ); 
    
    // ~ Rendering ~
    void draw( float shiny = 0.0f ); // Render the icosahedron
    

    // ~~ Public Members ~~
    Icosahedron_e icosGeo;

protected:
    // ~ Geometry ~
    vec3e center;
    float theta    = 0.0;
    float phi      = 0.0;
    vec3e color;
    uint  numFaces = 20;
    // ~ Lighting ~
    float shininess;
    vec3e emitColor{0,0,0};
    float intensity    = 0.0;
    float emitArray[4] = { 0.0 , 0.0 , 0.0 , 1.0 };
    // ~ Textures ~
    bool   hasTextr = false;
    uint   textureHandle;
    matXe  txtrVerts;
    string txtrFile; 
};

// __ End Icosahedron_OGL __


// == class RibbonBolt ==

class RibbonBolt{
    // Class for representing a glowing beam
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // ~ Con/Destructors ~

    RibbonBolt( const vec3e& orgn , 
                const vec3e& clr , float intns ,
                float lenTravelMin , float lenTravelMax , float travelSpeed ,
                float width , float lengthMin , float lengthMax );
                
    
    // ~ Animation ~
    void advance( float time_in_sec ); // If active, Advance the animation by 'time_in_sec' and check for a reset
    
    void set_emission_intensity( float intnsty ); // Load the array that controls surface glow
    
    void activate(); // - Ensure that bolt is active and ready to animate
    void deactivate(); // Shut down animation and hide
    
    void draw();
    
protected:

    void _reset_random(); // Point the beam in a random direction and set params for a new emission
    bool  active = false; // Flag for whether the beam should be painted
    vec3e origin; // ------- Beam starts at this point
    vec3e dirctn; // ------- Beam travels in this direction
    vec3e bColor; // ------- Beam color
    float intens; // ------- Surface emission intensity
    float travlMin; // ----- Beam disappears at this distance
    float travlMax; 
    float curTravl;
    float bSpeed; // ------- Speed in [units/s]
    float wdBeam; // ------- Beam width
    float lenMin; // ------- Beam length
    float lenMax; 
    float curLen;
    float progrs = 0.0; // - Progress from 0 to lnTrav + lnBeam
    vec3e crFlat; // ------- Current direction of flatness 
    matXe sampleBox; // ---- Sample directions from this
    float emColor[4] = { 0.0f , 0.0f , 0.0f , 1.0f };
};

// __ End RibbonBolt __


// == class PatchMesh == 

// Build a matrix from a vector of lines of text assumed to contain floats
matXe matx_from_lines( const stdvec<string>& lines , char separator , size_t numCols );

// Express the points in the camera frame so that it can be meshed properly
TriMeshVFN V_to_mesh_in_cam_frame( const matXe& V , 
                                   const vec3e& origin , 
                                   const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis );

// For each of the vertices in 'Vcamera', compute the UV position within [0,1] and return correspinding UV
matXe compute_UV_for_trimesh( const matXe& Vcamera , typeF horz_FOVdeg , typeF vert_FOVdeg );

class PatchMesh{
    // Scanned objects for display
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ~ Con/Destructors ~
    PatchMesh( string fPath , bool useTxtr = false );

    ~PatchMesh();
    
    uint load_texture( string fPath = "" );
    
    // ~ Rendering ~
    
    void set_solid_color( const vec3e& clrSld );
    
    void draw( float shiny , bool smoothN = false );

    // ~ Robot ~
    stdvec<float> get_joint_state( bool deg = true ); // Get joint state that shot was taken in either deg or radians

    // ~~ Public Members ~~
    // ~ Geometry ~
    stdvec<TriMeshVFN*> patches; // Hold each processed scan segment associated with the camera shot
    matXe /* ------- */ aabb;
    // ~ Texture ~
    bool hasTextr = false;

protected:
    // ~ Geometry ~
    vec3e camOrigin;
    vec3e cam_xBasis;
    vec3e cam_yBasis;
    vec3e cam_zBasis;
    // ~ Texture ~
    string txtrPath;
    uint   textureHandle;
    matXe  txtrVerts;
    // ~ Render ~
    vec3e colorSolid; // Color to use for rendering if no texture has been applied
    // ~ Other ~
    stdvec<float> joint_state; // Represents the pose of the robot when the shot was taken
};

// __ End PatchMesh __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================



// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

