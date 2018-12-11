/*****************************
 scanviewer.cpp
 James Watson, 2018 November
 View scan data from the robot
 ****************************/ 
/*
~~ DEV PLAN ~~
[Y] Migrate to Repo
[Y] Clean program
    [Y] Remove GLUT remnants
    [Y] Remove Spare Parts
[Y] Re-implement robot & Animate , Create an illuminated robot
    [Y] Correct Errors
    [Y] Create a UR5 class , This should emulate the the homework assignment
    [Y] Test HW3 funtionality
    [Y] Gut  HW6 elements
    [Y] Find brushed textures - NOT USED
    [Y] FIX THE BLUE LIGHTING ISSUE , IT MAKES EVERYTHING BLUE - SOLVED: Forgot to 'glDisable( GL_TEXTURE_2D )'
[Y] Troubleshoot hearbeat - Heartbeat works fine, but SDL2 has its own version of VSync - NOTE: Used heartbeat to time sim progress
[Y] Display Scan Data
    [Y] Get scans working on the robot
    [Y] Determine file format
    [Y] Load files
    [Y] Display scan meshes
    [Y] Load texture files
    [Y] Compute texture triangles
    [Y] Apply textures
[Y] Robot Control
    [Y] The robot configuration can be visualized in the configuration of each shot, 
    [Y] or moved to arbitrary configurations if the user wishes to assess its freedom in the workspace.
        [Y] Reinstate control from HW3
[Y] Make the 3D meshes selectable
    [Y] Detect mouse click
    [Y] Construct a ray from the eye to the virtual cursor
    [Y] Mesh-collision detection
        [Y] Store meshes in a list of pointers that can be iterated
        [Y] Store collision meshes in a list of pointers that can be iterated
    [Y] Put a bounding box around the selected mesh
    [Y] Click on air deselects
    [Y] Display mesh data { Num. Vertices , Num. Tris , BBox Volume }
    [Y] Deselect if the user stops showing a selected shot
[Y] Graphic improvements
    [Y] Smooth shading for scans by averaging normals at each point - COMPLETE: There is a marked difference and this is cool to point out
        [Y] Toggle smooth mesh shading to show the difference
    [Y] Make the axes on the robot smaller, except for the last link
[Y] Rename project to "final"
[Y] Configurations that show the goods
    [Y] Choose a beginning config
    [Y] Numbered configs for parts of the presentation
[ ] Disable VSync
[ ] Reshoot with the correct TCP offsets (without force sensor)
[ ] Shoot MORE objects

Project Guidelines
* How to run it
* Why is it important?

Parametric Curves
* 

*/

// === INIT ================================================================================================================================

// === Includes & Defines ===

// ~~ Standard ~~
#include <stdio.h> //- Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdlib.h> // defines four variable types, several macros, and various functions for performing general functions. , size_t
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function

// ~~ OpenGL ~~

// OpenGL with prototypes for glext
// NOTE: This MUST appear before '#include <GL/glext.h>' , otherwise will not work!
#define GL_GLEXT_PROTOTYPES // Important for all of your 5229 programs

// ~~ System-Specific Includes ~~
#ifdef __APPLE__ // This constant is always defined on Apple machines
    #include <GLUT/glut.h> // GLUT is in a different place on Apple machines
#else
    #include <GL/glut.h>
    #include <GL/glext.h>	
#endif

// ~~ SDL2 ~~
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>

// ~~ Local ~~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/
#include "MathGeo.h" // ___ Eigen + Math Utils
#include "OGL_utils.h" // _ Utility functions for 5229
#include "DH_Robot.h" // __ UR5 specification and utils
#include "ToyBox.h" // ____ Objects and Critters for Interesting Times
#include "SDL_utils.h" // _ Utility functions for SDL2

// ___ End Include ___

// === STRUCTS ===

struct CamPose{
    // Represents the position of the camera in spherical coordinates
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float rad;
    int   theta;
    int   psi;
    vec3e lookAt;
};

// ___ END STRUCT ___

// === GLOBALS ===

// ~~ Assignment ~~
string HWname = "final";

// ~~ Application ~~
bool run = true;

// ~~ View ~~
float dim /* --- */ =  2.0; // Scale Dimension
// ~ Screen ~
int   FOVy =   55; // ------------------------------ Field of view in the Y-direction [deg] (for perspective)
float w2h  = 0.0f; // ------------------------------ Aspect ratio
float Z_xt = 1.0f; // ------------------------------ Arbitrary distance from camera for an image plane
float Y_xt = tan( radians( FOVy / 2.0 ) ) * Z_xt; // Half the height of the image plane in R3 space (constant for given 'FOVy')
float X_xt; // ------------------------------------- Half the width  of the image plane in R3 space (variable with aspect ratio)

// ~ Camera ~
float camRadius     =    0.65; // --- Distance of the camera from scene center
float CAMRADINCR    =    0.0625; // - Zoom in and out by this far each keypress
int   DFLT_THETA    = -315; // ----- Initial rotation for viewing
int   DFLT_PSI      =   25; // ----- Initial elevation for viewing
int   VIEW_DEG_INCR =    5; // ----- View angle change for every key press
int   th /* ---- */ = DFLT_THETA; // Azimuth of view angle
int   ps /* ---- */ = DFLT_PSI; // - Elevation of view angle

vec3e eyeLoc{ 0 , 0 , 0 }; // ------ Camera location (world frame)


vec3e scanLocn{  0.42 , -0.48 , -0.08 }; 
vec3e robtBase{  0.00 ,  0.00 ,  0.00 };
vec3e midwLocn = ( scanLocn + robtBase ) / 2.0f;

vec3e lookPt = scanLocn; // ------ Focus of camera (world frame)

vec3e upVctr{ 0 , 0 , 0 }; // ------ Direction of "up"
vec3e lookDr; // ------------------- Direction that the camera is looking (not always used)

void assign_camera_pose( const CamPose& desPose ){
    // Set the spherical coordinates of the camera
    camRadius = desPose.rad;
    th /*- */ = desPose.theta;
    ps /*- */ = desPose.psi;
    lookPt    = desPose.lookAt;
}

stdvec<CamPose> preparedCamAngles;

// ~~ Scene ~~
vec3e gridColor{ 165.0/255 , 189.0/255 , 226.0/255 }; // Color of the world grid lines
// ~ Light Settings ~
int   emission  =   0; // Emission intensity (%)
int   ambient   =  30; // Ambient intensity (%)
int   diffuse   = 100; // Diffuse intensity (%)
int   specular  =   0; // Specular intensity (%)
int   shininess =   0; // Shininess (power of two)
float shiny     =   1; // Shininess (value)
// ~ Example Light source ~
float BALL_ORBIT_INCR = 2.5; // --- Angle increase for each button press
float ballOrbitSpeed  = 45.0; // -- Orbit speed of ball [deg/s]
float th_ball /* - */ = 0.0f; // -- Ball azimuth angle
float ps_ball /* - */ = 10.0; // -- Ball elevation angle
float rad_bal /* - */ = 1.25; // -- Radius of ball from origin
vec3e center_ball{0,0,0}; // ------ Location of the light ball in space
int   emsn_ball       =  20; // --- Emission intensity (%)
float shiny_ball      =   1; // --- Shininess (value)
float dimRad_ball     =   0.050; // Radius of the ball itself

// ~~ Data ~~
uint txtr1 , txtr2 , txtr3 , txtr4 , txtr5 , txtr6; // Textures

stdvec<float> targetJointState = { 0 , 0 , 0 , 0 , 0 , 0 };
float maxAngSpeed = 40.0; // [deg/s]

// ~~ Control ~~
// ~ Flags ~
bool BALLMOVAUTO = true;
bool ANIMATBEAMS = true;
bool SMOOTHMESHL = true;
// ~ Enums ~
enum JNTNUMBR{ JOINT1 , JOINT2 , JOINT3 , JOINT4 , JOINT5 , JOINT6 , NONE };
JNTNUMBR CURJOINT = NONE;
float DEGRINCR = 2.0;

// ~~ Geometry ~~
float AXESSCALE     =   1.17;
matXe testPoints;
vec3e cloudClr{ 102/255.0, 204/255.0, 255/255.0 };
float cloudSiz = 5.0f;

TriMeshVFN pointsMesh;
vec3e meshColor{ 153/255.0 , 51/255.0 , 255/255.0 };

bool ENABLECAMTXTR = true;

stdvec<PatchMesh*> scans; // ----- Shots with 2D, 3D, and robot pose data
stdvec<TargetVFN*> scanTargets; // Collision targets representing each scan
stdvec<size_t>     trgtDices; // - Indices of the scan associated with each patch

matXe scanBbox;
matXe trgtBbox;

stdvec<string> sourceList = {
    "robot_control/tallDino_WEST.txt"  ,
    "robot_control/tallDino_EAST.txt"  ,
    "robot_control/tallDino_SOUTH.txt" , 
    "robot_control/tallDino_NORTH.txt"
};

uint scanLen = sourceList.size();

stdvec<bool> shotFlags = { false , false , false , false , false , false , false , false , false , false };

uint MAXSHOTS = 10;

// ~~ Interaction ~~
Uint32 mouseButtonBitmask; // Mouse button state
int    winXmouse; // -------- Mouse X position in the window
int    winYmouse; // -------- Mouse Y position in the window
bool   windowHasMouse; // --- Flag for whether the window has the mouse or not
float  crossLen = 0.050; // - Crosshair length [m]
int    winW , winH; // ------ Window dimensions
float  viewXfrac = 0.0 , // _ [ -1 , 1 ] : right  --to-> left
       viewYfrac = 0.0 , // _ [ -1 , 1 ] : bottom --to-> top
       viewXcam  = 0.0 , // _ X position of the cursor on the image plane in the camera frame
       viewYcam  = 0.0 ; // _ Y position of the cursor on the image plane in the camera frame
vec3e  rayDir; // ----------- Direction that a click is pointing from the eye
// ~ Mesh Selection ~
bool /* ------ */ meshSelect = false; // Has a mesh been selected?
size_t /* ---- */ clickDex   = 0; // --- What is the index of the selected mesh?
size_t /* ---- */ selShotDex = 0; // --- What is the index of the selected shot?
FrameBases /*- */ cameraFrame; // ------ Camera reference frame
IndexSearchResult clickRayInt; // ------ Results of search for hits

// ___ END GLOBAL ___

// ___ END INIT ____________________________________________________________________________________________________________________________


// === VARIABLES & OBJECTS =================================================================================================================

// = Robot Parameters =
// Joint:                       0   ,  1      ,  2     ,  3     ,  4      ,   5      ,  6
std::vector<float> alphaRbt = { 0.0 , 90.0    ,  0.0   ,  0.0   , 90.0    , -90.0    ,  0.0    }; 
std::vector<float> aRbt     = { 0.0 ,  0.0    , -0.425 , -0.392 ,  0.0    ,   0.0    ,  0.0    }; 
std::vector<float> dRbt     = { 0.0 ,  0.0892 ,  0.0   ,  0.0   ,  0.1093 ,   0.0948 ,  0.0825 }; 
std::vector<float> thetaRbt = { 0.0 ,  0.0    ,  0.0   ,  0.0   ,  0.0    ,   0.0    ,  0.0    }; 
DH_Parameters paramsUR5{ alphaRbt , aRbt , dRbt , thetaRbt };
// static float BASEHEIGHT = 0.017;
// _ End Params _

UR5_OGL UR5{ vec3e{0,0,0} , paramsUR5 };

// ___ END VAR _____________________________________________________________________________________________________________________________


// === DRAWING =============================================================================================================================


void crosshairs( int mouseX , int mouseY , float hairLen ,
                 float& viewFracX , float& viewFracY ){
    // Draw the crosshairs as an overlay , Must be called last , Set variables to indicate where the cursor is with the viewport

    // Save transform attributes (Matrix Mode and Enabled Modes)
    glPushAttrib( GL_TRANSFORM_BIT | GL_ENABLE_BIT );

    glDisable( GL_DEPTH_TEST );

    // Save projection matrix and set unit transform
    glMatrixMode( GL_PROJECTION );
    glPushMatrix();
    glLoadIdentity();
    glOrtho( -w2h , +w2h , 
             -1   ,  1   , 
             -1   ,  1   );

    // Locate the mouse in the viewport
    viewFracX = 2 * ( (float)  mouseX / winW - 0.5 ); // [ -1 , 1 ] : right  --to-> left
    viewFracY = 2 * ( (float) -mouseY / winH + 0.5 ); // [ -1 , 1 ] : bottom --to-> top
    float Xrndr = viewFracX * w2h  ,
          Yrndr = viewFracY * 1.0f ;

    //  Save model view matrix and set to indentity
    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glLoadIdentity();

    //  Draw crosshairs
    glColor3f( 1 , 1 , 1 );

    glBegin( GL_LINES );
    /* Left  */   glVertex2f( (float) Xrndr , (float) Yrndr );   glVertex2f( (float) Xrndr - hairLen , (float) Yrndr           );
    /* Right */   glVertex2f( (float) Xrndr , (float) Yrndr );   glVertex2f( (float) Xrndr + hairLen , (float) Yrndr           );
    /* Down  */   glVertex2f( (float) Xrndr , (float) Yrndr );   glVertex2f( (float) Xrndr           , (float) Yrndr - hairLen );    
    /* Up    */   glVertex2f( (float) Xrndr , (float) Yrndr );   glVertex2f( (float) Xrndr           , (float) Yrndr + hairLen );
    glEnd();

    //  Reset model view matrix
    glPopMatrix();
    //  Reset projection matrix
    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    //  Pop transform attributes (Matrix Mode and Enabled Modes)
    glPopAttrib();

    glEnable( GL_DEPTH_TEST ); // https://stackoverflow.com/a/46036674

    /* Flip X so that we are in a right-handed coordinate system with 
       +Z pointing into the viewport, towards the scene
       +X to the left on the viewport
       +Y to the top of the viewport */
    viewFracX = -viewFracX;
}

void display( SDL_Window* window ){
    // Display the scene
    
    //  Clear the image
    glClearDepth( 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    
    //  Reset previous transforms to the identity matrix
    glLoadIdentity();
    
    // ==== Redraw ====
    
    // 1. Calculate the user view
    // Calc the eye position for perspective view (Orbit [0,0,0] spherically)
    eyeLoc = vec_sphr( camRadius , th , ps ) + lookPt;
    // lookPt = vec3e{0,0,0};
    lookDr = ( lookPt - eyeLoc ).normalized();
    upVctr = ( lookDr.cross( vec3e{0,0,1}.cross( lookDr ) ) ).normalized();
    gluLookAt( eyeLoc[0] , eyeLoc[1] , eyeLoc[2] ,  
               lookPt[0] , lookPt[1] , lookPt[2] ,  
               upVctr[0] , upVctr[1] , upVctr[2] );

    // 2. Draw the static scene
    
    // glColor3f( 1,1,1 );

    glPushMatrix();
    glTranslated( 0,0,-1 );
    draw_grid_org_XY( 0.250 , 20 , 20 , 0.5 , gridColor );
    glPopMatrix();
               
    // 2. Lighting Calcs
    //  Translate intensity to color vectors
    float Ambient[]  = { 0.01f*ambient  , 0.01f*ambient  , 0.01f*ambient  , 1.0f };
    float Diffuse[]  = { 0.01f*diffuse  , 0.01f*diffuse  , 0.01f*diffuse  , 1.0f };
    float Specular[] = { 0.01f*specular , 0.01f*specular , 0.01f*specular , 1.0f };
    //  Draw light position as ball (still no lighting here)
    // Calculate the ball position and render ball
    center_ball = vec_sphr( rad_bal , th_ball , ps_ball );
    light_ball( center_ball(0) , center_ball(1) , center_ball(2) , dimRad_ball ,
                emission , shiny );
    //  Light position
    float Position[]  = { center_ball(0) , center_ball(1) , center_ball(2) , 1.0f };
    //  OpenGL should normalize normal vectors
    glEnable( GL_NORMALIZE );
    //  Enable lighting , From this point until 'glDisable' lighting is applied
    glEnable( GL_LIGHTING );
    glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE );
    glEnable( GL_COLOR_MATERIAL );
    //  Enable light 0
    glEnable( GL_LIGHT0 );
    //  Set ambient, diffuse, specular components and position of light 0
    glLightfv( GL_LIGHT0 , GL_AMBIENT  , Ambient  );
    glLightfv( GL_LIGHT0 , GL_DIFFUSE  , Diffuse  );
    glLightfv( GL_LIGHT0 , GL_SPECULAR , Specular );
    glLightfv( GL_LIGHT0 , GL_POSITION , Position );
    
    // 3. Draw the dynamic scene

    // 12. Draw scan
    for( uint i = 0 ; i < MAXSHOTS ; i++ ){
        if(  ( i < scans.size() )  &&  ( shotFlags[i] )  )
            scans[i]->draw( shiny , SMOOTHMESHL );
    }

    // N. Draw the robot
    UR5.draw();
    
    // N. Turn off lighting
    glDisable( GL_LIGHTING );

    // N. Draw the origin
    draw_origin( 0.5 );

    // N. Draw the selected bounding boxes
    if( meshSelect ){
        draw_aabb( scanBbox , vec3e{1,1,1}                         , 2.0 );
        draw_aabb( trgtBbox , vec3e{0/255.0, 2040/255.0, 00/255.0} , 2.0 );
    }

    // == Status Message ==

    // NOTE: Text color MUST be specified before raster position for bitmap text
    // https://www.opengl.org/archives/resources/features/KilgardTechniques/oglpitfall/
    
    glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
    
    //  Display status
    glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
    
    PrintSDL( " emission %i , ambient %i , diffuse %i , specular %i , shininess %i | theta %i , psi %i" , 
              emission , ambient , diffuse , specular , shininess ,
              th , ps );

    if( meshSelect ){
        glWindowPos2i(   5 , 100 ); 
        PrintSDL( "Vertices: %i" , scanTargets[ clickDex ]->mesh.V.rows() );
        glWindowPos2i(   5 ,  80 ); 
        PrintSDL( "Facets: _ %i" , scanTargets[ clickDex ]->mesh.F.rows() );
    }
    // __ End Message __

    // ~ Draw crosshairs ~
    if( windowHasMouse ){  crosshairs( winXmouse , winYmouse , crossLen , viewXfrac , viewYfrac );  }

    // ____ End Draw ____

    // Check for errors, Flush, and swap
    ErrCheck( "display" );
    glFlush();
    SDL_GL_SwapWindow( window );
}

// ___ END DRAW ____________________________________________________________________________________________________________________________


// === INTERACTION =========================================================================================================================

bool key( const SDL_KeyboardEvent& event ){
    // SDL calls this routine when a key is pressed
    // Exit on ESC
    // NOTE: Must account for shift combinations

    // 1. Fetch the key state
    const Uint8 *state = SDL_GetKeyboardState( NULL );
    bool  SHIFTPRESS   = state[ SDL_SCANCODE_LSHIFT ]  ||  state[ SDL_SCANCODE_RSHIFT ];

    // NOTE: SDL2 seems to hear random keyboard events?, but we can corroborate by checking if it also heard a press
    // URL , Was a key pressed?: https://wiki.libsdl.org/SDL_KeyboardEvent
    if( event.state == SDL_PRESSED ){

        // 2. Repond to the event
        switch( event.keysym.sym ){
            
            // ~ Exit ~
            case SDLK_ESCAPE: // Esc: Exit the program
                // exit( 0 ); // NOPE. Have to clean up first!
                run = false;
                break;
            
            // ~ Camera Presets ~

            case SDLK_0: // 0 : Set view angles to 0
            case SDLK_KP_0: // 0 : Set view angles to 0
                th = DFLT_THETA;
                ps = DFLT_PSI;
                printf( "theta and psi reset!\n" );                
                break;

            // NOTE: The following camera presets are in presentation order

            case SDLK_KP_1: 
                assign_camera_pose( preparedCamAngles[0] ); // Set the initial camera angle
                break;
            case SDLK_KP_2: 
                assign_camera_pose( preparedCamAngles[1] ); // Scan inspection
                break;
            case SDLK_KP_3: 
            case SDLK_KP_4: 
            case SDLK_KP_5: 
            case SDLK_KP_6: 

            case SDLK_1: 
                if( !SHIFTPRESS ){
                    shotFlags[0] = !shotFlags[0];
                    if( shotFlags[0] )  targetJointState = scans[0]->get_joint_state();
                    else if( selShotDex == 0 )  meshSelect = false;
                }else{ 
                    CURJOINT = JOINT1;
                }
                break;

            case SDLK_2: 
                if( !SHIFTPRESS ){
                    shotFlags[1] = !shotFlags[1];
                    if( shotFlags[1] )  targetJointState = scans[1]->get_joint_state();
                    else if( selShotDex == 1 )  meshSelect = false;
                }else{ 
                    CURJOINT = JOINT2;
                }
                break;

            case SDLK_3: 
                if( !SHIFTPRESS ){
                    shotFlags[2] = !shotFlags[2];
                    if( shotFlags[2] )  targetJointState = scans[2]->get_joint_state();
                    else if( selShotDex == 2 )  meshSelect = false;
                }else{ 
                    CURJOINT = JOINT3;
                }
                break;

            case SDLK_4: 
                if( !SHIFTPRESS ){
                    shotFlags[3] = !shotFlags[3];
                    if( shotFlags[3] )  targetJointState = scans[3]->get_joint_state();
                    else if( selShotDex == 3 )  meshSelect = false;
                }else{ 
                    CURJOINT = JOINT4;
                }
                break;

            case SDLK_5: 
                if( SHIFTPRESS ){  CURJOINT = JOINT5;  }
                break;

            case SDLK_6: 
                if( SHIFTPRESS ){  CURJOINT = JOINT6;  }
                break;

            // ~ Robot Joint Control ~

            case SDLK_MINUS:
            case SDLK_KP_MINUS:
                if( CURJOINT != NONE ) targetJointState[CURJOINT] -= DEGRINCR;
                break;

            case SDLK_EQUALS:
            case SDLK_KP_PLUS:
                if( CURJOINT != NONE ) targetJointState[CURJOINT] += DEGRINCR;
                break;

            // ~~ Program Controls ~~

            // ~ Orbit Camera for Perspective View ~
        
            case SDLK_RIGHT: // Right arrow key - increase azimuth by 5 degrees
                th += VIEW_DEG_INCR;
                break;
            case SDLK_LEFT: // Left arrow key - decrease azimuth by 5 degrees
                th -= VIEW_DEG_INCR;
                break;
            case SDLK_UP: // Up arrow key - increase elevation by 5 degrees
                ps += VIEW_DEG_INCR;
                break;
            case SDLK_DOWN: // Down arrow key - decrease elevation by 5 degrees
                ps -= VIEW_DEG_INCR;
                break;
            case SDLK_PAGEDOWN: // Page Down Key - Zoom Out
                camRadius += CAMRADINCR;
                break;
            case SDLK_PAGEUP: // Page Up Key - Zoom in
                camRadius -= CAMRADINCR;
                break;
                
            // ~~ Camera Zoom ~~
                
            case SDLK_END: // End Key - Decrease camera radius from [0,0,0]
                rad_bal -= CAMRADINCR;
                break;
            case SDLK_HOME: // Home Key - Increase camera radius from [0,0,0]
                rad_bal += CAMRADINCR;
                break;

            // ~ Light Control ~
            case SDLK_LEFTBRACKET:
                ps_ball -= BALL_ORBIT_INCR;
                break;
            case SDLK_RIGHTBRACKET:
                ps_ball += BALL_ORBIT_INCR;
                break;
            case SDLK_m:
                toggle( BALLMOVAUTO );
                break;
            case SDLK_QUOTE:
                th_ball -= BALL_ORBIT_INCR;
                break;
            case SDLK_SLASH:
                th_ball += BALL_ORBIT_INCR;
                break;

            // ~ Light Properties ~
            case SDLK_a: // Ambient level
                if( SHIFTPRESS ){
                    if( ambient < 100 ) ambient += 5;
                }else{
                    if( ambient >   0 ) ambient -= 5;}
                break;
            case SDLK_d: // Diffuse level
                if( SHIFTPRESS ){
                    if( diffuse < 100 ) diffuse += 5;
                }else{
                    if( diffuse >   0 ) diffuse -= 5;}
                break;
            case SDLK_s: // Specular level
                if( SHIFTPRESS ){
                    if( specular < 100 ) specular += 5;
                }else{
                    if( specular >   0 ) specular -= 5;}
                break;
            case SDLK_e: // Emission level
                if( SHIFTPRESS ){
                    if( emission < 100 ) emission += 5;
                }else{
                    if( emission >   0 ) emission -= 5;}
                break;
            case SDLK_n: //  Shininess level
                if( SHIFTPRESS ){
                    if( shininess <  7 ) shininess += 1;
                }else{
                    if( shininess > -1 ) shininess -= 1;}
                break;

            // ~ Light Presets ~
            case SDLK_o: // October Mode
                emission  = 95;
                ambient   =  0;
                diffuse   =  5;
                specular  =  5;
                shininess =  1;
                break;
            case SDLK_p: // Pretty Mode
                emission  =  0;
                ambient   = 25;
                diffuse   = 90;
                specular  =  5;
                shininess =  3;
                break;
                


            // ~ Other ~
            case SDLK_b: // Toggle beams
                toggle( ANIMATBEAMS );
                break;

            case SDLK_h: // Toggle smooth shading for meshes
                toggle( SMOOTHMESHL );
                break;
        
            // <?> : Keys are nice, I guess!
                
            default :
                // printf( "There is no function for this key!\n" );
                // Do nothing , Do not notify unhandled events
                break;
        }
    }

    //  Translate shininess power to value (-1 => 0)
    shiny = shininess<0 ? 0 : pow(2.0,shininess);

    return true;
}

void reshape( int width , int height ){
    // GLUT calls this routine when the window is resized
    // Calc the aspect ratio: width to the height of the window
    w2h = ( height > 0 ) ? (float) width / height : 1;
    // Set the viewport to the entire window
    glViewport( 0 , 0 , width , height );
    // Set projection
    Project( FOVy , w2h , dim );
    // Calc params for lab-space position of cursor
    X_xt = w2h * Y_xt;
}

IndexSearchResult scan_index_of_closest_patch_intersection_with_ray( const vec3e& rayOrg , const vec3e& rayDir ){
    // Search each of the patches of each of the scans to see if it intersects , Return the closest to the camera
    IndexSearchResult result = default_false_result();
    RayHits /* --- */ intersections;
    RayHits /* --- */ accumHits;
    size_t numPatches = scanTargets.size() ,
           numEntr    = 0                  ,
           numExit    = 0                  ;
    // For each of the patch targets, accumulate hits
    for( size_t i = 0 ; i < numPatches ; i++ ){  
        if( shotFlags[ trgtDices[i] ] ){
            intersections = ray_intersect_TargetVFN( rayOrg , rayDir , *(scanTargets[i]) );
            // numEntr = intersections.enter.rows();
            // numExit = intersections.exit.rows();
            assign_num_entries_exits( intersections , numEntr , numExit );
            for( size_t j = 0 ; j < numEntr ; j++ ){  intersections.n_Index.push_back( i );  }
            for( size_t j = 0 ; j < numExit ; j++ ){  intersections.x_Index.push_back( i );  }
            accumHits += intersections;
        }
    }
    // Get the row of the hit that is closest to the ray origin
    // NOTE: Since the meshes in this 
    IndexTypeFResult closestEntr = closest_point_to_sq( accumHits.enter , rayOrg );
    IndexTypeFResult closestExit = closest_point_to_sq( accumHits.exit  , rayOrg );
    if( closestEntr.result ){
        if( closestExit.result ){
            if( closestEntr.measure < closestExit.measure ){
                result.result = true;
                result.index  = accumHits.n_Index[ closestEntr.index ];
            }else{
                result.result = true;
                result.index  = accumHits.x_Index[ closestExit.index ];
            }
        }else{
            result.result = true;
            result.index  = accumHits.n_Index[ closestEntr.index ];
        }
    }else if( closestExit.result ){
        result.result = true;
        result.index  = accumHits.x_Index[ closestExit.index ];
    }else{ 
        result.result = false;
        result.index  = 0;
    }
    return result;
}

FrameBases get_current_camera_frame(){
    // Get a right-handed coordinate system representing the camera
    FrameBases rtnStruct;
    rtnStruct.origin = eyeLoc;
    rtnStruct.zBasis = lookDr;
    rtnStruct.yBasis = upVctr;
    rtnStruct.xBasis = rtnStruct.yBasis.cross( rtnStruct.zBasis ).normalized();
    return rtnStruct;
}

// ___ END INTERACT ________________________________________________________________________________________________________________________


// === SIMULATION ==========================================================================================================================

double _time_elapsed;
double _last_time = 0.0;

// ___ END SIM _____________________________________________________________________________________________________________________________


// === MAIN ================================================================================================================================

// == Test Vars ==



// __ End Test __


// Start up SDL2 and tell it what to do
int main( int argc , char* argv[] ){
    rand_init(); // initialize random seed based on system clock

    // Load Data
    size_t len = 0;
    for( uint i = 0 ; i < scanLen ; i++ ){  
        scans.push_back( new PatchMesh( sourceList[i] , ENABLECAMTXTR ) );  
        len = scans[i]->patches.size();
        for( size_t j = 0 ; j < len ; j++ ){
            scanTargets.push_back( heap_target_from_trimesh( *(scans[i]->patches[j]) ) );
            trgtDices.push_back( i );
        }    
    }

    // Set up camera angles for presentation
    /* 1. Robot motion */  preparedCamAngles.push_back( CamPose{ 1.0 , -315 , 25 , midwLocn + vec3e{ 0 , 0 , 0.25 } } );
    /* 1. Scan select  */  preparedCamAngles.push_back( CamPose{ 0.4 , -250 , 15 , scanLocn + vec3e{ 0 , 0 , 0.05 } } );


    assign_camera_pose( preparedCamAngles[0] ); // Set the initial camera angle
    
    // 0. Start an OGL context
    
    //  Initialize SDL
    SDL_Init( SDL_INIT_VIDEO );
    SDL_Window*      displayWindow;
    SDL_Renderer*    displayRenderer;
    SDL_RendererInfo displayRendererInfo;

    displayWindow = SDL_CreateWindow(
        ( "James Watson , " + HWname ).c_str() , // window title
        SDL_WINDOWPOS_UNDEFINED, // --------------- initial x position
        SDL_WINDOWPOS_UNDEFINED, // --------------- initial y position
        800, // ----------------------------------- width  , in pixels
        600, // ----------------------------------- height , in pixels
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE // flags , see below
    ); 

    displayRenderer = SDL_CreateRenderer( displayWindow , -1 , SDL_RENDERER_PRESENTVSYNC ); 

    SDL_GL_SetSwapInterval( 0 ); // Immediate updates
    // SDL_GL_SetSwapInterval( 1 ); // Updates synchronized with the vertical retrace

    SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE , 32 ); // minimum number of bits in the depth buffer

    SDL_GetRendererInfo( displayRenderer , &displayRendererInfo );
    /* TODO: Check that we have OpenGL */
    if( ( displayRendererInfo.flags & SDL_RENDERER_ACCELERATED   ) == 0 || 
        ( displayRendererInfo.flags & SDL_RENDERER_TARGETTEXTURE ) == 0 ){
        /*TODO: Handle this. We have no render surface and not accelerated. */
        cout << "BAD SDL WINDOW!" << endl;
    }
    
    SDL_SetWindowTitle( displayWindow , "Final Project, James Watson" );

    //  Set screen size  &&  Init
    SDL_GetWindowSize( displayWindow , &winW , &winH );
    reshape( winW , winH );

    // Hide cursor in the window
    SDL_ShowCursor( SDL_DISABLE );
    
    // Set heartbeat for 60 fps
    SDL_Heartbeat hb{ 1.0f / 60.0f };  hb.mark_time();
    
    glEnable( GL_DEPTH_TEST ); // https://stackoverflow.com/a/46036674

    // Start in Pretty Mode
    emission  =  0;
    ambient   = 25;
    diffuse   = 90;
    specular  =  5;
    shininess =  3;
    
    // ~~ Init Work ~~

    // Load the textures onto each of the clusters associated with each shot
    for( uint i = 0 ; i < scans.size() ; i++ ){  scans[i]->load_texture();  }

    
    /// ===== Main SDL event loop ==========================================================================================================
    
    ErrCheck( "init" );
    
    float t0 /* ---- */ = 0.0f;
    float dt /* ---- */ = 1.0f / 60.0f; // NOPE
    float _time_elapsed = dt; // This is fixed time for state updates

    std::vector<float> currQ = { 0,0,0,0,0,0 }; // Joint state to send to the robot
    stdvec<float>      diffQ = { 0,0,0,0,0,0 }; // Difference between the current and the desired joint state
    float /* ------ */ frameSpeed;

    // while the run flag is active
    while( run ){
        
        // 1. Get elapsed time in seconds
        double t = SDL_GetTicks() / 1000.0;
        
        // 2. Process all pending events
        SDL_Event event;
        while( SDL_PollEvent( &event ) ){
            switch( event.type ){
                case SDL_WINDOWEVENT:
                    switch( event.window.event ){
                        case SDL_WINDOWEVENT_RESIZED:
                            reshape( event.window.data1 , event.window.data2 );
                            break;
                        // URL , SDL2 mouse enter and leave window: http://lazyfoo.net/tutorials/SDL/35_window_events/index.php
                        case SDL_WINDOWEVENT_ENTER: 
                            windowHasMouse = true;
                            break;
                        case SDL_WINDOWEVENT_LEAVE:
                            windowHasMouse = false;
                            break;
                        default:
                            // Do nothing , Do not notify unhandled events
                            break;
                    }
                    break;
                case SDL_QUIT:
                    run = 0;
                    break;
                case SDL_KEYDOWN:
                    key( event.key );
                    t0 = t + 0.5; // Wait 1/2 s before repeating
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    switch( event.button.button ){
                        // Is 'SDL_BUTTON_LEFT' still the primary click for a left-handed mouse?
                        case SDL_BUTTON_LEFT:
                            cerr << endl << "LEFT CLICK!" << endl << endl;

                            // A. Constuct a ray from the eye to the viewport
                            viewXcam  = viewXfrac * X_xt; // X position of the cursor on the image plane in the camera frame
                            viewYcam  = viewYfrac * Y_xt; // Y position of the cursor on the image plane in the camera frame
                            rayDir    = vec3e( viewXcam , viewYcam , Z_xt );
                            // B. Express the direction in the lab frame
                            cameraFrame = get_current_camera_frame();
                            rayDir = transform_vec( rayDir , cameraFrame.xBasis , cameraFrame.yBasis , cameraFrame.zBasis );
                            // C. Intersect the transformed ray with all of the active meshes
                            clickRayInt = scan_index_of_closest_patch_intersection_with_ray( cameraFrame.origin , rayDir );
                            // D. Unpack results
                            meshSelect = clickRayInt.result;
                            clickDex   = clickRayInt.index;
                            selShotDex = trgtDices[ clickDex ];
                            // E. Load AABB for the patch and for the shot
                            cerr << "Hit?: " << yesno( meshSelect ) << " , Target Index: " << clickDex << " , Shot Index: " << selShotDex << endl;
                            scanBbox = scans[ selShotDex ]->aabb;
                            trgtBbox = AABB( *scanTargets[ clickDex ] );

                        // Other mouse buttons if you need them
                        case SDL_BUTTON_MIDDLE:
                        case SDL_BUTTON_RIGHT:
                        case SDL_BUTTON_X1:
                        case SDL_BUTTON_X2:
                            break;
                    }
                    break;
            }
            
            //  Repeat key every 50 ms
            if( t - t0 > 0.05 ){
                key( event.key );
                t0  = t;
            }
        }

        // 3. Mouse interaction
        if( windowHasMouse ){
            mouseButtonBitmask = SDL_GetMouseState( &winXmouse , &winYmouse );
        }else{ 
            winXmouse = -1;  winYmouse = -1;
        }
        // cerr << "Mouse X: " << winXmouse << " , Mouse Y: " << winYmouse << endl;

        
        // 3. Draw
        display( displayWindow );
        
        // cerr << "viewXcam: " << viewXcam << " , viewYcam: " << viewYcam << endl;

        // 4. Calculate the next frame
        _time_elapsed = (float) hb.seconds_elapsed();

        // ~ Update light ball ~
        if( BALLMOVAUTO ){  th_ball += _time_elapsed * ballOrbitSpeed;  }
        th_ball = fmod( th_ball , 360.0f );

        // ~ Move the joints ~
        diffQ = targetJointState - currQ; // Get the difference between current and desired
        frameSpeed = _time_elapsed * maxAngSpeed; // Calc max angle we can move this frame
        diffQ = clamp_vec( diffQ , -frameSpeed ,  frameSpeed ); // Limit difference to max angle
        currQ += diffQ; // Update current angle towards desired
        UR5.set_joint_state( currQ ); // Send angle to robot

        // N-1. Error check
        // ErrCheck( "loop" ); // DEV: Will this slow the program down?

        // N. Mark time for next update interval
        hb.mark_time();
    }
    
    /// _____ END MAIN LOOP ________________________________________________________________________________________________________________
    
    // ~~ Cleanup ~~
    cerr << "About to clean scans ..." << endl;
    clearif( scans );
    cerr << "About to clean targets ..." << endl;
    clearif( scanTargets );
    cerr << "About to check errors ... ";
    cerr << ( ErrCheck( "exit" ) ? "Error(s) found!" : "OK!" ) << endl;
    cerr << "EXIT!" << endl;

    //  Return code
    return 0;
}

// ___ END MAIN ____________________________________________________________________________________________________________________________


/* === SPARE PARTS =========================================================================================================================



   ___ END PARTS ___ */
