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
[ ] Re-implement robot & Animate , Create an illuminated robot
	[Y] Correct Errors
	[Y] Create a UR5 class , This should emulate the the homework assignment
	[Y] Test HW3 funtionality
	[Y] Gut  HW6 elements
    [Y] Find brushed textures
    [ ] Pick shiny material props for the arm beams
    [ ] Camera at robot gripper
[Y] Troubleshoot hearbeat - Heartbeat works fine, but SDL2 has its own version of VSync
[ ] Display Scan Data
    [Y] Get scans working on the robot
    [Y] Determine file format
    [Y] Load files
    [ ] Display scan meshes
    [ ] Load texture files
    [ ] Compute texture triangles
    [ ] Apply textures
{ } Robot IK


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

// === GLOBALS ===

// ~~ Assignment ~~
string HWname = "HW7";

// ~~ View ~~
float dim /* --- */ =  2.0; // Scale Dimension
// ~ Screen ~
float w2h = 0.0f; // Aspect ratio
// ~ Camera ~
float camRadius     =    0.65; // --- Distance of the camera from scene center
float CAMRADINCR    =    0.0625; // - Zoom in and out by this far each keypress
int   DFLT_THETA    = -315; // ----- Initial rotation for viewing
int   DFLT_PSI      =   25; // ----- Initial elevation for viewing
int   VIEW_DEG_INCR =    5; // ----- View angle change for every key press
int   th /* ---- */ = DFLT_THETA; // Azimuth of view angle
int   ps /* ---- */ = DFLT_PSI; // - Elevation of view angle
int   fov /* --- */ = 55; // ------- Field of view (for perspective)
vec3e eyeLoc{ 0 , 0 , 0 }; // ------ Camera location (world frame)

vec3e lookPt{ 0.42 , -0.48 , -0.08 }; // ------ Focus of camera (world frame)
// vec3e lookPt{ 0.0 , 0.0 , 0.0 }; // ------ Focus of camera (world frame)

vec3e upVctr{ 0 , 0 , 0 }; // ------ Direction of "up"
vec3e lookDr; // ------------------- Direction that the camera is looking (not always used)

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
float BALL_ORBIT_INCR = 2.5; // ---- Angle increase for each button press
float ballOrbitSpeed  = 45.0; // --- Orbit speed of ball [deg/s]
float th_ball /* - */ = 0.0f; // --- Ball azimuth angle
float ps_ball /* - */ = 10.0; // Ball elevation angle
float rad_bal /* - */ = 1.25; // ---- Radius of ball from origin
vec3e center_ball{0,0,0}; // ------- Location of the light ball in space
int   emsn_ball       =  20; // ----- Emission intensity (%)
float shiny_ball      =   1; // ----- Shininess (value)
float dimRad_ball     =   0.050; // - Radius of the ball itself

// ~~ Data ~~
uint txtr1 , txtr2 , txtr3 , txtr4 , txtr5 , txtr6; // Textures

stdvec<float> targetJointState = { 0 , 0 , 0 , 0 , 0 , 0 };
float maxAngSpeed = 30.0; // [deg/s]

// ~~ Control ~~
// ~ Flags ~
bool BALLMOVAUTO = true;
bool ANIMATBEAMS = true;
// ~ Enums ~

// ~~ Geometry ~~
float AXESSCALE     =   1.17;
matXe testPoints;
vec3e cloudClr{ 102/255.0, 204/255.0, 255/255.0 };
float cloudSiz = 5.0f;

TriMeshVFN pointsMesh;
vec3e meshColor{ 153/255.0 , 51/255.0 , 255/255.0 };

bool ENABLECAMTXTR = true;

PatchMesh testScan1{ "robot_control/tallDino_WEST.txt" , ENABLECAMTXTR };
vec3e meshColor1{ 153/255.0 , 51/255.0 , 255/255.0 };

PatchMesh testScan2{ "robot_control/tallDino_EAST.txt" , ENABLECAMTXTR };
vec3e meshColor2{ 0/255.0, 153/255.0, 204/255.0 };

PatchMesh testScan3{ "robot_control/tallDino_SOUTH.txt" , ENABLECAMTXTR };
vec3e meshColor3{ 0/255.0, 153/255.0, 51/255.0 };

PatchMesh testScan4{ "robot_control/tallDino_NORTH.txt" , ENABLECAMTXTR };
vec3e meshColor4{ 204/255.0, 0/255.0, 204/255.0 };

bool SHOT1 = true  , 
     SHOT2 = false , 
     SHOT3 = false , 
     SHOT4 = false ;

// ___ END GLOBAL ___

// ___ END INIT ____________________________________________________________________________________________________________________________


// === FUNCTIONS ===========================================================================================================================



// ___ END FUNC ____________________________________________________________________________________________________________________________


// === CLASSES =============================================================================================================================




// ___ END CLASS ___________________________________________________________________________________________________________________________


// === VARIABLES & OBJECTS =================================================================================================================
//						  float rad , const vec3e& cntr , const vec3e& colr , float shiny
// Icosahedron_OGL icosTest{ 0.5 , vec3e{0,0,0} , vec3e{0,1,0} , 5.5 };
// std::vector<Icosahedron_OGL*> nodules;
// vec3e RXcolor{ 0.0/255 , 204.0/255 , 102.0/255 };
// std::vector<RibbonBolt*> particles;

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
	lookDr = lookPt - eyeLoc;
	upVctr = ( lookDr.cross( vec3e{0,0,1}.cross( lookDr ) ) ).normalized();
	gluLookAt( eyeLoc[0] , eyeLoc[1] , eyeLoc[2] ,  
			   lookPt[0] , lookPt[1] , lookPt[2] ,  
			   upVctr[0] , upVctr[1] , upVctr[2] );
			   

	// 2. Draw the static scene
	// Draw a grid
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
	
	// //  Enable light 1
	// glEnable( GL_LIGHT1 );
	// float virsCntr[]  = { 0.0f , 0.0f , 0.0f , 1.0f };
	// glLightfv( GL_LIGHT1 , GL_POSITION , virsCntr );
	// float virsColr[] = { RXcolor(0) * 0.01f * emission , RXcolor(1) * 0.01f * emission , RXcolor(2) * 0.01f * emission , 1.0f };
	// float virsSpec[] = { RXcolor(0) * 0.01f * emission * 0.01f * specular , 
	// 					 RXcolor(1) * 0.01f * emission * 0.01f * specular , 
	// 					 RXcolor(2) * 0.01f * emission * 0.01f * specular , 
	// 					 1.0f };
	// glLightfv( GL_LIGHT1 , GL_DIFFUSE  , virsColr );
	// glLightfv( GL_LIGHT1 , GL_SPECULAR , virsSpec );
	

	// 10. draw particles
	// for( uint i = 0 ; i < 20 ; i++ ){  particles[i]->draw();  }

    // for( uint i = 0 ; i < testScan.patches.size() ; i++ ){
    //     draw_aabb( AABB( testScan.patches[i]->V ) , vec3e{0,1,0} , 2.0 );
    //     draw_point_cloud( testScan.patches[i]->V , cloudSiz , cloudClr );
    // }
	
	//~ // 11. Draw cloud
	//~ draw_point_cloud( testPoints , cloudSiz , cloudClr );
    //~ draw_trimesh( pointsMesh , meshColor , shiny );

	// 12. Draw scan
	if( SHOT1 ) testScan1.draw( shiny );
    if( SHOT2 ) testScan2.draw( shiny );
    if( SHOT3 ) testScan3.draw( shiny );
    if( SHOT4 ) testScan4.draw( shiny );

	// N. Draw the robot
	UR5.draw();
	

	glDisable( GL_LIGHTING );

	// N. Draw the origin
	draw_origin( 0.5 );


	// == Status Message ==

	// NOTE: Text color MUST be specified before raster position for bitmap text
	// https://www.opengl.org/archives/resources/features/KilgardTechniques/oglpitfall/
	
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
	
	//  Display status
	glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
	
	PrintSDL( " emission %i , ambient %i , diffuse %i , specular %i , shininess %i | theta %i , psi %i" , 
		   emission , ambient , diffuse , specular , shininess ,
		   th , ps );
		   
	// __ End Message __

	// ____ End Draw ____

	// Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	// glutSwapBuffers();
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


	// 2. Repond to the event
	switch( event.keysym.sym ){
		case SDLK_ESCAPE: // Esc: Exit the program
			exit( 0 );
		case SDLK_0: // 0 : Set view angles to 0
		case SDLK_KP_0: // 0 : Set view angles to 0
			th = DFLT_THETA;
			ps = DFLT_PSI;
			printf( "theta and psi reset!\n" );
			break;

        case SDLK_1: 
		case SDLK_KP_1: 
            SHOT1 = !SHOT1;
            if( SHOT1 )  targetJointState = testScan1.get_joint_state();
            break;

        case SDLK_2: 
		case SDLK_KP_2: 
            SHOT2 = !SHOT2;
            if( SHOT2 )  targetJointState = testScan2.get_joint_state();
            break;

        case SDLK_3: 
		case SDLK_KP_3: 
            SHOT3 = !SHOT3;
            if( SHOT3 )  targetJointState = testScan3.get_joint_state();
            break;

        case SDLK_4: 
		case SDLK_KP_4: 
            SHOT4 = !SHOT4;
            if( SHOT4 )  targetJointState = testScan4.get_joint_state();
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
	
		// <?> : Keys are nice, I guess!
			
		default :
			// printf( "There is no function for this key!\n" );
			// Do nothing , Do not notify unhandled events
			break;
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
	Project( fov , w2h , dim );
}

// ___ END INTERACT ________________________________________________________________________________________________________________________


// === SIMULATION ==========================================================================================================================

double _time_elapsed;
double _last_time = 0.0;

// ___ END SIM _____________________________________________________________________________________________________________________________


// === MAIN ================================================================================================================================

// == Test Vars ==

const vec3e test1{    1.0f , 0.0f , 0.0f };
const vec3e test2 = { 2.0f , 0.0f , 0.0f };
	  vec3e test3{    3.0f , 0.0f , 0.0f };
	  vec3e test4 = { 4.0f , 0.0f , 0.0f };

// __ End Test __


// Start up GLUT and tell it what to do
int main( int argc , char* argv[] ){
	rand_init(); // initialize random seed based on system clock

	// Run tests
	cerr << "test1: " << test1 << endl;
	cerr << "test2: " << test2 << endl;
	cerr << "test3: " << test3 << endl;
	cerr << "test4: " << test4 << endl;
	
	// 0. Start an OGL context
	int winW , winH;
	
	//  Initialize SDL
	SDL_Init( SDL_INIT_VIDEO );
    SDL_Window*      displayWindow;
    SDL_Renderer*    displayRenderer;
    SDL_RendererInfo displayRendererInfo;

	displayWindow = SDL_CreateWindow(
        ( "James Watson , " + HWname ).c_str() ,                  // window title
        SDL_WINDOWPOS_UNDEFINED,           // initial x position
        SDL_WINDOWPOS_UNDEFINED,           // initial y position
        800,                               // width, in pixels
        600,                               // height, in pixels
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE                // flags - see below
    ); 

	displayRenderer = SDL_CreateRenderer( displayWindow , -1 , SDL_RENDERER_PRESENTVSYNC ); 
	// SDL_GL_SetSwapInterval( 0 ); // Immediate updates
	SDL_GL_SetSwapInterval( 1 ); // Updates synchronized with the vertical retrace

	SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE , 32 );


    SDL_GetRendererInfo( displayRenderer , &displayRendererInfo );
    /* TODO: Check that we have OpenGL */
    if( ( displayRendererInfo.flags & SDL_RENDERER_ACCELERATED   ) == 0 || 
        ( displayRendererInfo.flags & SDL_RENDERER_TARGETTEXTURE ) == 0 ){
        /*TODO: Handle this. We have no render surface and not accelerated. */
        cout << "BAD SDL WINDOW!" << endl;
	}
	
	//~ SDL_WM_SetCaption( "More Lighting - SDL" , "sdl20" );
	SDL_SetWindowTitle( displayWindow , "More Lighting - SDL" );

	//  Set screen size
	SDL_GetWindowSize( displayWindow , &winW , &winH );
	reshape( winW , winH );
	
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
	
	// ~ Load textures ~
	// txtr1 = LoadTexBMP( "textures/triBrownHuge.bmp" );
	// txtr2 = LoadTexBMP( "textures/triDirtHuge.bmp" );
	// txtr3 = LoadTexBMP( "textures/oldBoxesSml.bmp" );
	// txtr4 = LoadTexBMP( "textures/triRando.bmp" );
	// txtr5 = LoadTexBMP( "textures/mineralGreen.bmp" );
	// txtr6 = LoadTexBMP( "textures/concrete.bmp" );
	
	testScan1.load_texture();
	testScan2.load_texture();
	testScan3.load_texture();
	testScan4.load_texture();
	
	// ~ Create objects ~
	// icosTest.set_emission_color( RXcolor );
	// icosTest.assign_face_textures_randomly( txtr1 , 100.0f , 1024 , 512 );
	
	// float lenTravelMin = 0.5f;
	// float lenTravelMax = 2.0f;
	// float travelSpeed  = 1.75f;
	// float width /* -*/ = 0.0075;
	// float lengthMin    = 0.010;
	// float lengthMax    = 1.5f;
	
	// for( uint i = 0 ; i < 20 ; i++ ){
		
	// 	nodules.push_back(  
	// 		new Icosahedron_OGL( 0.125 , vec3e{0,0,0} , vec3e{255.0/255, 102.0/255, 0.0/255} , 5.5 )
	// 	);
	// 	nodules[i]->assign_face_textures_randomly( txtr5 , 40.0f , 512 , 128 );
		
	// 	particles.push_back(
	// 		new RibbonBolt( vec3e{0,0,0} , 
	// 						RXcolor , 1.0f ,
	// 						lenTravelMin , lenTravelMax , travelSpeed ,
	// 						width , lengthMin , lengthMax )
	// 	);
		
	// }
	
	// // ~  Create Points ~
	// size_t numPts = 100;
	// matXe sampleBox = matXe::Zero( 2 , 3 );	
	// sampleBox << 0.5 , 0.5 , 0.00 ,
	// 			 1.0 , 1.0 , 0.25 ;
	// testPoints = sample_from_AABB( numPts , sampleBox );

    // // ~ Mesh Points ~
    // pointsMesh = delaunay_from_V( testPoints );

    // ~ Read files ~
    stdvec<string> fNames = { "tallDino_NORTH.txt" , "tallDino_SOUTH.txt" , 
                              "tallDino_EAST.txt"  , "tallDino_WEST.txt"  };
    // uint numNames = fNames.size();
    stdvec<string> lines;
    
    testScan1.set_solid_color( meshColor1 );
    testScan2.set_solid_color( meshColor2 );
    testScan3.set_solid_color( meshColor3 );
    testScan4.set_solid_color( meshColor4 );

    //~ for( uint i = 0 ; i < numNames ; i++ ){
        //~ lines = readlines( "robot_control/" + fNames[i] ); // Return all the lines of text file as a string vector
        //~ printlines( lines ); // Print all the lines read from a file
    //~ }
	
	/// ===== Main SDL event loop ==========================================================================================================
	
	ErrCheck( "init" );
	
	bool  run = true;
	float t0  = 0.0f;
	float dt = 1.0f / 60.0f;
	float _time_elapsed = dt; // This is fixed time for state updates

	std::vector<float> currQ = { 0,0,0,0,0,0 };
    stdvec<float>      diffQ = { 0,0,0,0,0,0 };
    float /* ------ */ frameSpeed;

	bool QUITEARLY = false;
	
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
						default:
							// Do nothing , Do not notify unhandled events
							break;
					}
					break;
				case SDL_QUIT:
					run = 0;
					break;
				case SDL_KEYDOWN:
					run = key( event.key );
					t0 = t + 0.5;  // Wait 1/2 s before repeating
					break;
			}
			
			//  Repeat key every 50 ms
			if( t - t0 > 0.05 ){
				run = key( event.key );
				t0  = t;
			}
		}
		
		// 3. Draw
		display( displayWindow );
		
		// 4. Calculate the next frame
		
		// Update light ball
		if( BALLMOVAUTO ){  th_ball += _time_elapsed * ballOrbitSpeed;  }
		th_ball = fmod( th_ball , 360.0f );
		
		// // Set number of particles active in proportion to emissitivity of the core
		// // Core begins to emit rays at emissitivity >= 50
		// uint numActive = (uint) ( emission > 49 ? ( emission * 1.5 / 5 ) : 0 ); 
		// // Update particles
		// for( uint i = 0 ; i < 20 ; i++ ){
		// 	if( ANIMATBEAMS ){
		// 		if( i < numActive ){
		// 			particles[i]->activate(); 
		// 			particles[i]->set_emission_intensity( emission );
		// 			particles[i]->advance( _time_elapsed );
		// 		}else{
		// 			particles[i]->deactivate(); 
		// 		}
		// 	}else{  particles[i]->deactivate();  }
		// }

		// Move the joints
		

        diffQ = targetJointState - currQ;
        frameSpeed = (float) hb.seconds_elapsed() * maxAngSpeed;
        diffQ = clamp_vec( diffQ , -frameSpeed ,  frameSpeed );
        currQ += diffQ;

		UR5.set_joint_state( currQ );
		
		// N. Sleep for remainder
		hb.sleep_remainder(); // Not really needed with VSYNC, but just in case

		// N+1. Quit for Debug
		if( QUITEARLY ){  break;  }
	}
	
	/// _____ END MAIN LOOP ________________________________________________________________________________________________________________
	
	
	//  Return code
	return 0;
}

// ___ END MAIN ____________________________________________________________________________________________________________________________


/* === SPARE PARTS =========================================================================================================================



   ___ END PARTS ___ */
