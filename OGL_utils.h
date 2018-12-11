#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
OGL_utils.h
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

#ifndef OGL_UTILS_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_UTILS_H // from multiple imports

// ~~ Defines ~~
#define LEN 8192 // ---------- Maximum length of text string
#define _USE_MATH_DEFINES // - M_PI , etc.
#define GL_GLEXT_PROTOTYPES // Important for all of your programs
// ~ Which system support text? ~
// #define TEXTWITHGLUT
#define TEXTWITHSDL2


// ~~ Imports ~~
// ~ Standard ~
#include <stdio.h> // - Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdarg.h> //- macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function
#include <math.h> // -- ceilf
#include <stdbool.h> // Why isn't this a part of every language since ever?

// ~ OpenGL ~
#ifdef __APPLE__ // This constant is always defined on Apple machines
      #include <GLUT/glut.h> // GLUT is in a different place on Apple machines
#else
      #include <GL/glut.h>
#endif

// ~ Eigen ~
#include <Eigen/OpenGLSupport>

// ~ Local ~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/
#include "MathGeo.h"
#include "SDL_utils.h"



// ~~ Constants ~~
const uint NUMCYLFACETS = 75;
const float MATL_WHITE[] = { 1 , 1 , 1 , 1 };
const float MATL_BLACK[] = { 0 , 0 , 0 , 1 };

// ~~ Shortcuts and Aliases ~~



// === Classes and Structs =================================================================================================================



// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

bool ErrCheck( const char* where ); // --- See if OpenGL has raised any errors , Tag with 'where' for info
bool ErrCheck( string where );

void Fatal( const char* format , ... ); // Scream and Run

void glVec3e( const vec3e& v ); // Set a vertex with an Eigen vector
void glNrm3e( const vec3e& n ); // Set a normal with an Eigen vector
void glClr3e( const vec3e& c ); // Set the color with an Eigen vector
void glTxr2e( const vec2e& v ); // Set a texture vertex with and Eigen vector

void Print( const char* format , ... ); // Convenience routine to output raster text , Use VARARGS to make this more flexible   
void Print( string format      , ... ); // Convenience routine to output raster text , Use VARARGS to make this more flexible (std::string version)

void draw_origin( float scale ); //  Draw scaled axes at origin in [R,G,B] for [X,Y,Z]

// Draw a square grid centered at the origin, extending 'xPlusMinus' units in X and 'yPlusMinus' units in Y
void draw_grid_org_XY( float gridSize , uint xPlusMinus , uint yPlusMinus , 
					   float lineThic , vec3e color );
					   
void Vertex_sphr( float th , float ph ); // Draw vertex in polar coordinates

void sphere2( float x , float y , float z , float r ); // Draw a sphere (version 2) at (x,y,z) radius (r)

// Draw a cube at (x,y,z) dimensions (dx,dy,dz) and an outline
void cube( float x , float y , float z ,
           float dx , float dy , float dz ,
           float fillColor[3] , float lineColor[3] );
           
// Draw a cylinder of 'length' and 'radius' with the center of one end at 'origin', with an 'axisDir'
void draw_cylinder( const vec3e& origin  , typeF length , typeF radius , uint facets ,
					const vec3e& fillClr , float shiny ,
					uint txtrHandle = 9999 );

// Draw a glowing white ball at (x,y,z) radius (r)
void light_ball( float x , float y , float z , float r ,
                 int emission , float shiny );

void Reverse( void* x , const int n ); // Reverse n bytes

unsigned int LoadTexBMP( const char* file ); // Load texture from BMP file

void Project( float FOVy , float w2h , float dim ); // Set projection

void draw_point_cloud( const matXe& cloud , typeF pntSize , const vec3e& color ); // Render a bunch of points of uniform color

void draw_trimesh( const TriMeshVFN& mesh , const vec3e& color , float shiny ); // Draw the mesh from VFN data in uniform color, simple shading

void draw_textured_trimesh( const TriMeshVFN& mesh , float shiny , uint txtrHandle , bool smoothN = false ); // Draw the mesh from VFN data with texture

void draw_aabb( const matXe& bbox , const vec3e& color , typeF lineWidth ); // Draw the Axis-Aligned Bounding Box represented by 'bbox'

void populate_smooth_N( TriMeshVFN& mesh ); // Populate normals for the sake of smooth lighting

// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

