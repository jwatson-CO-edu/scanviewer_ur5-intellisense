/***********  
SOURCE_TEMPLATE.cpp
James Watson , YYYY MONTHNAME
A ONE-LINE DESRIPTION OF THE FILE

Template Version: 2018-06-07
***********/

#include "DH_Robot.h"

// === Classes and Structs =================================================================================================================

// == class RobotLink ==

RobotLink::RobotLink( float pTheta , const vec3e& pOrigin , 
					  float pD_dist , float pA_dist , const vec3e& pNextRotnAxis , float pNextRotnAngl , 
					  void (*pDrawFunc)(const DH_Parameters& DH) ){
	// Create a robot link with an orientation, origin, and draw function
	theta        = pTheta;
	origin       = pOrigin;
	d_dist       = pD_dist;
	a_dist       = pA_dist;
	nextRotnAxis = pNextRotnAxis;
	nextRotnAngl = pNextRotnAngl;
	drawFunc     = pDrawFunc;
	
	cout << "Created a RobotLink with: alpha = " << nextRotnAngl << " , a = " << a_dist << " , d = " << d_dist << " , theta = " << theta << endl;
}

void RobotLink::add_distal( RobotLink* link ){  distalLinks.push_back( link );  }

void RobotLink::set_theta( float pTheta ){  theta = pTheta;  } // Set the angle of the joint at the base of the link

static float AXESSCALE = 0.17;

void RobotLink::draw(){
	// Render the link and all distal links
	// 1. Save transformation for this link
	glPushMatrix(); // Transformations apply only to this link
	
	
	
	// 2. Transform link
	glRotated( theta , 0 , 0 , 1 ); // ------------------ 2 , end
	glTranslated( origin[0] , origin[1] , origin[2] ); // 1 , bgn 
	// 3. Render link
	drawFunc();
	// 4. Downstream link transform
	
	glTranslated( a_dist , 0 , 0 ); // ------------------------- 1 , bgn 
	glTranslated( 0 , 0 , d_dist ); // ------------------------- 1 , bgn 

	glRotated( nextRotnAngl , // ------------------------------------ 2 , end
			   nextRotnAxis[0] , nextRotnAxis[1] , nextRotnAxis[2] );
	
	draw_origin( AXESSCALE );
	
	
	// Capture the transform if we are at the final link
	// if( is_leaf() ){  
	// 	glGetFloatv( GL_MODELVIEW_MATRIX , OGLmat );  
	// 	for( uint i = 0 ; i < 16 ; i++ ){  modelMat[i] = OGLmat[i];  }
	// }
	
	
	// 4. For each distal link
	uint numDistl = distalLinks.size();
	// 5. Draw the link *relative* to this link!
	for( uint i = 0 ; i < numDistl ; i++ ){  distalLinks[i]->draw();  }
	// 6. Untransform from link frame
	glPopMatrix();
}

bool RobotLink::is_leaf(){  return distalLinks.size() == 0;  }

// __ End RobotLink __


// == class UR5_OGL ==

// = Robot Parameters =
// Joint:                           0   ,  1      ,  2     ,  3     ,  4      ,   5      ,  6
static std::vector<float> alpha = { 0.0 , 90.0    ,  0.0   ,  0.0   , 90.0    , -90.0    ,  0.0    }; 
static std::vector<float> a     = { 0.0 ,  0.0    , -0.425 , -0.392 ,  0.0    ,   0.0    ,  0.0    }; 
static std::vector<float> d     = { 0.0 ,  0.0892 ,  0.0   ,  0.0   ,  0.1093 ,   0.0948 ,  0.0825 }; 
static std::vector<float> theta = { 0.0 ,  0.0    ,  0.0   ,  0.0   ,  0.0    ,   0.0    ,  0.0    }; 
static DH_Parameters DH{ alpha , a , d , theta };
static float BASEHEIGHT = 0.017;
static vec3e X{1,0,0};
// _ End Params _

UR5_OGL::UR5_OGL( const vec3e& baseOrigin ){
	// 1. Instantiate the links
	Link1 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
				 		   DH.d[1] , DH.a[1] , X ,  DH.alpha[1] , 
				 		   lnk1_draw );
				

	Link2 = new RobotLink{ 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   DH.d[2] , DH.a[2] , X ,  DH.alpha[2] , // 3.0*0.045 + 0.280
						   lnk2_draw };
					
	Link3 = new RobotLink{ 0.0 , vec3e{ 0 , 0 , 0.000 } , 
	 					   DH.d[3] , DH.a[3] , X ,  DH.alpha[3] , 
							lnk3_draw };
					
	Link4 = new RobotLink{ 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   DH.d[4] , DH.a[4] , X ,  DH.alpha[4] , 
						   lnk4_draw };
						
	Link5 = new RobotLink{ 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   DH.d[5] , DH.a[5] , X ,  DH.alpha[5] , 
						   lnk5_draw };
					
	Link6 = new RobotLink{ 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   DH.d[6] , DH.a[6] , X ,  DH.alpha[6] , 
						   lnk6_draw };

	// 2. Set up the link structure

	// FIXME : SET UP THE SERIAL LINKS , NESTED STRUCTURE
}

// __ End UR5_OGL __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

void lnk1_draw( const DH_Parameters& DH ){
	// Draw Link 1
	float length   = 0.140;
	float diameter = 0.120;
	float zOffset  = 0.015;
	glPushMatrix(); // Transformations apply only to this link
	draw_cylinder( vec3e{ 0 , 0 , ( DH.d[1] - length/2.0f ) } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	
	glTranslated( 0 , 0 , length/2.0 + zOffset ); 
	glRotated( 90 , 1 , 0 , 0 ); 
	draw_cylinder( vec3e{ 0,0,0 } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	glPopMatrix();
}

void lnk2_draw( const DH_Parameters& DH ){
	// Draw Link 2
	float length   = 0.140;
	float diameter = 0.120;
	float armDia   = 0.085;
	glPushMatrix(); // Transformations apply only to this link
	// 1. Vertical cylinder
	glRotated( -90 , 0 , 0 , 1 ); 
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3e{ 0,0,0 } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	// 2. Horizontal Cylinders
	
	
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , length/2.0 , 0 ); 
	
	draw_cylinder( vec3e{ 0,0,0 } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	draw_cylinder( vec3e{ 0,0,diameter/2.0f } , 0.295 , armDia/2.0f , NUMCYLFACETS , 
				   URMETL , 0.0f );
	draw_cylinder( vec3e{ 0,0,-DH.a[2]-length/2.0f } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	
	glRotated( 90 , 1 , 0 , 0 ); 
	draw_cylinder( vec3e{ 0,-DH.a[2],-length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	glPopMatrix();
}

void lnk3_draw( const DH_Parameters& DH ){
	// Draw Link 3
	float length   = 0.090;
	//~ float bigLen   = 0.140;
	float diameter = 0.072;
	float armDia   = diameter;
	glPushMatrix(); // Transformations apply only to this link
	// 1. Vertical cylinder
	glRotated( -90 , 0 , 0 , 1 ); 
	glTranslated( 0 , 0 , - length * 0.5f ); 
	
	float xtraHght = 0.020;
	float zAdjust  = 0.002;
	// - bigLen/2.0f - length/2.0f - length
	draw_cylinder( vec3e{ 0,0, DH.d[3] + zAdjust } , length + xtraHght , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	// 2. Horizontal Cylinders
	
	
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , length/2.0f , 0.0 ); 
	
	draw_cylinder( vec3e{ 0,0,0 } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	draw_cylinder( vec3e{ 0,0,0.045 } , -DH.a[3]-length , armDia/2.0f , NUMCYLFACETS , 
				   URMETL , 0.0f );
	draw_cylinder( vec3e{ 0,0,-DH.a[3]-length/2.0f } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	glRotated( 90 , 1 , 0 , 0 ); 
	draw_cylinder( vec3e{ 0 , -DH.a[3] , -length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	glPopMatrix();
}

void lnk4_draw( const DH_Parameters& DH ){
	// Draw Link 4
	float length   = 0.090;
	float diameter = 0.072;
	
	glPushMatrix(); // Transformations apply only to this link
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3e{ 0,0, 0.00 } , DH.d[4]-length/2.0f  , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
				   
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , DH.d[4]-length/2.0f , 0.0 ); 
	// - ( DH.d[3] -length/2.0f )
	// DH.d[3] + length/2.0f + ( DH.d[3] -length/2.0f )
	
	draw_cylinder( vec3e{ 0,0, -length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	
	glPopMatrix();
}

void lnk5_draw( const DH_Parameters& DH ){
	// Draw Link 5
	float length   = 0.090;
	float diameter = 0.072;
	
	glPushMatrix(); // Transformations apply only to this link
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3e{ 0,0, 0.00 } , DH.d[5]-length/2.0f  , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
				   
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , DH.d[5]-length/2.0f , 0.0 ); 
	// - ( DH.d[3] -length/2.0f )
	// DH.d[3] + length/2.0f + ( DH.d[3] -length/2.0f )
	
	draw_cylinder( vec3e{ 0,0, -length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	
	glPopMatrix();
}

void lnk6_draw( const DH_Parameters& DH ){
	// Draw Link 6
	float length   = 0.090;
	float diameter = 0.072;
	glPushMatrix(); // Transformations apply only to this link
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3e{ 0,0, 0.00 } , DH.d[6]-length/2.0f  , diameter/2.0f , NUMCYLFACETS , 
				   URMETL , 0.0f );
	glPopMatrix();
}

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

