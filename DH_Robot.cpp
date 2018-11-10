/***********  
SOURCE_TEMPLATE.cpp
James Watson , YYYY MONTHNAME
A ONE-LINE DESRIPTION OF THE FILE

Template Version: 2018-06-07
***********/

#include "DH_Robot.h"

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

// === Classes and Structs =================================================================================================================


// == struct DH_Parameters ==

DH_Parameters copy_dh_params( const DH_Parameters& origParams ){
	// Copy DH parameters into a new structure
	DH_Parameters rtnStruct;
	rtnStruct.a     = vec_copy( origParams.a     );
	rtnStruct.d     = vec_copy( origParams.d     );
	rtnStruct.alpha = vec_copy( origParams.alpha );
	rtnStruct.theta = vec_copy( origParams.theta );
	return rtnStruct;
}

// __ End DH __


// == class RobotLink ==

RobotLink::RobotLink( float pTheta , const vec3e& pOrigin , 
					  float pD_dist , float pA_dist , const vec3e& pNextRotnAxis , float pNextRotnAngl , 
					  void (*pDrawFunc)( const DH_Parameters& DH ) ){
	// Create a robot link with an orientation, origin, and draw function

	cerr << "About to set 'RobotLink' vars ... " << endl;

	theta        = pTheta;
	// cerr << "Got the value d = " << pD_dist << endl;
	d_dist       = pD_dist;
	// cerr << "Assigned the value d = " << d_dist << endl;
	origin       = pOrigin;
	a_dist       = pA_dist;
	nextRotnAxis = pNextRotnAxis;
	nextRotnAngl = pNextRotnAngl;
	drawFunc     = pDrawFunc;
	
	cerr << "Created a RobotLink with: alpha = " << nextRotnAngl << " , a = " << a_dist << " , d = " << d_dist << " , theta = " << theta << endl;
}

void RobotLink::add_distal( RobotLink* link ){  distalLinks.push_back( link );  }

uint RobotLink::get_num_distal(){
	// Return the number of distal links attached to this link
	return distalLinks.size();
}

void RobotLink::set_theta( float pTheta ){  theta = pTheta;  } // Set the angle of the joint at the base of the link

static float AXESSCALE = 1.0;

void RobotLink::draw( const DH_Parameters& DH ){
	// Render the link and all distal links
	// 1. Save transformation for this link
	glPushMatrix(); // Transformations apply only to this link

	// 2. Transform link
	glRotated( theta , 0 , 0 , 1 ); // ------------------ 2 , end
	glTranslated( origin[0] , origin[1] , origin[2] ); // 1 , bgn 
	// 3. Render link
	drawFunc( DH );
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
	for( uint i = 0 ; i < numDistl ; i++ ){  distalLinks[i]->draw( DH );  }
	// 6. Untransform from link frame
	glPopMatrix();
}

bool RobotLink::is_leaf(){  return distalLinks.size() == 0;  }

// __ End RobotLink __


// == class UR5_OGL ==

vec3e X{1,0,0};

// ~ Con/Destuctors ~

UR5_OGL::UR5_OGL( const vec3e& baseOrigin , const DH_Parameters& pParams ){
	// 1. Instantiate the links

	basePos = baseOrigin;
	params  = copy_dh_params( pParams );

	cerr << "About to instantiate robot links!" << endl;
	uint linkNum = 1;

	Link1 = new RobotLink( 0.0 , basePos , 
				 		   params.d[1] , params.a[1] , X ,  params.alpha[1] , 
				 		   lnk1_draw );
				
	cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;

	Link2 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[2] , params.a[2] , X ,  params.alpha[2] , // 3.0*0.045 + 0.280
						   lnk2_draw );
					
	cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;

	Link3 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
	 					   params.d[3] , params.a[3] , X ,  params.alpha[3] , 
							lnk3_draw );

	cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;	
					
	Link4 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[4] , params.a[4] , X ,  params.alpha[4] , 
						   lnk4_draw );

	cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;
						
	Link5 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[5] , params.a[5] , X ,  params.alpha[5] , 
						   lnk5_draw );

	cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;
					
	Link6 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[6] , params.a[6] , X ,  params.alpha[6] , 
						   lnk6_draw );

	cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;

	cerr << "About to link robot links!" << endl;

	// 2. Set up the link structure: Connect Links
	Link1->add_distal( Link2 );  cout << "Link 1 has " << Link1->get_num_distal() << " distal links" << endl;
	Link2->add_distal( Link3 );  cout << "Link 2 has " << Link2->get_num_distal() << " distal links" << endl;
	Link3->add_distal( Link4 );  cout << "Link 3 has " << Link3->get_num_distal() << " distal links" << endl;
	Link4->add_distal( Link5 );  cout << "Link 4 has " << Link4->get_num_distal() << " distal links" << endl;
	Link5->add_distal( Link6 );  cout << "Link 5 has " << Link5->get_num_distal() << " distal links" << endl;
								 cout << "Link 6 has " << Link6->get_num_distal() << " distal links" << endl;
}

// ~ Rendering ~
void UR5_OGL::draw(){
	// Render the robot
	Link1->draw( params ); // This will draw all distal links as well
}

// __ End UR5_OGL __

// ___ End Classes _________________________________________________________________________________________________________________________


/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

