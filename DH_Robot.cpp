/***********  
SOURCE_TEMPLATE.cpp
James Watson , YYYY MONTHNAME
A ONE-LINE DESRIPTION OF THE FILE

Template Version: 2018-06-07
***********/

#include "DH_Robot.h"

// // ~~ Constants ~~
// const float BASEHEIGHT = 0.017f;
// const vec3e URGREY = { 117.0f/255.0f , 125.0f/255.0f , 130.0f/255.0f };
// const vec3e URBLCK{  50.0f/255.0f ,  50.0f/255.0f ,  50.0f/255.0f };
// const vec3e URMETL{ 224.0f/255.0f , 224.0f/255.0f , 224.0f/255.0f };
// const vec3e unitX{ 1.0f , 0.0f , 0.0f };

// === Functions ===========================================================================================================================

void lnk1_draw( const DH_Parameters& DH ){
	// Draw Link 1

	bool SHOWDEBUG = false;

	float length   = 0.140;
	float diameter = 0.120;
	float zOffset  = 0.015;
	vec3e URGREY = { 117.0f/255.0f , 125.0f/255.0f , 130.0f/255.0f };
	vec3e URBLCK{  50.0f/255.0f ,  50.0f/255.0f ,  50.0f/255.0f };
	vec3e URMETL{ 224.0f/255.0f , 224.0f/255.0f , 224.0f/255.0f };
	vec3e unitX{ 1.0f , 0.0f , 0.0f };

	glPushMatrix(); // Transformations apply only to this link
	draw_cylinder( vec3e{ 0 , 0 , ( DH.d[1] - length/2.0f ) } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	
	glTranslated( 0 , 0 , length/2.0 + zOffset ); 
	glRotated( 90 , 1 , 0 , 0 ); 
	draw_cylinder( vec3e{ 0,0,0 } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , 0.0f );
	glPopMatrix();

	if( SHOWDEBUG ){ 
		cerr << "Paint link 1 , DH populated?: " << yesno( dh_populated( DH ) ) << endl; 
		cerr << "OGL ERROR LINK 1?:" << yesno( ErrCheck( "lnk1_draw" ) ) << endl;
	}
}

void lnk2_draw( const DH_Parameters& DH ){
	// Draw Link 2

	bool SHOWDEBUG = false;

	float length   = 0.140;
	float diameter = 0.120;
	float armDia   = 0.085;
	vec3e URGREY = { 117.0f/255.0f , 125.0f/255.0f , 130.0f/255.0f };
	vec3e URBLCK{  50.0f/255.0f ,  50.0f/255.0f ,  50.0f/255.0f };
	vec3e URMETL{ 224.0f/255.0f , 224.0f/255.0f , 224.0f/255.0f };
	vec3e unitX{ 1.0f , 0.0f , 0.0f };

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

	if( SHOWDEBUG ){ 
		cerr << "Paint link 2 , DH populated?: " << yesno( dh_populated( DH ) ) << endl; 
		cerr << "OGL ERROR LINK 2?:" << yesno( ErrCheck( "lnk2_draw" ) ) << endl;
	}
}

void lnk3_draw( const DH_Parameters& DH ){
	// Draw Link 3

	bool SHOWDEBUG = false;

	float length   = 0.090;
	//~ float bigLen   = 0.140;
	float diameter = 0.072;
	float armDia   = diameter;
	vec3e URGREY = { 117.0f/255.0f , 125.0f/255.0f , 130.0f/255.0f };
	vec3e URBLCK{  50.0f/255.0f ,  50.0f/255.0f ,  50.0f/255.0f };
	vec3e URMETL{ 224.0f/255.0f , 224.0f/255.0f , 224.0f/255.0f };
	vec3e unitX{ 1.0f , 0.0f , 0.0f };

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

	if( SHOWDEBUG ){ 
		cerr << "Paint link 3 , DH populated?: " << yesno( dh_populated( DH ) ) << endl; 
		cerr << "OGL ERROR LINK 3?:" << yesno( ErrCheck( "lnk3_draw" ) ) << endl;
	}
}

void lnk4_draw( const DH_Parameters& DH ){
	// Draw Link 4

	bool SHOWDEBUG = false;

	float length   = 0.090;
	float diameter = 0.072;
	vec3e URGREY = { 117.0f/255.0f , 125.0f/255.0f , 130.0f/255.0f };
	vec3e URBLCK{  50.0f/255.0f ,  50.0f/255.0f ,  50.0f/255.0f };
	vec3e URMETL{ 224.0f/255.0f , 224.0f/255.0f , 224.0f/255.0f };
	vec3e unitX{ 1.0f , 0.0f , 0.0f };

	
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

	if( SHOWDEBUG ){ 
		cerr << "Paint link 4 , DH populated?: " << yesno( dh_populated( DH ) ) << endl; 
		cerr << "OGL ERROR LINK 4?:" << yesno( ErrCheck( "lnk4_draw" ) ) << endl;
	}
}

void lnk5_draw( const DH_Parameters& DH ){
	// Draw Link 5

	bool SHOWDEBUG = false;

	float length   = 0.090;
	float diameter = 0.072;
	vec3e URGREY = { 117.0f/255.0f , 125.0f/255.0f , 130.0f/255.0f };
	vec3e URBLCK{  50.0f/255.0f ,  50.0f/255.0f ,  50.0f/255.0f };
	vec3e URMETL{ 224.0f/255.0f , 224.0f/255.0f , 224.0f/255.0f };
	vec3e unitX{ 1.0f , 0.0f , 0.0f };

	
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

	if( SHOWDEBUG ){ 
		cerr << "Paint link 5 , DH populated?: " << yesno( dh_populated( DH ) ) << endl; 
		cerr << "OGL ERROR LINK 5?:" << yesno( ErrCheck( "lnk5_draw" ) ) << endl;
	}
}

void lnk6_draw( const DH_Parameters& DH ){
	// Draw Link 6

	bool SHOWDEBUG = false;

	float length   = 0.090;
	float diameter = 0.072;
	// vec3e URGREY = { 117.0f/255.0f , 125.0f/255.0f , 130.0f/255.0f };
	vec3e URBLCK{  50.0f/255.0f ,  50.0f/255.0f ,  50.0f/255.0f };
	vec3e URMETL{ 224.0f/255.0f , 224.0f/255.0f , 224.0f/255.0f };
	vec3e unitX{ 1.0f , 0.0f , 0.0f };

	glPushMatrix(); // Transformations apply only to this link
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3e{ 0,0, 0.00 } , DH.d[6]-length/2.0f  , diameter/2.0f , NUMCYLFACETS , 
				   URMETL , 0.0f );
	glPopMatrix();

	if( SHOWDEBUG ){ 
		cerr << "Paint link 6 , DH populated?: " << yesno( dh_populated( DH ) ) << endl;
		cerr << "OGL ERROR LINK 6?:" << yesno( ErrCheck( "lnk6_draw" ) ) << endl;
	}
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

bool dh_populated( const DH_Parameters& origParams ){
	// Are there params and are they of equal length?
	if( !( origParams.a.size() && origParams.d.size() && origParams.alpha.size() && origParams.theta.size() ) )
		return false;
	if( !( origParams.a.size() == origParams.d.size() && 
		   origParams.d.size() == origParams.alpha.size() &&
		   origParams.alpha.size() == origParams.theta.size() ) )
		return false;
	return true;
}

// __ End DH __


// == class RobotLink ==

RobotLink::RobotLink( float pTheta , const vec3e& pOrigin , 
					  float pD_dist , float pA_dist , const vec3e& pNextRotnAxis , float pNextRotnAngl , 
					  void (*pDrawFunc)( const DH_Parameters& DH ) ){
	// Create a robot link with an orientation, origin, and draw function

    bool SHOWDEBUG = false;

	if( SHOWDEBUG ) cerr << "About to set 'RobotLink' vars ... " << endl;

	theta        = pTheta;
	// cerr << "Got the value d = " << pD_dist << endl;
	d_dist       = pD_dist;
	// cerr << "Assigned the value d = " << d_dist << endl;
	origin       = pOrigin;
	a_dist       = pA_dist;

	if( SHOWDEBUG ) cerr << "Got the value axis = ____ " << pNextRotnAxis << endl;
	nextRotnAxis = pNextRotnAxis;
	if( SHOWDEBUG ) cerr << "Assigned the value axis = " << nextRotnAxis  << endl;
	
	nextRotnAngl = pNextRotnAngl;
	drawFunc     = pDrawFunc;
	
    if( SHOWDEBUG ) 
        cerr << "Created a RobotLink with: alpha = " << nextRotnAngl 
            << " , a = " << a_dist 
            << " , d = " << d_dist 
            << " , theta = " << theta
            << " , alpha axis = " << nextRotnAxis << endl;
}

RobotLink::~RobotLink(){  clearif( distalLinks );  }

void RobotLink::add_distal( RobotLink* link ){  distalLinks.push_back( link );  }

uint RobotLink::get_num_distal(){
	// Return the number of distal links attached to this link
	return distalLinks.size();
}

void RobotLink::set_theta( float pTheta ){  theta = fmod( pTheta , 360.0f );  } // Set the angle of the joint at the base of the link

static float AXESSCALE = 0.5;

bool RobotLink::has_draw_func(){  return drawFunc;  }

void RobotLink::draw( const DH_Parameters& DH ){
	// Render the link and all distal links
	// 1. Save transformation for this link
	glPushMatrix(); // Transformations apply only to this link

	// 2. Transform link
	glRotated( theta , 0 , 0 , 1 ); // ------------------ 2 , end
	
	// glTranslated( origin[0] , origin[1] , origin[2] ); // 1 , bgn 
	glTranslated( origin(0) , origin(1) , origin(2) ); // 1 , bgn 
	
	// 3. Render link
	if( enableLinkDraw ) drawFunc( DH );
	// 4. Downstream link transform
	
	glTranslated( a_dist , 0 , 0 ); // ------------------------- 1 , bgn 
	glTranslated( 0 , 0 , d_dist ); // ------------------------- 1 , bgn 

	glRotated( nextRotnAngl , // ------------------------------------ 2 , end
			//    nextRotnAxis[0] , nextRotnAxis[1] , nextRotnAxis[2] );
			   nextRotnAxis(0) , nextRotnAxis(1) , nextRotnAxis(2) );
	
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

// ~ Con/Destuctors ~

UR5_OGL::UR5_OGL( const vec3e& baseOrigin , const DH_Parameters& pParams ){
	// 1. Instantiate the links

	bool SHOWDEBUG = false;

	basePos = baseOrigin;
	params  = copy_dh_params( pParams );


	vec3e unitX{ 1.0f , 0.0f , 0.0f };


	uint linkNum = 1;

	Link1 = new RobotLink( 0.0 , basePos , 
				 		   params.d[1] , params.a[1] , unitX ,  params.alpha[1] , 
				 		   lnk1_draw );
				
	if( SHOWDEBUG ) cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;

	Link2 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[2] , params.a[2] , unitX ,  params.alpha[2] , // 3.0*0.045 + 0.280
						   lnk2_draw );
					
	if( SHOWDEBUG ) cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;

	Link3 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
	 					   params.d[3] , params.a[3] , unitX ,  params.alpha[3] , 
						   lnk3_draw );

	if( SHOWDEBUG ) cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;	
					
	Link4 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[4] , params.a[4] , unitX ,  params.alpha[4] , 
						   lnk4_draw );

	if( SHOWDEBUG ) cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;
						
	Link5 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[5] , params.a[5] , unitX ,  params.alpha[5] , 
						   lnk5_draw );

	if( SHOWDEBUG ) cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;
					
	Link6 = new RobotLink( 0.0 , vec3e{ 0 , 0 , 0.000 } , 
						   params.d[6] , params.a[6] , unitX ,  params.alpha[6] , 
						   lnk6_draw );

	if( SHOWDEBUG ) cerr << "Link " << linkNum++ << " instantiated!" << endl << endl;

	if( SHOWDEBUG ) cerr << "About to link robot links!" << endl;

	// 2. Set up the link structure: Connect Links
	Link1->add_distal( Link2 );  
	Link2->add_distal( Link3 );  
	Link3->add_distal( Link4 );  
	Link4->add_distal( Link5 );  
	Link5->add_distal( Link6 );  
								 
	if( SHOWDEBUG ){
		cout << "Link 1 has " << Link1->get_num_distal() 
			 << " distal links. Has draw function?: " << yesno( Link1->has_draw_func() ) << endl;
		cout << "Link 2 has " << Link2->get_num_distal() 
			 << " distal links. Has draw function?: " << yesno( Link2->has_draw_func() ) << endl;
		cout << "Link 3 has " << Link3->get_num_distal() 
			 << " distal links. Has draw function?: " << yesno( Link3->has_draw_func() ) << endl;
		cout << "Link 4 has " << Link4->get_num_distal() 
			 << " distal links. Has draw function?: " << yesno( Link4->has_draw_func() ) << endl;
		cout << "Link 5 has " << Link5->get_num_distal() 
			 << " distal links. Has draw function?: " << yesno( Link5->has_draw_func() ) << endl;
		cout << "Link 6 has " << Link6->get_num_distal() 
			 << " distal links. Has draw function?: " << yesno( Link6->has_draw_func() ) << endl;
	}
}

UR5_OGL::~UR5_OGL(){
    // Delete all links
    delif( Link1 );
    delif( Link2 );
    delif( Link3 );
    delif( Link4 );
    delif( Link5 );
    delif( Link6 );
}

// ~ Rendering ~
void UR5_OGL::draw(){
	// Render the robot
	// Link1->draw( params ); // This will draw all distal links as well
	Link1->draw( params ); // This will draw all distal links as well
}

// ~ Joint State ~
void UR5_OGL::set_joint_state( const std::vector<float>& qNu ){
	// Check that the new joint state has the correct length and if so set

	bool SHOWDEBUG = false;

	uint qLen = qNu.size();
	if( qLen == 6 ){
		q = vec_copy( qNu );
		if( SHOWDEBUG ) cout << "Joints set to " << q << endl;
		// 3. Update joints
		Link1->set_theta( q[0] );
		Link2->set_theta( q[1] );
		Link3->set_theta( q[2] );
		Link4->set_theta( q[3] );
		Link5->set_theta( q[4] );
		Link6->set_theta( q[5] );
	}else{
		cout << "UR5_OGL::set_joint_state , Got a q vector of size " << qLen << " but expected 6" << endl;
	}
}

// __ End UR5_OGL __

// ___ End Classes _________________________________________________________________________________________________________________________


/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

