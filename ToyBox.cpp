/***********  
ToyBox.cpp
James Watson , 2018 October
Objects and Critters to render

Template Version: 2018-06-07
***********/

#include "ToyBox.h"

// === Classes and Structs =================================================================================================================

// == class PegBlock ==


PegBlock::PegBlock( float minSide , float maxSide , const matXe& bbox ){
	// Generate and instantiate a cube with pegs sticking out of it
	// 1. Choose a side length
	side = randrange( minSide , maxSide );
	// 2. Choose a center in the bbox
	center = sample_from_AABB( bbox );
	// 3. Choose theta and phi
	th = 360 * random();
	ph = 180 * random();
	// 4. Choose peg lengths
	pegLen = randrange_vec( side * 0.1f , side , (size_t)4 * 6 );
	// 5. Choose colors
	fillColor = vec3e_random();
	lineColor = vec3e{ 0,0,0 };
	
	cout << "Created a PegBlock! With fill color " << fillColor << " and line color " << lineColor << endl;
}

void PegBlock::draw(){
	// Draw pegblock
	glPushMatrix();
	// 1. Rotate into theta and phi
	glRotated( ph , 1 , 0 , 0 ); // 2. Rotate around the X axis
	glRotated( th , 0 , 0 , 1 ); // 1. Rotate around the Z axis
	glTranslated( center[0] , center[1] , center[2] );
	// 2. Draw cube
	float fill[3];	//fillColor.load_array( fill );
	float line[3];	//lineColor.load_array( line );
	//~ uint pegDex = 0;
	cube( 0 , 0 , 0 ,
          side , side , side ,
          fill , line );
	// 3. For each cube face
		// 4. Rotate into face
		
		//~ // 5. Draw 4 pegs
		//~ draw_cylinder( vec3e{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ // 6. Pop
		
		//~ glRotated( 180 , 1 , 0 , 0 ); // 2. Rotate around the X axis
		//~ // 5. Draw 4 pegs
		//~ draw_cylinder( vec3e{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
					   
		//~ glRotated( -90 , 1 , 0 , 0 ); // 2. Rotate around the X axis
		//~ // 5. Draw 4 pegs
		//~ draw_cylinder( vec3e{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
		//~ draw_cylinder( vec3e{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   //~ fillColor , lineColor );  pegDex++;
					   
		//~ for( uint i = 0 ; i < 3 ; i++ ){
			//~ glRotated( -90 , 0 , 1 , 0 ); // 2. Rotate around the X axis
			//~ // 5. Draw 4 pegs
			//~ draw_cylinder( vec3f{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   //~ fillColor , lineColor );  pegDex++;
			//~ draw_cylinder( vec3f{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   //~ fillColor , lineColor );  pegDex++;
			//~ draw_cylinder( vec3f{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   //~ fillColor , lineColor );  pegDex++;
			//~ draw_cylinder( vec3f{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   //~ fillColor , lineColor );  pegDex++;
		//~ }
		
	
	// 7. Pop
	glPopMatrix();
}

// __ End PegBlock __


// == class Icosahedron_OGL ==

// ~ Con/Destructors ~

Icosahedron_OGL::Icosahedron_OGL( float rad , const vec3e& cntr , const vec3e& colr , float shiny ){
	center    = cntr;
	theta     = 0.0;
	phi       = 0.0;
	icosGeo   = Icosahedron_e( rad , vec3e{0,0,0} );
	color     = colr;
	shininess = shiny;
}

// ~ Setters ~

void Icosahedron_OGL::set_th_ph( float thNu , float phNu ){
	// Set theta and phi to 'thNu' and 'phNu'
	theta = thNu;
	phi   = phNu;
}

void Icosahedron_OGL::turn_th_ph( float thIncr , float phIncr ){
	// Increment theta and phi by 'thIncr' and 'phIncr'
	theta += thIncr;
	phi   += phIncr;
}

void Icosahedron_OGL::set_emission_color( const vec3e& emitClr ){  
	emitColor = emitClr;  
	emitArray[3] = 1.0f;
	
}

void Icosahedron_OGL::set_emission_intensity( float intnsty ){  
	intensity = intnsty;  
	for( uint i = 0 ; i < 3 ; i ++ ){  emitArray[i] = emitColor[i] * ( intensity / 100.0f );  }
}

// ~ Textures ~
void Icosahedron_OGL::assign_face_textures_randomly( uint txtrHandle , float patchRad , uint xDim , uint yDim ){
	// Pick a patch from a (big) texture and assign it to each face
	// 1. Designate this icos as having texture
	hasTextr      = true;
	textureHandle = txtrHandle;
	txtrVerts     = matXe::Zero( 20 * 3 , 2 );
	vec2e center{0,0};
	vec2e txtrVert;
	matXe triVerts;
	uint j = 0;
	float xDimF = (float)xDim;
	float yDimF = (float)yDim;
	// 2. Choose corresponding texture vertices for all of the faces , For each face
	matXe box = matXe::Zero( 2 , 2 );
	box << patchRad               , patchRad               , 
		   (float)xDim - patchRad , (float)yDim - patchRad ;
	for( uint i = 0 ; i < 20 ; i++ ){
		// 3. Choose a center of sufficient distance from the texture edge
		center = sample_from_box( box );
		// 4. Generate vertices
		triVerts = equilateral_tri_vertices( center , patchRad );
		// 5. Load vertices
		for( j = 0 ; j < 3 ; j++ ){
			txtrVert = vec2e( triVerts(j,0) / xDimF , triVerts(j,1) / yDimF );
			txtrVerts.row( i * 3 + j ) = txtrVert;
		}
	}
}

// ~ Rendering ~

void Icosahedron_OGL::draw( float shiny ){
	// Render the icosahedron
	vec3e currNorm;
	vec3e currVert;
	vec2e txtrVert;
	if( shiny == 0 ) shiny = shininess;
	// 0. Set material props	
	glMaterialf(  GL_FRONT , GL_SHININESS , shiny      ); // Set the material shininess
	glMaterialfv( GL_FRONT , GL_SPECULAR  , MATL_WHITE ); // Set the color of the specular reflection
	glMaterialfv( GL_FRONT , GL_EMISSION  , emitArray  );
	// 1. Push
	glPushMatrix();
	// 2. Apply rotations and set center
	glRotated( phi   , 1 , 0 , 0 ); // 2. Rotate around the X axis
	glRotated( theta , 0 , 0 , 1 ); // 1. Rotate around the Z axis
	glTranslated( center[0] , center[1] , center[2] );
	// 3.5. Set texture if exists
	if( hasTextr ){    
		glColor4f( 1,1,1,1 ); // Set all color channels to full
		glEnable( GL_TEXTURE_2D );
		glBindTexture( GL_TEXTURE_2D , textureHandle );
	}else{
		// 3. Set color
		glColor( color );
	}
	// 4. For each face
	glBegin( GL_TRIANGLES );
	for( uint i = 0 ; i < numFaces ; i++ ){
		// 5. Set the lighting normal for the face
		currNorm = icosGeo.N.row( i );
		//~ glNormal3f( currNorm );
		glNormal( currNorm );
		// 6. Start a triangle
		for( uint j = 0 ; j < 3 ; j++ ){
			// 7. Lay out each of the vertices
			if( hasTextr ){
				txtrVert = txtrVerts.row( i * 3 + j );
				glTxr2e( txtrVert );
			}
			currVert = icosGeo.V.row( icosGeo.F(i,j) );
			//~ glVertex3f( currVert );
			glVertex( currVert );
		}
	}
	glEnd();
	// N. Pop
	glPopMatrix();
	glDisable( GL_TEXTURE_2D );
}

// __ End Icosahedron_OGL __


// == class RibbonBolt ==


// ~ Con/Destructors ~

RibbonBolt::RibbonBolt( const vec3e& orgn , 
						const vec3e& clr , float intns ,
						float lenTravelMin , float lenTravelMax , float travelSpeed ,
						float width , float lengthMin , float lengthMax ){
	// Set (initial) params for a beam
	
	origin   = orgn; // ------- Beam starts at this point
	bColor   = clr; // -------- Beam color
	intens   = intns; // ------ Surface emission intensity
	travlMin = lenTravelMin; // Beam disappears at this distance
	travlMax = lenTravelMax; 
	bSpeed   = travelSpeed; //- Speed in [units/s]
	wdBeam   = width; // ------ Beam width
	lenMin   = lengthMin; // -- Beam length
	lenMax   = lengthMax; 
	
	sampleBox = matXe::Zero( 2 , 3 );
	sampleBox << -1 , -1 , -1 ,
				  1 ,  1 ,  1 ;
}
			

// ~ Animation ~
void RibbonBolt::advance( float time_in_sec ){
	// If active, Advance the animation by 'time_in_sec' and check for a reset
	if( active ){
		progrs += bSpeed * time_in_sec;
		if( progrs >= curLen + curTravl ){  _reset_random();  }
	} // else inactive , No action
}

void RibbonBolt::_reset_random(){
	// Point the beam in a random direction and set params for a new emission
	progrs   = 0.0f; // Reset progress
	curTravl = randrange( travlMin , travlMax ); // Set the current beam travel
	curLen   = randrange( lenMin   , lenMax   ); // Set the current beam length
	dirctn   = sample_from_AABB( sampleBox );  // This will not be uniform in directions, but it will be close enough
	// NOTE: This function assumes that 'sample_from_AABB' will not return a zero vector
	dirctn.normalize(); // Direction should be normalized
	crFlat   = get_any_perpendicular( dirctn ); // Pick a random flat direction perpendicular to the emission direction
}

void RibbonBolt::set_emission_intensity( float intnsty ){
	// Load the array that controls surface glow
	intens = intnsty;
	for( uint i = 0 ; i < 3 ; i++ ){  emColor[i] = bColor[i] * ( intens / 100.0f );  }
}

void RibbonBolt::activate(){
	// Ensure that bolt is active and ready to animate
	if( !active ){
		active = true;
		_reset_random();
	} // else is already active , allow anim to continue
}

void RibbonBolt::deactivate(){
	// Shut down animation and hide
	active = false;
}

void RibbonBolt::draw(){
	// If the bolt is active, draw a glowing quad radiating from 'origin'
	if( active ){
		// Render the bolt as a quad
		vec3e frntMid , frntNeg , frntPos , 
			  backMid , backNeg , backPos ;
		// 1. Find the front of the bolt
		float front = min( curTravl , progrs );
		// 2. Get the front corners
		frntMid = origin + dirctn * front;
		frntNeg = frntMid + crFlat * ( -wdBeam / 2.0f );
		frntPos = frntMid + crFlat * (  wdBeam / 2.0f );
		// 3. Find the back of the bolt
		float back  = max( 0.0f , progrs - curLen );
		// 4. Get the back corners
		backMid = origin + dirctn * back;
		backNeg = backMid + crFlat * ( -wdBeam / 2.0f );
		backPos = backMid + crFlat * (  wdBeam / 2.0f );
		// 5. Set the emission
		glColor( bColor );
		glMaterialfv( GL_FRONT , GL_EMISSION , emColor );
		// 6. Draw quad
		glBegin( GL_QUADS );
			glVec3e( frntNeg );
			glVec3e( frntPos );
			glVec3e( backPos );
			glVec3e( backNeg );
		glEnd();
	}
}

// __ End RibbonBolt __


// == class PatchMesh == 

matXe matx_from_lines( const stdvec<string>& lines , char separator , size_t numCols ){
    // Build a matrix from a vector of lines of text assumed to contain floats
    // NOTE: This function assumes that the
    size_t len = lines.size();
    matXe rtnMatx = matXe::Zero( len , numCols );
    stdvec<float> tokens; 
    for( size_t i = 0 ; i < len ; i++ ){
        tokens = tokenize_to_float_w_separator( lines[i] , separator );
        for( size_t j = 0 ; j < numCols ; j++ ){
            rtnMatx(i,j) = tokens[j];
        }
    }
    return rtnMatx;
}

TriMeshVFN V_to_mesh_in_cam_frame( const matXe& V , 
                                   const vec3e& origin , 
                                   const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis ){
	// Express the points in the camera frame so that it can be meshed properly
	// 1. Transform points
	matXe Vxfrm = V_in_child_frame( V , origin , xBasis , yBasis , zBasis );
	
    // 2. Mesh points with Delaunay
	return delaunay_from_V( Vxfrm );
    // return delaunay_from_V( V );
}

matXe compute_UV_for_trimesh( const matXe& Vcamera , typeF horz_FOVdeg , typeF vert_FOVdeg ){
    // For each of the vertices in 'Vcamera', compute the UV position within [0,1] and return correspinding UV
    // NOTE: This function assumes that 'Vcamera' is in the camera frame
    // NOTE: There is a row-row correspondence between 'Vmesh' and the return matrix
    // NOTE: U:X as V:Y, from the bottom-left , https://stackoverflow.com/a/5532670
    
    bool SHOWDEBUG = true;

    // A. The center of the shot is ( 0.5 , 0.5 )
    size_t len     = Vcamera.rows();
    matXe  rtnMatx = matXe::Zero( len , 2 );
    typeF  H_horz  = tan( radians( horz_FOVdeg / 2.0f ) );
    typeF  H_vert  = tan( radians( vert_FOVdeg / 2.0f ) );
    vec3e  p_i;

    // 1. For each of the original points in the mesh
    for( size_t i = 0 ; i < len ; i++ ){
        p_i = Vcamera.row(i);
        // 2. Compute U
        rtnMatx(i,0) = ( - p_i(0) / p_i(2) ) / ( 2 * H_horz ) + 0.5f;
        // rtnMatx(i,0) = ( p_i(0) / p_i(2) ) / ( H_horz ) + 0.5f;;
        // 3. Compute V
        rtnMatx(i,1) = (   p_i(1) / p_i(2) ) / ( 2 * H_vert ) + 0.5f;
        // rtnMatx(i,1) = ( p_i(1) / p_i(2) ) / ( H_vert ) + 0.5f;;
    }

    if( SHOWDEBUG )  cerr << "Found UV in the following ranges:" << endl << AABB( rtnMatx ) << endl;

    return rtnMatx;
}

PatchMesh::PatchMesh( string fPath , bool useTxtr ){
    // Create a Patchmesh from a file
    // NOTE: This function assumes that 'fPath' has at least one "OBJ" defined

    bool SHOWDEBUG = true; // if( SHOWDEBUG ){  cout <<  << endl;  }

    hasTextr = useTxtr;

    // 0. Get lines
    stdvec<string> lines /* - */ = readlines( fPath );
    size_t /* - */ numLines      = lines.size() , 
                   firstMeshLine = 6;

    //  1. Read the texture path & load texture
    txtrPath = lines[0];
    if( SHOWDEBUG ){  cout << "Texture File: " << txtrPath << endl;  }

    //  2. Read the camera origin
    camOrigin = str_to_vec3( lines[1] , ',' );
    if( SHOWDEBUG ){  cout << "Camera Origin: " << camOrigin << endl;  }

    //  3. Read the camera basis vectors
    cam_xBasis = str_to_vec3( lines[2] , ',' ).normalized();
    cam_yBasis = str_to_vec3( lines[3] , ',' ).normalized();
    cam_zBasis = str_to_vec3( lines[4] , ',' ).normalized();
    if( SHOWDEBUG ){  
        cout << "Camera Basis Vectors: " << cam_xBasis << " , " 
                                         << cam_yBasis << " , "                                                        
                                         << cam_zBasis << endl
             << "Bases Orthonormal?: " << yesno( check_bases_orthonormal( cam_xBasis , cam_yBasis , cam_zBasis ) ) << endl;
    }

    // 4. Read the robot joints
    joint_state = tokenize_to_float_w_separator( lines[5] , ',' );

    string /* - */ currLine;
    stdvec<string> patchLines;
    matXe /* -- */ patchVerts;
    TriMeshVFN     camPatch;

    // ~~~~~ LAMBDA: Process all the vertices currently held in 'patchLines' ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    auto process_lines = [&](){
        patchVerts = matx_from_lines( patchLines , ',' , 3 );
        camPatch   = V_to_mesh_in_cam_frame( patchVerts , camOrigin , cam_xBasis , cam_yBasis , cam_zBasis );
        //  A. If texture is used , then Get UV patches for each of the mesh triangles
        if( hasTextr ){  
            camPatch.UV = compute_UV_for_trimesh( camPatch.V , FOVHORZDEG , FOVVERTDEG );  
            if( SHOWDEBUG ){  cout << "Computed UV!" << endl
                                   << "Num UV Coords: " << camPatch.UV.rows() << endl;  }
        }

        //  B. Transform back into lab frame
        camPatch.V = V_in_parent_frame( camPatch.V , camOrigin , cam_xBasis , cam_yBasis , cam_zBasis );
        
        if( SHOWDEBUG ){  cout << "Transformed to lab frame!" << endl;  }
        camPatch.N = N_from_VF( camPatch.V , camPatch.F );
        if( SHOWDEBUG ){  cout << "Recomputed normals!" << endl;  }

        //  C. Trash triangles that span disconnected parts of the mesh
        camPatch = prune_big_triangles_from( MESHSIZE * 1.2f , camPatch );
        if( SHOWDEBUG ){  cout << "Pruned triangles!" << endl;  }

        //  D. Calc smooth normals
        populate_smooth_N( camPatch );
        
        //  E. Store
        patches.push_back( copy_mesh_to_heap( camPatch ) );
        if( SHOWDEBUG ){  cout << "Stowed patch!" << endl;  }

    }; // ~~~~~~ NOTE: Lambda expression must end with a semicolon ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //  5. Read the mesh points
    for( size_t i = firstMeshLine ; i < numLines ; i++ ){
        currLine = lines[i];
        //  6. If a patch header is encountered , check if there is an existing patch to be processed
        if( !currLine.compare( "OBJ" ) ){
            //  7. If there are stored lines , then fetch vertices and mesh with the camera looking down Z
            if( patchLines.size() > 0 ){  process_lines();  }
            patchLines.clear();
        //  8. else assume we are on a non-empty line containing a triple
        }else{
            patchLines.push_back( currLine );
        }
    }
    //  9. Read the last patch, if it exists
    if( patchLines.size() > 0 ){  process_lines();  }

    // 10. Calc the AABB
    size_t len = patches.size();
    for( size_t i = 0 ; i < len ; i++ ){
        if( i == 0 ){
            aabb = AABB( patches[i]->V );
        }else{
            aabb = AABB_union( aabb , AABB( patches[i]->V ) );
        }
    }

    if( SHOWDEBUG ){
		// How many patches are there?  
		cout << "There are " << patches.size() << " patches to render" << endl;  
		// How many points does each have?
        for( uint i = 0 ; i < patches.size() ; i++ ){
            cout << "\t" << patches[i]->V.rows() << " vertices , " << patches[i]->F.rows() << " triangles" << endl;
            cout << "\tBounding Box: " << endl <<  AABB( patches[i]->V ) << endl;
        }
	}
}

PatchMesh::~PatchMesh(){  clearif( patches );  }

uint PatchMesh::load_texture( string fPath ){ 
	uint handle;
	//  5. Load Texture
	if( fPath.length() == 0 )
		handle = textureHandle = LoadTexBMP( ( "robot_control/" + txtrPath ).c_str() );
	else
		handle = textureHandle = LoadTexBMP( fPath.c_str() );
	return handle;
}

// ~ Rendering ~

void PatchMesh::set_solid_color( const vec3e& clrSld ){  colorSolid = clrSld;  }

void PatchMesh::draw( float shiny , bool smoothN ){
	// 1. For each of the patches
	uint len = patches.size();
	if( !hasTextr ){  for( uint i = 0 ; i < len ; i++ ){  draw_trimesh( *patches[i] , colorSolid , shiny );              }  }
    else{             for( uint i = 0 ; i < len ; i++ ){  draw_textured_trimesh( *patches[i] , shiny , textureHandle , smoothN );  }  }
}

// ~ Robot ~

stdvec<float> PatchMesh::get_joint_state( bool deg ){  
    // Get the joint state that the shot was taken in in either degrees (default) or radians
    if( deg ){
        stdvec<float> rtnVec;
        uint len = joint_state.size();
        for( uint i = 0 ; i < len ; i++ ){  rtnVec.push_back( degrees( joint_state[i] ) );  }
        return rtnVec;
    }else{  return vec_copy( joint_state );  }  
}

// __ End PatchMesh __


// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================



// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

