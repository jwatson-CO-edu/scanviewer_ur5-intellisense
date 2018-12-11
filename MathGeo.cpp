/***********  
MathGeo_ASP.cpp
James Watson , 2018 October
Basic Math and 2D / 3D Geometry Utilities
NOTE: This library can be templated on either float/double using the 'MG_FLOAT'/'MG_DUBBL' flags

Template Version: 2018-06-07
***********/

#include "MathGeo.h"

// === Classes and Structs =================================================================================================================

// == class Icosahedron_d ==

// Geometry based on Paul Bourke's excellent article:
//   Platonic Solids (Regular polytopes in 3D)
//   http://paulbourke.net/geometry/platonic/

// ~ Constructors & Destructors ~
void Icosahedron_e::_init( typeF rad , const vec3e& cntr ){
    center = cntr;
    radius = rad;
    a = ( radius / ratio ) * 0.5;
    b = ( radius / ratio ) / ( 2.0f * phi );
    V = matXe::Zero( 12 ,  3 ); // Points of the mesh
    F = matXi::Zero( 20 ,  3 ); // Facets corresponding to the points V
    
    // Define the icosahedron's 12 vertices:
    V.row(  0 ) = cntr + vec3e(  0 ,  b , -a );
    V.row(  1 ) = cntr + vec3e(  b ,  a ,  0 );
    V.row(  2 ) = cntr + vec3e( -b ,  a ,  0 );
    V.row(  3 ) = cntr + vec3e(  0 ,  b ,  a );
    V.row(  4 ) = cntr + vec3e(  0 , -b ,  a );
    V.row(  5 ) = cntr + vec3e( -a ,  0 ,  b );
    V.row(  6 ) = cntr + vec3e(  0 , -b , -a );
    V.row(  7 ) = cntr + vec3e(  a ,  0 , -b );
    V.row(  8 ) = cntr + vec3e(  a ,  0 ,  b );
    V.row(  9 ) = cntr + vec3e( -a ,  0 , -b );
    V.row( 10 ) = cntr + vec3e(  b , -a ,  0 );
    V.row( 11 ) = cntr + vec3e( -b , -a ,  0 );
    
    // Define the icosahedron's 20 triangular faces:
    //   CCW            ||  CW
    F <<  2 ,  1 ,  0 , //~  0 ,  1 ,  2 ,
          1 ,  2 ,  3 , //~  3 ,  2 ,  1 ,
          5 ,  4 ,  3 , //~  3 ,  4 ,  5 ,
          4 ,  8 ,  3 , //~  3 ,  8 ,  4 ,
          7 ,  6 ,  0 , //~  0 ,  6 ,  7 ,
          6 ,  9 ,  0 , //~  0 ,  9 ,  6 ,
         11 , 10 ,  4 , //~  4 , 10 , 11 ,
         10 , 11 ,  6 , //~  6 , 11 , 10 ,
          9 ,  5 ,  2 , //~  2 ,  5 ,  9 ,
          5 ,  9 , 11 , //~ 11 ,  9 ,  5 ,
          8 ,  7 ,  1 , //~  1 ,  7 ,  8 ,
          7 ,  8 , 10 , //~ 10 ,  8 ,  7 ,
          2 ,  5 ,  3 , //~  3 ,  5 ,  2 ,
          8 ,  1 ,  3 , //~  3 ,  1 ,  8 ,
          9 ,  2 ,  0 , //~  0 ,  2 ,  9 ,
          1 ,  7 ,  0 , //~  0 ,  7 ,  1 ,
         11 ,  9 ,  6 , //~  6 ,  9 , 11 ,
          7 , 10 ,  6 , //~  6 , 10 ,  7 ,
          5 , 11 ,  4 , //~  4 , 11 ,  5 ,
         10 ,  8 ,  4 ; //~  4 ,  8 , 10 ;
         
     // Define the normals
     N = N_from_VF( V , F );
}

Icosahedron_e::Icosahedron_e(){ _init( 1.0d , vec3e( 0.0d , 0.0d , 0.0d ) ); } // Default constructor

Icosahedron_e::Icosahedron_e( typeF rad , const vec3e& cntr ){ _init( rad , cntr ); } // Parameter constructor

Icosahedron_e::~Icosahedron_e(){ /* Nothing to do here! */ } // Destructor

// ~ Getters ~
matXe& Icosahedron_e::get_vertices(){ return V; }
matXi& Icosahedron_e::get_facets(){   return F; };
matXe& Icosahedron_e::get_normals(){  return N; };

// __ End Icosahedron_d __


// == struct TriMeshVFN ==

TriMeshVFN* copy_mesh_to_heap( const TriMeshVFN& original ){
    TriMeshVFN* rtnStruct = new TriMeshVFN{};
    rtnStruct->V      = original.V; // ---- N x 3 matrix in which each row is a unique point in the mesh
    rtnStruct->F      = original.F; // ---- M x 3 matrix in which each row is a list of indices of 'V' that comprise the facet
    rtnStruct->N      = original.N; // ---- List of normal vectors corresponding to F
    rtnStruct->N_rn   = original.N_rn; // - List of normal vectors corresponding to V (for rendering purposes)
    rtnStruct->UV     = original.UV; // --- N x 2 matrix in which each row is the R2 <u,v> tuple assocated with same row 'V' R3 vertex
    rtnStruct->center = original.center; // Center of the mesh, used for some expansion operations
    rtnStruct->axis   = original.axis; // - Main axis, used for some expansion operations
    rtnStruct->type   = original.type;
    return rtnStruct;
}

TriMeshVFN  copy_trimesh( const TriMeshVFN& original ){
    TriMeshVFN rtnStruct;
    rtnStruct.V      = original.V; // ---- N x 3 matrix in which each row is a unique point in the mesh
    rtnStruct.F      = original.F; // ---- M x 3 matrix in which each row is a list of indices of 'V' that comprise the facet
    rtnStruct.N      = original.N; // ---- List of normal vectors corresponding to F
    rtnStruct.N_rn   = original.N_rn; // - List of normal vectors corresponding to V (for rendering purposes)
    rtnStruct.UV     = original.UV; // --- N x 2 matrix in which each row is the R2 <u,v> tuple assocated with same row 'V' R3 vertex
    rtnStruct.center = original.center; // Center of the mesh, used for some expansion operations
    rtnStruct.axis   = original.axis; // - Main axis, used for some expansion operations
    rtnStruct.type   = original.type;
    return rtnStruct;
}

// __ End TriMeshVFN __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

// == CSCI 5229 ==

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
typeF Cos( typeF x ){ return (typeF)cos( (x) * 3.1415927 / 180 ); }
typeF Sin( typeF x ){ return (typeF)sin( (x) * 3.1415927 / 180 ); }

vec2e polr_2_cart_0Y( const vec2e& polarCoords ){ // 0 angle is +Y North 
    // Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ 
    return vec2e{ polarCoords[0] * sinf( polarCoords[1] ) , polarCoords[0] * cosf( polarCoords[1] ) };
}

matXe circ_space( typeF dia , uint numPts , const vec2e& center ){
    // Return a list of 'numPts' points equally spaced around a 2D circle with a center at (0,0), or at 'center' if specified 
    typeF div = 2.0 * M_PI / numPts;
    matXe circPts = matXe::Zero( numPts , 2 );
    vec2e offset;
    for( uint pntDex = 0 ; pntDex < numPts ; pntDex++ ){
        offset = polr_2_cart_0Y( vec2e{ dia/2 , pntDex * div } );
        offset = center + offset;
        circPts.row( pntDex ) = offset;
    }
    return circPts;
}

matXe pts_XY_at_Z( const matXe& XY , typeF Z ){
    // Set 'XY' points in 3D space at 'Z' height
    uint len = XY.rows();
    matXe rtnPts = matXe::Zero( len , 3 );
    for( uint i = 0 ; i < len ; i++ ){  rtnPts.row(i) = vec3e{ XY(i,0) , XY(i,1) , Z };  }
    return rtnPts;
}

vec3e sphr_2_cart_pnt( typeF r , typeF th , typeF ph ){
    // For the given spherical coordinates , Return the Cartesian point
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
    return vec3e{
        r * Sin( th ) * Cos( ph ) , 
        r * Sin( ph ) , 
        r * Cos( th ) * Cos( ph )
    };
}

vec3e vec_sphr( typeF r , typeF th , typeF ps ){
    // Return a vertex in spherical coordinates , Theta axis is Z+
    // Based on code provided by Willem A. (Vlakkies) Schreüder  
    return vec3e{ r * Cos( th ) * Cos( ps ) , 
                  r * Sin( th ) * Cos( ps ) , 
                  r * Sin( ps ) };
}

matXi equilateral_tri_pixels( const vec2e& center , typeF radius ){  
    // Compute the corners of
    matXe fltVerts = equilateral_tri_vertices( center , radius );
    matXi rtnMatx = fltVerts.cast<int>();
    return rtnMatx;
}

// __ End 8229 __


// == Trigonometry ==

typeF degrees( typeF angRad ){  return (typeF)angRad * 180.0 / M_PI;  } // radians --> degrees
typeF radians( typeF angDeg ){  return (typeF)angDeg * M_PI / 180.0;  } // degrees --> radians

// __ End Trig __


// == Geo 2D ==

// Compute the vertices of an equlateral triangle that is circumscribed by a circle with 'center' and 'radius' 
matXe equilateral_tri_vertices( const vec2e& center , typeF radius ){  return circ_space( 2.0*radius , 3 , center );  }

vec2e vec2e_random(){  return vec2e( random() , random() );  } // Return a random 2D vector sampled uniformly, element-wise on [0,1)

vec2e rand_corners( const vec2e& corner1 , const vec2e& corner2 ){
    // Return a random 2D point sampled uniformly, element-wise from the given bounding box , 'corner2' is the upper bound
    vec2e span = corner2 - corner1;
    vec2e sample = vec2e_random();
    return vec2e{ corner1(0)+span(0)*sample(0) , corner1(1)+span(1)*sample(1) };
}

vec2e sample_from_box( const matXe& box ){
    // Return a random 2D point sampled uniformly, element-wise from the given bounding box
    vec2e crnr1 = box.row(0);
    vec2e crnr2 = box.row(1);
    return rand_corners( crnr1 , crnr2 );
}

typeF d_point_to_segment_2D( const vec2e& point , const Segment2D& segment ){
    // Return the shortest (perpendicular) distance between 'point' and a line 'segment' 
    // URL: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    vec2e segPt1 = segment.pnt1; 
    vec2e segPt2 = segment.pnt2;
    return abs( ( segPt2(0) - segPt1(0) ) * ( segPt1(1) - point(1)  ) - 
                ( segPt1(0) - point(0)  ) * ( segPt2(1) - segPt1(1) ) ) / 
                  sqrt( pow( segPt2(0) - segPt1(0) , 2 ) + pow( segPt2(1) - segPt1(1) , 2 ) );
}

// __ End 2D __


// == Geo 3D ==

typeF angle_between( const vec3e& vec1 , const vec3e& vec2 ){
    // Get the angle between two R3 vectors , radians
    typeF angle = acos( vec1.normalized().dot( vec2.normalized() ) ); // for now assume that there are no special cases
    if( isnan( angle ) ){
        if( vec1.normalized() == vec2.normalized() ){  return (typeF)0.0;  }
        else{  return (typeF)M_PI;  }
    }else{  return (typeF)angle;  }
}

vec3e vec3e_random(){  return vec3e( random() , random() , random() );  } // Return a random 3D vector sampled uniformly, element-wise on [0,1)

vec3e rand_corners( const vec3e& corner1 , const vec3e& corner2 ){
    // Return a random 3D point sampled uniformly, element-wise from the given bounding box , 'corner2' is the upper bound
    vec3e span = corner2 - corner1;
    vec3e sample = vec3e_random();
    return vec3e{ corner1(0)+span(0)*sample(0) , corner1(1)+span(1)*sample(1) , corner1(2)+span(2)*sample(2) };
}

matXe AABB( const matXe& V ){
    // Return the minimum and maximum corners of an AA Bounding Box of arbitrary dimension , Return size 2 x M
    size_t numCols = V.cols() , 
           numRows = V.rows() ;
    matXe corners = matXe::Zero( 2 , numCols );
    corners.row(0) = V.row(0);  corners.row(1) = V.row(0); // Init corners for a proper comparision
    for( size_t i = 0 ; i < numRows ; i++ ){
        for( size_t j = 0 ; j < numCols ; j++ ){
            corners( 0 , j ) = min( corners( 0 , j ) , V( i , j ) ); // min corner
            corners( 1 , j ) = max( corners( 1 , j ) , V( i , j ) ); // max corner
        }
    }
    return corners;
}

// Return the minimum and maximum corners of an AA Bounding Box of 'mesh'
matXe AABB( const TriMeshVFN& mesh ){  return AABB( mesh.V );  } 

// Return the minimum and maximum corners of an AA Bounding Box for a mesh collision target
matXe AABB( const TargetVFN& trgt ){  return trgt.aabb;  } // NOTE: This function assumes that the AABB has already been computed

matXe AABB_union( const matXe& aabb1 , const matXe& aabb2 ){
    // Return an AABB that encompasses both AABBs
    // NOTE: This function assumes that 'aabb1' and 'aabb2' are of the same dimensionality
    matXe allPts = vstack( aabb1 , aabb2 );
    return AABB( allPts );
}

matXe sample_from_AABB( size_t N , const matXe& aabb ){
    // Return 'N' uniform, random samples from AABB
    matXe rtnMatx = matXe::Zero( N , 3 );
    vec3e crnr1 = aabb.row(0);
    vec3e crnr2 = aabb.row(1);
    for( size_t i = 0 ; i < N ; i++ ){
        rtnMatx.row(i) = rand_corners( crnr1 , crnr2 );
    }
    return rtnMatx;
}

vec3e sample_from_AABB( const matXe& aabb ){
    // Return one uniform, random sample from AABB
    vec3e crnr1 = aabb.row(0);
    vec3e crnr2 = aabb.row(1);
    return rand_corners( crnr1 , crnr2 );
} 

vec3e get_any_perpendicular( const vec3e& query , typeF CRIT_ANG ){
    // Get any unit vector that is perpendicular to 'query'
    vec3e op = vec3e_random();
    while(  eq( angle_between( op , query ) , (typeF)0.0 , CRIT_ANG )  ){  op = vec3e_random();  }
    return op.cross( query ).normalized();
}

matXe verts3d_proj_to_plane_2D( matXe V , 
                                vec3e planePnt , vec3e normal , vec3e xBasis ){
    // Project all vertices 'V' to the defined plane given the X basis parallel to the plane
    // NOTE: This function assumes that 'planePnt' is the origin of the 2D plane
    size_t len = V.rows();
    matXe rtnMatx = matXe::Zero( len , 2 );
    vec3e queryPnt;
    vec3e diff;
    // 0. Obtain mutually orthogonal basis vectors and ensure that they are normalized
    vec3e yBasis = normal.cross( xBasis ).normalized();
    xBasis = yBasis.cross( normal ).normalized();
    for( size_t i = 0 ; i < len ; i++ ){
        queryPnt = V.row(i);
        // 1. Subtract the origin
        diff = queryPnt - planePnt;
        // 2. Project the vector onto each of the components
        rtnMatx( i , 0 ) = xBasis.dot( diff );  rtnMatx( i , 1 ) = yBasis.dot( diff );  
    }
    return rtnMatx;
}

vec3e err_vec3(){  // Return a 3D vec populated with NaN
    return vec3e( nanF("") , nanF("") , nanF("") );  
} 

vec3e basis_change( const vec3e& vec_A , 
                    const vec3e& xBasis_B , const vec3e& yBasis_B , const vec3e& zBasis_B ){
    // Express 'vec_A' in a frame that is contained in 'point_A''s current frame
    vec3e rtnVec;
    rtnVec << vec_A.dot( xBasis_B ) , vec_A.dot( yBasis_B ) , vec_A.dot( zBasis_B ) ;
    return rtnVec;
}

vec3e transform_vec( const vec3e& vec_A , 
                     const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis ){
    // Express 'vec_A' in a frame that contains 'point_A's current frame
    return ( xBasis * vec_A(0) ) + ( yBasis * vec_A(1) ) + ( zBasis * vec_A(2) );
}

vec3e point_basis_change( const vec3e& point_A  , const vec3e& origin_B , 
                          const vec3e& xBasis_B , const vec3e& yBasis_B , const vec3e& zBasis_B ){
    // Express 'point_A' in a frame that is contained in 'point_A''s current frame
    vec3e offset = point_A - origin_B;
    return basis_change( offset , xBasis_B , yBasis_B , zBasis_B );
}

vec3e transform_point( const vec3e& point_A , 
                       const vec3e& origin , 
                       const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis ){
    // Express 'point_A' in a frame that contains 'point_A's current frame
    return ( xBasis * point_A(0) ) + ( yBasis * point_A(1) ) + ( zBasis * point_A(2) ) + origin;
}

bool check_bases_orthonormal( const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis ){ 
    // Return true if { 'xBasis' , 'yBasis' , 'zBasis' } form an orthonormal basis
    bool XYperp = eqF( xBasis.dot( yBasis ) , 0.0f );
    if( !XYperp ) cout << "X and Y are NOT perpendicular! " << xBasis.dot( yBasis ) << endl;
    bool YZperp = eqF( yBasis.dot( zBasis ) , 0.0f );
    if( !YZperp ) cout << "Y and Z are NOT perpendicular! " << yBasis.dot( zBasis ) << endl;
    bool ZXperp = eqF( zBasis.dot( xBasis ) , 0.0f );
    if( !ZXperp ) cout << "Z and X are NOT perpendicular! " << zBasis.dot( xBasis ) << endl;
    bool XYrght = eqF( xBasis.cross( yBasis ).dot( zBasis ) , 1.0f );
    if( !XYrght ) cout << "X cross Y is NOT Z! Not righthand! " << xBasis.cross( yBasis ).dot( zBasis ) << endl;
    bool YZrght = eqF( yBasis.cross( zBasis ).dot( xBasis ) , 1.0f );
    if( !YZrght ) cout << "Y cross Z is NOT X! Not righthand! " << yBasis.cross( zBasis ).dot( xBasis ) << endl;
    bool ZXrght = eqF( zBasis.cross( xBasis ).dot( yBasis ) , 1.0f );
    if( !ZXrght ) cout << "Z cross X is NOT Y! Not righthand! " << zBasis.cross( xBasis ).dot( yBasis ) << endl;
    bool Xnorml = eqF( xBasis.norm() , 1.0f );
    if( !Xnorml ) cout << "X is NOT a normal vector! " << xBasis.norm() << endl;
    bool Ynorml = eqF( yBasis.norm() , 1.0f );
    if( !Ynorml ) cout << "Y is NOT a normal vector! " << yBasis.norm() << endl;
    bool Znorml = eqF( zBasis.norm() , 1.0f );
    if( !Znorml ) cout << "Z is NOT a normal vector! " << zBasis.norm() << endl;
    return XYperp && YZperp && ZXperp && XYrght && YZrght && ZXrght && Xnorml && Ynorml && Znorml;
}

IndexTypeFResult closest_point_to_sq( matXe points , vec3e queryPnt ){
    // Return the index of the point that is the closest squared distance to 'queryPnt', as well as the squared distance , linear search
    if( points.rows() > 0 ){
        vec3e compare = points.row(0);
        double dist  = 0.0;
        IndexTypeFResult rtnStruct{ 0 , ( compare - queryPnt ).squaredNorm() , true };
        size_t len   = points.rows();
        for( size_t i = 1 ; i < len ; i++ ){
            compare = points.row(i);
            dist = ( compare - queryPnt ).squaredNorm();
            if( dist < rtnStruct.measure ){
                rtnStruct.index = i;
                rtnStruct.measure = dist;
            }
        }
        return rtnStruct;
    }else{  return IndexTypeFResult{ 0 , nanF("") , false };  }
}

// __ End 3D __


// == Mesh Operations ==

vec3e get_CCW_tri_norm( const vec3e& v0 , const vec3e& v1 , const vec3e& v2 ){
    // Return the counterclockwise normal of the defined triangle
    vec3e xBasis = ( v1 - v0 ).normalized();
    vec3e vecB   = ( v2 - v0 ).normalized();
    return xBasis.cross( vecB ).normalized(); // This should already be normalized
}

vec3e get_CCW_tri_norm( const matXe& V ){
    // Return the counterclockwise normal of the defined triangle
    vec3e v0 = V.row(0);
    vec3e v1 = V.row(1);
    vec3e v2 = V.row(2);
    return get_CCW_tri_norm( v0 , v1 , v2 );
}

matXe N_from_VF( const matXe& V , const matXi& F ){
    // Given vertices 'V' and facets 'F' defined in matrix row format, return a list of normals corresponding to 'F'
    size_t len = F.rows();
    matXe allNorms = matXe::Zero( len , 3 );
    vec3e v0 , v1 , v2;
    for( size_t i = 0 ; i < len ; i++ ){
        v0 = V.row( F( i , 0 ) );
        v1 = V.row( F( i , 1 ) );
        v2 = V.row( F( i , 2 ) );
        allNorms.row( i ) = get_CCW_tri_norm( v0 , v1 , v2 );
    }
    return allNorms;
}

matXe V_in_child_frame( const matXe& V , 
                        const vec3e& origin , 
                        const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis ){
    // Express V in a frame that is contained in V's current frame
    size_t len = V.rows();
    matXe Vtrans = matXe::Zero( len , 3 );
    vec3e point;
    for( size_t i = 0 ; i < len ; i++ ){
        point = V.row(i);
        Vtrans.row(i) = point_basis_change( point , origin , xBasis , yBasis , zBasis );
    }
    return Vtrans;
}

matXe V_in_parent_frame( const matXe& V , 
                         const vec3e& origin , 
                         const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis ){
    // Express V in a frame that contains V's current frame
    size_t len = V.rows();
    matXe Vtrans = matXe::Zero( len , 3 );
    vec3e point;
    for( size_t i = 0 ; i < len ; i++ ){
        point = V.row(i);
        Vtrans.row(i) = transform_point( point , origin , xBasis , yBasis , zBasis );
    }
    return Vtrans;
}

TriMeshVFN delaunay_from_V( const matXe& V ){
    // Perform the Delaunay Triangulation on the vertices 'V', uses code by Paul Bourke
    // NOTE: This is a 2D triangulation in X-Y with facets "draped" over the points from +Z

    bool SHOWDEBUG = true;

    // ~ Inputs ~
    int  len = V.rows();
    XYZ* p   = new XYZ[ len + 3 ]; // --- The vertex array pxyz must be big enough to hold 3 more points
    XYZ  temp;
    // ~ Outputs ~
    ITRIANGLE* v    = new ITRIANGLE[ 3 * len ]; // The triangle array 'v' should be malloced to 3 * nv
    int /*- */ ntri = 0;

    // 1. Load points
    for( int i = 0 ; i < len ; i++ ){
        temp = XYZ{ V(i,0) , V(i,1) , V(i,2) };
        p[i] = temp;
    }

    // 2. The vertex array must be sorted in increasing x values say: qsort(p,nv,sizeof(XYZ),XYZCompare);
    qsort( p , len , sizeof( XYZ ) , XYZCompare );

    // 3. Obtain traingulation
    Triangulate( len , p , v , ntri );
    
    if( SHOWDEBUG ){  cout << "Built " << ntri << " triangles from " << len << " vertices!" << endl;  }

    TriMeshVFN rtnStruct;

    // 4. Load the sorted points
    rtnStruct.V = matXe::Zero( len , 3 );
    for( int i = 0 ; i < len ; i++ ){
        rtnStruct.V(i,0) = p[i].x;
        rtnStruct.V(i,1) = p[i].y;
        rtnStruct.V(i,2) = p[i].z;
    }

    // 5. Set F
    // Triangles are arranged in a consistent clockwise order by delaunay, --to-> CCW
    rtnStruct.F = matXi::Zero( ntri , 3 );
    for( int i = 0 ; i < ntri ; i++ ){
        rtnStruct.F(i,0) = v[i].p1;
        rtnStruct.F(i,1) = v[i].p2;
        rtnStruct.F(i,2) = v[i].p3;
    }

    // 6. Compute normals
    rtnStruct.N = N_from_VF( rtnStruct.V , rtnStruct.F );
    // for( int i = 0 ; i < ntri ; i++ ){  rtnStruct.N.row(i) = -rtnStruct.N.row(i);  }

    // 7. Cleanup
    delete[] p;
    delete[] v;

    // N. Return
    return rtnStruct;
}

TriMeshVFN prune_big_triangles_from( typeF sizeLimit , const TriMeshVFN& original ){ 
    // Return a version of 'original' with all triangles removed with a side longer than 'sizeLimit'
    
    bool SHOWDEBUG = false;
    
    TriMeshVFN rtnStruct;
    rtnStruct.V  = original.V; // Has the same vertices as the original, but potentially lest triangles
    rtnStruct.UV = original.UV;
    size_t numTri = original.F.rows();
    vec3e p0  , p1  , p2;
    typeF d01 , d12 , d20;
    vec3i f_i;
    vec3e n_i;
    vec2e uv_i;
    bool  hasUV = original.UV.rows() > 0;
    if( SHOWDEBUG ){  
        cerr << "About to process triangles!" << endl;
        if( hasUV )  cerr << "Incoming mesh has UV defined!" << endl;
    }
    for( size_t i = 0 ; i < numTri ; i++ ){
        p0  = original.V.row( original.F(i,0) );
        p1  = original.V.row( original.F(i,1) );
        p2  = original.V.row( original.F(i,2) );
        d01 = ( p0 - p1 ).norm();
        d12 = ( p1 - p2 ).norm();
        d20 = ( p2 - p0 ).norm();
        f_i = original.F.row(i);
        n_i = original.N.row(i);
        //~ if( hasUV )  uv_i = original.UV.row(i);

        if( ( d01 <= sizeLimit ) && ( d12 <= sizeLimit ) && ( d20 <= sizeLimit ) ){
            if( SHOWDEBUG )  cerr << "Triangle accepted!" << endl;
            rtnStruct.F = copy_F_plus_row( rtnStruct.F , f_i );
            if( SHOWDEBUG )  cerr << "Copied facet!" << endl;
            rtnStruct.N = copy_V_plus_row( rtnStruct.N , n_i );
            if( SHOWDEBUG )  cerr << "Copied normal!" << endl;
            //~ if( hasUV ){  
                //~ rtnStruct.UV = copy_V_plus_row( rtnStruct.UV , uv_i );
                //~ if( SHOWDEBUG )  cerr << "Triangle accepted!" << endl;
            //~ }
        }
    }
    if( SHOWDEBUG )  cerr << "Triangles processed!" << endl;
    return rtnStruct;
}

// __ End Mesh __


// == Collision Detection ==

typeF winding_num( vec2e& point , matXe& polygon ){
    // Find the winding number of a point with respect to a polygon , works for both CW and CCWpoints
    //  This algorithm is translation invariant, and can handle convex, nonconvex, and polygons with crossing sides.
    //  This works by shifting the point to the origin (preserving the relative position of the polygon points), and 
    // tracking how many times the positive x-axis is crossed
    
    /* NOTE: Function assumes that the points of 'polygon' are ordered. Algorithm does not work if they are not 
       NOTE: This algorithm does NOT handle the case when the point lies ON a polygon side. For this problem it is assumed that
             there are enough trial points to ignore this case */
    
    double w   = 0; 
    int    len = polygon.rows();
    matXe transformedPoly = matXe::Zero( len , 2 );
    vec2e polyPnt;
    vec2e ofstPnt;
    
    //~ for vertex in polygon: # Shift the point to the origin, preserving its relative position to polygon points
    for( int i = 0 ; i < len ; i++ ){ 
        //~ v_i.append( np.subtract( vertex , point ) )
        polyPnt = polygon.row( i );
        ofstPnt = polyPnt - point;
        transformedPoly.row( i ) = ofstPnt;
    }
    
    double x_i = 0.0;  double x_i1 = 0.0;
    double y_i = 0.0;  double y_i1 = 0.0;
    
    //~ for i in range(len(v_i)): # for each of the transformed polygon points, consider segment v_i[i]-->v_i[i+1]
    for( int i = 0 ; i < len ; i++ ){ 
    
        x_i  = transformedPoly( indexw( len , i   ) , 0 );
        x_i1 = transformedPoly( indexw( len , i+1 ) , 0 );
        y_i  = transformedPoly( indexw( len , i   ) , 1 );  
        y_i1 = transformedPoly( indexw( len , i+1 ) , 1 );  
        
        // NOTE: This function ALWAYS excludes colinear points, whether this bit is set or not
        if( eq( 0.0 , 
                d_point_to_segment_2D( vec2e{ 0 , 0 } , 
                                       Segment2D{ vec2e{ x_i  , y_i  } , 
                                                  vec2e{ x_i1 , y_i1 } } ) ) ){
            return 0.0;
        }
        
        double r = 0.0;
        
        if( y_i * y_i1 < 0 ){ // if the segment crosses the x-axis
            r = x_i + ( y_i * ( x_i1 - x_i ) ) / ( y_i - y_i1 ); // location of x-axis crossing
            if( r > 0 ){ // positive x-crossing
                if( y_i < 0 ){
                    w += 1; // CCW encirclement
                } else {
                    w -= 1; //  CW encirclement
                }
            }
        // If one of the polygon points lies on the x-axis, we must look at the segments before and after to determine encirclement
        } else if( eq( y_i , 0 ) && ( x_i > 0 ) ){ // on the positive x-axis, leaving possible crossing
            if( y_i1 > 0 ){
                w += 0.5; // possible CCW encirclement or switchback from a failed CW crossing
            } else {
                w -= 0.5; // possible CW encirclement or switchback from a failed CCW crossing
            }
        } else if( ( eq( y_i1 , 0.0 ) ) && ( x_i1 > 0.0 ) ){ // on the positive x-axis, approaching possible crossing
            if( y_i < 0 ){
                w += 0.5; // possible CCW encirclement pending
            } else {
                w -= 0.5; // possible  CW encirclement pending
            }
        }
    }
    return w;
}

//~ def point_in_poly_w( point , polygon ): 
bool point_in_poly_w( vec2e& point , matXe& polygon , bool makeCycle ){
    // Return True if the 'polygon' contains the 'point', otherwise return False, based on the winding number
    if( makeCycle ){
        vec2e firstPoint = polygon.row(0);
        matXe comparePoly = copy_V_plus_row( polygon , firstPoint );
        return !eq( winding_num( point , comparePoly ) , 0.0 ); // The winding number gives the number of times a polygon encircles a point 
    }else{
        return !eq( winding_num( point , polygon ) , 0.0 ); // The winding number gives the number of times a polygon encircles a point 
    }
}

vec3e line_intersect_plane( vec3e rayOrg , vec3e rayDir , 
                            vec3e planePnt , vec3e planeNrm ,
                            bool pntParallel ){
    // URL , Intersection point between line and plane: https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
    // NOTE: Line is defined by a ray lying on line, though this function will return an intersection point on either side of the ray origin
    //       whichever side it occurs
    double d = 0.0;
    // If the ray direction and plane normal are perpendicular, then the ray is parallel to the plane
    if(  eq( rayDir.dot( planeNrm ) , 0.0 )  ){
        // if the line segment between the plane point and the ray origin has no component in the plane normal direction, plane contains ray
        if(  eq( ( planePnt - rayOrg ).dot( planeNrm ) , 0.0 )  ){  
            if( pntParallel ){  return rayOrg;  } // If ray origin as an appropriate stand-in for the intersection of coplanar line
            else{  return err_vec3();  } // else return no-intersection
        } // Return ray origin for sake of convenience
        else{  return err_vec3();  } // else the ray is apart from and parallel to the plane, no intersection to ret
    }else{ // else the ray and plane intersect at exactly one point
        // 1. Calculate the distance along the ray that the intersection occurs
        d = ( ( planePnt - rayOrg ).dot( planeNrm ) ) / ( rayDir.dot( planeNrm ) );
        return rayOrg + rayDir * d;
    }
}

RayHits ray_intersect_VFN( const vec3e& rayOrg , const vec3e& rayDir , const TriMeshVFN& mesh ){
    // Return all the intersection points with the mesh , and classify them as either entry or exit points
    
    bool SHOWDEBUG = false;
    
    RayHits rtnStruct;  rtnStruct.anyHits = false;
    size_t numTris = mesh.F.rows();
    matXe tri = matXe::Zero( 4 , 2 );
    vec3e xBasis , yBasis , zBasis ,
          v0 , v1 , v2 , 
          interPnt ,
          temp ;
    vec2e intPnt2D ,
          v0flt , v1flt , v2flt ;
    
    //  1. For each triangle
    for( size_t f_i = 0 ; f_i < numTris ; f_i++ ){
        // Get plane point and normal from the mesh
        v0 = mesh.V.row( mesh.F( f_i , 0 ) );
        zBasis = mesh.N.row( f_i );  zBasis.normalize();
        //  2. Calc the intersection point
        interPnt = line_intersect_plane( rayOrg , rayDir , v0 , zBasis , false ); // ray parallel to facet doesn't intersect in this context
        //  3. If a valid intersection was returned
        if( !is_err( interPnt ) ){
            
            if( SHOWDEBUG ){  cout << "Plane intersection at " << interPnt << endl;  }
            
            //  4. Determine if the intersection happens in front of the ray
            if(  ( interPnt - rayOrg ).dot( rayDir ) >= 0.0  ){
                
                if( SHOWDEBUG ){  cout << "Intersection in FRONT of ray!" << endl;  }
                
                //  5. Get plane basis vectors from the triangle
                v1 = mesh.V.row( mesh.F( f_i , 1 ) );
                v2 = mesh.V.row( mesh.F( f_i , 2 ) );
                xBasis = ( v1 - v0 ).normalized();
                yBasis = zBasis.cross( xBasis ).normalized();
                //  6. Project the point onto the plane
                temp = point_basis_change( interPnt , v0 , xBasis , yBasis , zBasis );
                intPnt2D << temp(0) , temp(1);
                //  7. Project the triangle onto the plane
                // v0 will always be at ( 0 , 0 ) , per above
                // v1 will always be at ( ( v1 - v0 ) * x , 0 ) , per above
                tri( 1 , 0 ) = ( v1 - v0 ).dot( xBasis );
                // v2 will have ( x , y )
                temp = point_basis_change( v2 , v0 , xBasis , yBasis , zBasis );
                tri( 2 , 0 ) = temp( 0 );
                tri( 2 , 1 ) = temp( 1 );
                // v3 return to v0 for cycle = ( 0 , 0 ) , easy!
            
                if( SHOWDEBUG ){  
                    cout << "Flat Point: " << intPnt2D << endl
                         << "Flat Tri" << endl
                         << tri << endl;
                }
                
                //  8. point in poly test , If the point is inside the triangle
                if(  point_in_poly_w( intPnt2D , tri )  ){
                    
                    if( SHOWDEBUG ){  cout << "Point INSIDE tri!"  << endl
                                           << "Direction Dot: " << zBasis.dot( rayDir ) << endl;  }
                    if( !rtnStruct.anyHits ){  rtnStruct.anyHits = true;  }
                    //  9. Dot the ray with the tri norm, if negative
                    if( zBasis.dot( rayDir ) < 0 ){
                        // 10. else append to entrances
                        rtnStruct.enter = copy_V_plus_row( rtnStruct.enter , interPnt );
                        rtnStruct.n_Metric = copy_column_plus_numF( rtnStruct.n_Metric , angle_between( -rayDir , zBasis ) );
                        if( SHOWDEBUG ){  
                            cout << "n_Metric"  << endl
                                 << rtnStruct.n_Metric << endl;
                        }
                    }else{ 
                        // 11. Append to exits
                        rtnStruct.exit  = copy_V_plus_row( rtnStruct.exit  , interPnt );
                        rtnStruct.x_Metric = copy_column_plus_numF( rtnStruct.x_Metric , angle_between(  rayDir , zBasis ) );
                        if( SHOWDEBUG ){  
                            cout << "x_Metric"  << endl
                                 << rtnStruct.x_Metric << endl;
                        }
                    }
                }else{ // else plane intersection outside of tri bounds, no action
                    if( SHOWDEBUG ){  cout << "Point OUTSIDE tri!"  << endl;  }
                }
            }else{ // else triangle is behind ray, no collision
                if( SHOWDEBUG ){  cout << "Intersection BEHIND ray!" << endl;  }
            }
        } // else the ray missed the triangle plane
        if( SHOWDEBUG ){  cout << endl;  }
    }
    if( SHOWDEBUG ){  cout << endl;  }
    // 12. Return all hits
    return rtnStruct;
}

vec3e ray_intersect_AABB( const vec3e& origin , const vec3e& dir , const matXe& aabb ){
    /* 1. Fast Ray-Box Intersection
          by Andrew Woo
          from "Graphics Gems", Academic Press, 1990
          URL: https://web.archive.org/web/20090803054252/http://tog.acm.org/resources/GraphicsGems/gems/RayBox.c
       2. Adapted by Eric Haines
          URL: https://github.com/erich666/GraphicsGems/blob/master/gems/RayBox.c
       3. Adapted for C++ / Eigen by James Watson
          URL: https://bitbucket.org/jwatson_utah_edu/scanviewer_ur5-intellisense/src/master/MathGeo.cpp
    */
    // Return the last point at which the ray intersects the AABB , otherwise return ( NaN , NaN , NaN )
    
    size_t NUMDIM     = 3 , 
           RIGHT      = 0 , 
           LEFT /*-*/ = 1 , 
           MIDDLE     = 2 , 
           i /* -- */ = 0 ,
           whichPlane = 0 ;
    bool inside = true;
    size_t quadrant[ NUMDIM ];
    double maxT[NUMDIM];
    double candidatePlane[NUMDIM];
    
    vec3e minB = aabb.row(0);
    vec3e maxB = aabb.row(1);
    
    vec3e coord;

    /* Find candidate planes; this loop can be avoided if rays cast all from the eye ( assume perpsective view ) */
    for( i = 0 ; i < NUMDIM ; i++ ){
        if( origin(i) < minB(i) ){
            quadrant[i] = LEFT;
            candidatePlane[i] = minB(i);
            inside = false;
        }else if( origin(i) > maxB(i) ){
            quadrant[i] = RIGHT;
            candidatePlane[i] = maxB(i);
            inside = false;
        }else{
            quadrant[i] = MIDDLE;
        }
    }

    /* Ray origin inside bounding box */
    if( inside ){
        //~ coord = origin;
        //~ return (TRUE);
        return origin;
    }

    /* Calculate T distances to candidate planes */
    for( i = 0 ; i < NUMDIM ; i++ ){
        if( quadrant[i] != MIDDLE && dir[i] != 0.0 ) 
            maxT[i] = ( candidatePlane[i] - origin(i) ) / dir(i);
        else
            maxT[i] = -1.0;
    }

    /* Get largest of the maxT's for final choice of intersection */
    whichPlane = 0;
    for( i = 1 ; i < NUMDIM ; i++ ){
        if( maxT[ whichPlane ] < maxT[i] ){  whichPlane = i;  }
    }

    /* Check final candidate actually inside box */
    if( maxT[ whichPlane ] < 0.0){  return err_vec3();  }
    
    for( i = 0 ; i < NUMDIM ; i++ ){
        if( whichPlane != i ){
            coord(i) = origin(i) + maxT[ whichPlane ] * dir(i);
            if(  ( coord(i) < minB(i) )  ||  ( coord(i) > maxB(i) )  ){  return err_vec3();  }
        } else {
            coord(i) = candidatePlane[i];
        }
    }
    return coord;				/* ray hits box */
} 

TargetVFN target_mesh_from_trimesh( const TriMeshVFN& original ){
    // Convert the mesh to target with bounding box
    TargetVFN rtnStruct;
    rtnStruct.mesh = copy_trimesh( original );
    rtnStruct.aabb = AABB( original );
    return rtnStruct;
}

TargetVFN* heap_target_from_trimesh( const TriMeshVFN& original ){
    // Allocate && Convert the mesh to target with bounding box
    return new TargetVFN{
        copy_trimesh( original ) , 
        AABB( original )
    };
}

RayHits ray_intersect_TargetVFN( const vec3e& rayOrg , const vec3e& rayDir , const TargetVFN& target ){
    // Fast collision recording between ray and mesh-target
    // NOTE: This function assumes that 'target' AABB and mesh concur and are up to date
    RayHits rtnStruct;  rtnStruct.anyHits = false;
    
    // 1. If there is a collision with the bounding box, then we may proceed with the more costly computation of mesh collisions
    if(  !is_err(  ray_intersect_AABB( rayOrg , rayDir , target.aabb )  )  ){
        rtnStruct = ray_intersect_VFN( rayOrg , rayDir , target.mesh );
    }
    
    return rtnStruct;
}

// __ End Collision __


// == Print Helpers ==

std::ostream& operator<<( std::ostream& os , const vec3e& vec ){ 
     // Print a 3D vector to a stream
    os << "[ " << vec(0) <<  " , " << vec(1) <<  " , " << vec(2) << " ]";
    return os;
}

std::ostream& operator<<( std::ostream& os , const vec2e& vec ){ 
     // Print a 2D vector to a stream
    os << "[ " << vec(0) <<  " , " << vec(1) << " ]";
    return os;
}

// __ End Print __


// == I/O Helpers ==

vec3e str_to_vec3( string delimitedTriple , char delimiter ){
    // Interpret 'delimitedTriple' as an R3 vector
    std::vector<double> elems = tokenize_to_dbbl_w_separator( delimitedTriple , delimiter );
    if( elems.size() == 3 )
        #ifdef MG_FLOAT
            return vec3e{ (float) elems[0] , (float) elems[1] , (float) elems[2] };
        #endif
        #ifdef MG_DUBBL
            return vec3e{ elems[0] , elems[1] , elems[2] };
        #endif
    else
        return err_vec3();
}

// __ End I/O __


// == Struct Helpers ==

// Test if the error vector was returned
bool is_err( const vec3e& query ){  return isnan( query(0) ) || isnan( query(1) ) || isnan( query(2) );  }

matXe copy_V_plus_row( const matXe& pMatx , const vec3e& nuVec ){ 
    // Extend vertices list by 1 R3 vector
    // NOTE: This function assumes that 'pMatx' is either empty or has 3 columns
    // NOTE: This function is not efficient
    
    bool SHOWDEBUG = false;
    
    matXe rtnMatx;
    size_t pRows = pMatx.rows();
    if( SHOWDEBUG ){  cout << "Found a matrix with " << pRows << " rows" << endl;  }
    if( pRows < 1 ){
        rtnMatx = matXe::Zero(1,3);
        rtnMatx.row(0) = nuVec;
    }else{
        rtnMatx = matXe::Zero( pRows+1 , 3 );
        rtnMatx.block( 0 , 0 , pRows , 3 ) = pMatx;
        rtnMatx.row( pRows ) = nuVec;
    }
    
    if( SHOWDEBUG ){
        cout << "About to return expanded matrix ..." << endl;
        cout << rtnMatx << endl;
    }
    
    return rtnMatx;
}

matXe copy_V_plus_row( const matXe& pMatx , const vec2e& nuVec ){ 
    // Extend vertices list by 1 R2 vector , return copy
    matXe rtnMatx;
    size_t pRows = pMatx.rows();
    if( pRows < 1 ){
        rtnMatx = matXe::Zero(1,2);
        rtnMatx.row(0) = nuVec;
    }else{
        rtnMatx = matXe::Zero( pRows+1 , 2 );
        rtnMatx.block( 0 , 0 , pRows , 3 ) = pMatx;
        rtnMatx.row( pRows ) = nuVec;
    }
    
    return rtnMatx;
}

matXi copy_F_plus_row( const matXi& pMatx , const vec3i& nuVec ){
    // Extend vertices list by 1 I3 vector , return copy
    matXi rtnMatx;
    size_t pRows = pMatx.rows();
    if( pRows < 1 ){
        rtnMatx = matXi::Zero(1,3);
        rtnMatx.row(0) = nuVec;
    }else{
        rtnMatx = matXi::Zero( pRows+1 , 3 );
        rtnMatx.block( 0 , 0 , pRows , 3 ) = pMatx;
        rtnMatx.row( pRows ) = nuVec;
    }
    return rtnMatx;
}

matXe copy_column_plus_numF( const matXe& columnMatx , typeF nuNum ){ 
    // Extend column by 1 number , return copy
    matXe rtnMatx;
    size_t pRows = columnMatx.rows();
    if( pRows < 1 ){
        rtnMatx = matXe::Zero(1,1);
        rtnMatx.row(0) << nuNum;
    }else{
        rtnMatx = matXe::Zero( pRows+1 , 1 );
        rtnMatx.block( 0 , 0 , pRows , 1 ) = columnMatx;
        rtnMatx.row( pRows ) << nuNum;
    }
    return rtnMatx;
}

RayHits& operator+=( RayHits& opLeft , const RayHits& opRght ){
    // Combine hits in a sensible way
    opLeft.anyHits  = opLeft.anyHits || opRght.anyHits;
    opLeft.enter    = vstack( opLeft.enter , opRght.enter );
    opLeft.exit     = vstack( opLeft.exit , opRght.exit );
    opLeft.n_Metric = vstack( opLeft.n_Metric , opRght.n_Metric ); // WHAT TO DO IF BOTH OPERANDS ARE EMPTY?
    opLeft.x_Metric = vstack( opLeft.x_Metric , opRght.x_Metric );
    opLeft.n_Index  = vec_join( opLeft.n_Index , opRght.n_Index );
    opLeft.x_Index  = vec_join( opLeft.x_Index , opRght.x_Index );
    return opLeft;
}

void assign_num_entries_exits( const RayHits& hits , size_t& numEntr , size_t& numExit ){
    // Assign the number of entries and exits recorded in 'hits' to 'numEntr' and 'numExit', respectively
    numEntr = hits.enter.rows();
    numExit = hits.exit.rows();
}

matXe vstack( const matXe& A , const matXe& B ){
    // URL , Stack two matrices vertically: https://stackoverflow.com/a/21496281
    // NOTE: This function assumes that 'A' and 'B' have the same number of columns
    size_t Alen = A.rows() ,
           Blen = B.rows() ;
    if( Alen < 1 ){  return B;  } // If either matrix is empty , return the other
    if( Blen < 1 ){  return A;  }
    matXe rtnMatx( Alen + Blen , A.cols() );
    rtnMatx << A , 
               B ;
    return rtnMatx;
}

matXi vstack( const matXi& A , const matXi& B ){
    // URL , Stack two matrices vertically: https://stackoverflow.com/a/21496281
    // NOTE: This function assumes that 'A' and 'B' have the same number of columns
    size_t Alen = A.rows() ,
           Blen = B.rows() ;
    if( Alen < 1 ){  return B;  } // If either matrix is empty , return the other
    if( Blen < 1 ){  return A;  }
    matXi rtnMatx( Alen + Blen , A.cols() );
    rtnMatx << A , 
               B ;
    return rtnMatx;
}

// __ End Struct __


// ___ End Func ____________________________________________________________________________________________________________________________





/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

