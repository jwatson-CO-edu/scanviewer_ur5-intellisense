#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
MathGeo_ASP.h
James Watson , 2018 October
Basic Math and 2D / 3D Geometry Utilities
NOTE: This library can be templated on either float/double using the 'MG_FLOAT'/'MG_DUBBL' flags

Template Version: 2018-07-16
***********/

#ifndef MATH_GEO_H // This pattern is to prevent symbols to be loaded multiple times
#define MATH_GEO_H // from multiple imports

#define MG_FLOAT // Use floats for all Eigen operations
//~ #define MG_DUBBL // Use doubles for all Eigen operations

// ~~ Includes ~~
// ~ Eigen ~
#include <Eigen/Core> // ---- The living heart of Eigen
#include <Eigen/Dense> // --- Cross Product , etc.
#include <Eigen/Geometry> //- Quaternion , etc
// ~ Local ~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/
#include "Delaunay.h"

// ~~ Shortcuts and Aliases ~~
// ~ Eigen ~
#ifdef MG_FLOAT
    using vec2e = Eigen::Vector2f;
    using vec3e = Eigen::Vector3f;
    using matXe = Eigen::MatrixXf;
    using typeF = float;
    #define random           rand_float
    #define nanF             nanf
    #define eqF              eqf
    #define IndexTypeFResult IndexFloatResult
#endif
#ifdef MG_DUBBL
    using vec2e = Eigen::Vector2d;
    using vec3e = Eigen::Vector3d;
    using matXe = Eigen::MatrixXd;
    using typeF = double;
    #define random           rand_dbbl
    #define nanF             nan
    #define eqF              eq
    #define IndexTypeFResult IndexDbblResult
#endif
using vec2i = Eigen::Vector2i;
using vec3i = Eigen::Vector3i;
using matXi = Eigen::MatrixXi;


// ~~ Constants ~~
const size_t ICOS_SPHERE_DIVISN  =   7; // ---- Divide each icosahedron triangle into 28 triangles
const size_t CIRCLE_DIVISION     = 180; // ---- Divide each circle into 300 Segments
const typeF  GEO_CRIT_ANG /*- */ =   0.0349; // Angle Criterion , 2deg

// === Classes and Structs =================================================================================================================

// == Geometric Primitives ==

struct Segment2D{
    // Represents a line segment between two endpoints
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vec2e pnt1; 
    vec2e pnt2; 
};

// __ End Primitive __


// == Geometric Objects ==

struct FrameBases{
    // Represents a frame of reference
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vec3e origin;
    vec3e xBasis;
    vec3e yBasis;
    vec3e zBasis;
};

// __ End Objects __

// == class Icosahedron_e ==

// Geometry based on Paul Bourke's excellent article:
//   Platonic Solids (Regular polytopes in 3D)
//   http://astronomy.swin.edu.au/~pbourke/polyhedra/platonic/

class Icosahedron_e{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ~ Constants ~
    typeF sqrt5 = (typeF) sqrt( 5.0d ); // ----------------------------------- Square root of 5
    typeF phi   = (typeF)( 1.0d + sqrt5 ) * 0.5d; // ------------------------- The Golden Ratio
    typeF ratio = (typeF)sqrt( 10.0d + ( 2.0d * sqrt5 ) ) / ( 4.0d * phi ); // ratio of edge length to radius
    
    // ~ Variables ~
    vec3e center;
    typeF /* -- */ radius;
    typeF /* -- */ a; 
    typeF /* -- */ b; 
    matXe V; // Points of the mesh
    matXi F; // Facets corresponding to the points V
    matXe N; // Mesh normals
    
    // ~ Constructors & Destructors ~
    Icosahedron_e(); // ------------------------------ Default constructor
    Icosahedron_e( typeF rad , const vec3e& cntr ); // Parameter constructor
    ~Icosahedron_e(); // ----------------------------- Destructor
    
    // ~ Getters ~
    matXe& get_vertices();
    matXi& get_facets();
    matXe& get_normals();
    
protected:
    // ~ Init ~
    void _init( typeF rad , const vec3e& cntr );
};

// __ End Icosahedron_e __


// == struct TriMeshVFN ==

enum MESHTYPE{ GENERIC }; //- Default mesh type

struct TriMeshVFN{
    // Trimesh with some extra structure
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    matXe    V; // ---- N x 3 matrix in which each row is a unique point in the mesh
    matXi    F; // ---- M x 3 matrix in which each row is a list of indices of 'V' that comprise the facet
    matXe    N; // ---- List of normal vectors corresponding to F
    matXe    N_rn; // - List of normal vectors corresponding to V (for rendering purposes)
    matXe    UV; // --- N x 2 matrix in which each row is the R2 <u,v> tuple assocated with same row 'V' R3 vertex
    vec3e    center; // Center of the mesh, used for some expansion operations
    vec3e    axis; // - Main axis, used for some expansion operations
    MESHTYPE type = GENERIC;
};

TriMeshVFN* copy_mesh_to_heap( const TriMeshVFN& original );

TriMeshVFN  copy_trimesh( const TriMeshVFN& original );

// __ End TriMeshVFN __


// == Collision Structs ==

struct TargetVFN{ 
    // TriMesh and its bounding box , AABB is to make collision checking faster
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TriMeshVFN  mesh;
    matXe /*-*/ aabb;
};

TargetVFN  target_mesh_from_trimesh( const TriMeshVFN& original ); // Convert the mesh to target with bounding box
TargetVFN* heap_target_from_trimesh( const TriMeshVFN& original ); // Allocate && Convert the mesh to target with bounding box

struct RayHits{
    // A record of all ray-mesh intersections, including entries and exits
    // NOTE: For nonclosed meshes, there may be an uneven number of entries and exits
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool /* --- */ anyHits = false; // Were there any intersections recorded?
    matXe /* -- */ enter; // --------- Row-list of entry points
    matXe /* -- */ exit; // ---------- Row-list of exit points
    matXe /* -- */ n_Metric; // ------ Generic entry metrics to be populated by client code (e.g. grasp pair angles)
    matXe /* -- */ x_Metric; // ------ Generic exit  metrics to be populated by client code (e.g. grasp pair angles)
    stdvec<size_t> n_Index; // ------- Indices related to entries
    stdvec<size_t> x_Index; // ------- Indices related to exits
};

RayHits& operator+=( RayHits& opLeft , const RayHits& opRght );

// Assign the number of entries and exits recorded in 'hits' to 'numEntr' and 'numExit', respectively
void assign_num_entries_exits( const RayHits& hits , size_t& numEntr , size_t& numExit );

// __ End Collision __


// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

// == CSCI 5229 ==

// Cosine and Sine in degrees
typeF Cos( typeF x );
typeF Sin( typeF x );

vec2e polr_2_cart_0Y( const vec2e& polarCoords ); // Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ 

// Return a list of 'numPts' points equally spaced around a 2D circle with a center at (0,0), or at 'center' if specified 
matXe circ_space( float dia , uint numPts , const vec2e& center );

matXe pts_XY_at_Z( const matXe& XY , typeF Z ); // Set 'XY' points in 3D space at 'Z' height

vec3e sphr_2_cart_pnt( typeF r , typeF th , typeF ph ); // For the given spherical coordinates , Return the Cartesian point

vec3e vec_sphr( typeF r , typeF th , typeF ps ); // Return a vertex in spherical coordinates , Theta axis is Z+

// Compute the vertices of an equlateral triangle that is circumscribed by a circle with 'center' and 'radius' , Pixel version
matXi equilateral_tri_pixels( const vec2e& center , typeF radius );

// __ End 8229 __


// == Trigonometry ==

typeF degrees( typeF angRad ); // radians --> degrees
typeF radians( typeF angDeg ); // degrees --> radians

// __ End Trig __


// == Geo 2D ==

// Compute the vertices of an equlateral triangle that is circumscribed by a circle with 'center' and 'radius' 
matXe equilateral_tri_vertices( const vec2e& center , typeF radius ); 

vec2e vec2e_random();  // Return a random 2D vector sampled uniformly, element-wise on [0,1)

// Return a random 2D point sampled uniformly, element-wise from the given bounding box , 'corner2' is the upper bound
vec2e rand_corners( const vec2e& corner1 , const vec2e& corner2 );

vec2e sample_from_box( const matXe& box ); // Return a random 2D point sampled uniformly, element-wise from the given bounding box

// Return the shortest (perpendicular) distance between 'point' and a line 'segment' 
typeF d_point_to_segment_2D( const vec2e& point , const Segment2D& segment ); 

// __ End 2D __


// == Geo 3D ==

typeF angle_between( const vec3e& vec1 , const vec3e& vec2 ); // Get the angle between two R3 vectors , radians

vec3e vec3e_random(); // Return a random 2D vector sampled uniformly, element-wise on [0,1)

// Return a random 2D point sampled uniformly, element-wise from the given bounding box , 'corner2' is the upper bound
vec3e vec3e_rand_corners( const vec3e& corner1 , const vec3e& corner2 );

matXe AABB( const matXe& V ); // Return the minimum and maximum corners of an AA Bounding Box of arbitrary dimension , Return size 2 x M
matXe AABB( const TriMeshVFN& mesh ); // Return the minimum and maximum corners of an AA Bounding Box of 'mesh'
matXe AABB( const TargetVFN&  trgt ); // Return the minimum and maximum corners of an AA Bounding Box for a mesh collision target

matXe AABB_union( const matXe& aabb1 , const matXe& aabb2 ); // Return an AABB that encompasses both AABBs

matXe sample_from_AABB( size_t N , const matXe& aabb ); // Return 'N' uniform, random samples from AABB
vec3e sample_from_AABB( const matXe& aabb ); // Return one uniform, random sample from AABB

vec3e get_any_perpendicular( const vec3e& query , typeF CRIT_ANG = GEO_CRIT_ANG ); // Get any unit vector that is perpendicular to 'query'

// Project all vertices 'V' to the defined plane given the X basis parallel to the plane
matXe verts3d_proj_to_plane_2D( matXe V , 
                                vec3e planePnt , vec3e normal , vec3e xBasis );

vec3e err_vec3(); // Return a 3D vec populated with NaN

// Express 'vec_A' in a (child) frame that is contained in 'point_A''s current frame
vec3e basis_change( const vec3e& vec_A , 
                    const vec3e& xBasis_B , const vec3e& yBasis_B , const vec3e& zBasis_B );

// Express 'vec_A' in a (parent) frame that contains 'point_A's current frame
vec3e transform_vec( const vec3e& vec_A , 
                     const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis );

// Express 'point_A' in a (child) frame that is contained in 'point_A''s current frame
vec3e point_basis_change( const vec3e& point_A  , const vec3e& origin_B , 
                          const vec3e& xBasis_B , const vec3e& yBasis_B , const vec3e& zBasis_B );

// Express 'point_A' in a (parent) frame that contains 'point_A's current frame
vec3e transform_point( const vec3e& point_A , 
                       const vec3e& origin , 
                       const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis );

// Return true if { 'xBasis' , 'yBasis' , 'zBasis' } form an orthonormal basis
bool check_bases_orthonormal( const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis );

// Return the index of the point that is the closest squared distance to 'queryPnt', as well as the squared distance , linear search
IndexTypeFResult closest_point_to_sq( matXe points , vec3e queryPnt );

// __ End 3D __


// == Mesh Operations ==

vec3e get_CCW_tri_norm( const vec3e& v0 , const vec3e& v1 , const vec3e& v2 ); // Return the counterclockwise normal of the defined triangle

vec3e get_CCW_tri_norm( const matXe& V ); // Return the counterclockwise normal of the defined triangle

// Given vertices 'V' and facets 'F' defined in matrix row format, return a list of normals corresponding to 'F'
matXe N_from_VF( const matXe& V , const matXi& F ); 

// Express V in a frame that is contained in V's current frame
matXe V_in_child_frame( const matXe& V , 
                        const vec3e& origin , 
                        const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis );

// Express V in a frame that contains V's current frame
matXe V_in_parent_frame( const matXe& V , 
                         const vec3e& origin , 
                         const vec3e& xBasis , const vec3e& yBasis , const vec3e& zBasis );

TriMeshVFN delaunay_from_V( const matXe& V ); // Perform the Delaunay Triangulation on the vertices 'V', uses code by Paul Bourke

// Return a version of 'original' with all triangles removed with a side longer than 'sizeLimit'
TriMeshVFN prune_big_triangles_from( typeF sizeLimit , const TriMeshVFN& original );

// __ End Mesh __


// == Collision Detection ==

typeF winding_num( vec2e& point , matXe& polygon ); // Return the number of times polygon winds around point

bool point_in_poly_w( vec2e& point , matXe& polygon , bool makeCycle = false ); // Return true for a nonzero winding number

// URL , Intersection point between line and plane: https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
vec3e line_intersect_plane( vec3e rayOrg , vec3e rayDir , 
                            vec3e planePnt , vec3e planeNrm ,
                            bool pntParallel = true );

// Return all the intersection points with the mesh , and classify them as either entry or exit points
RayHits ray_intersect_VFN( const vec3e& rayOrg , const vec3e& rayDir , const TriMeshVFN& mesh );

// Return the last point at which the ray intersects the AABB , otherwise return ( NaN , NaN , NaN )
vec3e ray_intersect_AABB( const vec3e& origin , const vec3e& dir , const matXe& aabb );

// Fast collision recording between ray and mesh-target
RayHits ray_intersect_TargetVFN( const vec3e& rayOrg , const vec3e& rayDir , const TargetVFN& target );

// __ End Collision __


// == Print Helpers ==

std::ostream& operator<<( std::ostream& os , const vec3e& vec ); // Print a 3D vector to a stream

std::ostream& operator<<( std::ostream& os , const vec2e& vec ); // Print a 2D vector to a stream

// __ End Print __


// == I/O Helpers ==

vec3e str_to_vec3( string delimitedTriple , char delimiter ); // Interpret 'delimitedTriple' as an R3 vector

// __ End I/O __


// == Struct Helpers ==

bool is_err( const vec3e& query ); // Test if the error vector was returned

matXe copy_V_plus_row( const matXe& pMatx , const vec3e& nuVec ); // Append 3D typeF vector to matrix
matXe copy_V_plus_row( const matXe& pMatx , const vec2e& nuVec ); // Append 2D typeF vector to matrix

matXi copy_F_plus_row( const matXi& pMatx , const vec3i& nuVec ); // Append 3D integer vector to matrix

matXe copy_column_plus_numF( const matXe& columnMatx , typeF nuNum ); // Extend column by 1 number , return copy

matXe vstack( const matXe& A , const matXe& B ); // Stack two matrices vertically
matXi vstack( const matXi& A , const matXi& B ); // Stack two matrices vertically

// __ End Struct __


// ___ End Func ____________________________________________________________________________________________________________________________




#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

