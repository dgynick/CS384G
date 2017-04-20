#include <cmath>
#include <float.h>
#include <algorithm>
#include <assert.h>
#include "trimesh.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

using namespace std;

Trimesh::~Trimesh()
{
	for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
		delete *i;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const Vec3d &v )
{
    vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
    materials.push_back( m );
}

void Trimesh::addNormal( const Vec3d &n )
{
    normals.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
    int vcnt = vertices.size();

    if( a >= vcnt || b >= vcnt || c >= vcnt ) return false;

    TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
    newFace->setTransform(this->transform);
    if (!newFace->degen) faces.push_back( newFace );


    // Don't add faces to the scene's object list so we can cull by bounding box
    // scene->add(newFace);
    return true;
}

char* Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
    if( !materials.empty() && materials.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of materials.";
    if( !normals.empty() && normals.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of normals.";

    return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	double tmin = 0.0;
	double tmax = 0.0;
	typedef Faces::const_iterator iter;
	bool have_one = false;
	for( iter j = faces.begin(); j != faces.end(); ++j )
	  {
	    isect cur;
	    if( (*j)->intersectLocal( r, cur ) )
	      {
		if( !have_one || (cur.t < i.t) )
		  {
		    i = cur;
		    have_one = true;
		  }
	      }
	  }
	if( !have_one ) i.setT(1000.0);
	return have_one;
}

bool TrimeshFace::intersect(ray& r, isect& i) const {
  return intersectLocal(r, i);
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in alpha, beta and gamma.
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{

    const Vec3d& a = parent->vertices[ids[0]];
    const Vec3d& b = parent->vertices[ids[1]];
    const Vec3d& c = parent->vertices[ids[2]];

    // YOUR CODE HERE
    
    // a,b,c in world coordinates?

    //compute surface parameters
    const Vec3d& n = crossprod(a - c, b - c);
    const double& d = -(n * a);

    //compute t
    const Vec3d& rp = r.getPosition();
    const Vec3d& rd = r.getDirection();
    double t = n * rd;
    if(n.length() > 1000 * t){
      //parallel
      return false;
    }

    t = -(n * rp + d)/t;
    
    if(t < 0){
      return false;
    }
    
    i.setT(t);

    //compute bary coords
    //to-do: consider the case where abc is a line after being projected to x-y plane?
    Vec3d p = rp + t * rd;
    double abx = b[0] - a[0];
    double aby = b[1] - a[1];
    double acx = c[0] - a[0];
    double acy = c[1] - a[1];
    double apx = p[0] - a[0];
    double apy = p[1] - a[1];
    

    double sabc = abx * acy - aby * acx;
    double sabp = abx * apy - aby * apx;
    double sapc = apx * acy - apy * acx;

    double beta = sapc/sabc;
    double gamma = sabp/sabc;
    double alpha = 1 - beta - gamma;

    if(alpha < 0 || beta < 0 || gamma < 0){
      return false;
    }
    i.setBary(alpha, beta, gamma);

    i.setObject(this);
    i.setMaterial(this->getMaterial());
    if(parent -> vertNorms){
      Vec3d norm = (alpha * parent -> normals[ids[0]] + beta * parent -> normals[ids[1]] + gamma * parent -> normals[ids[2]]);
      norm.normalize();
      i.setN(norm);
    }else{
		 i.setN(this->normal);//if parent-> vertnorm = true then need to use bary
    }


    //to-do: set uv
    return true;
}

void Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
    int cnt = vertices.size();
    normals.resize( cnt );
    int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
    memset( numFaces, 0, sizeof(int)*cnt );
    
    for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
    {
		Vec3d faceNormal = (**fi).getNormal();
        
        for( int i = 0; i < 3; ++i )
        {
            normals[(**fi)[i]] += faceNormal;
            ++numFaces[(**fi)[i]];
        }
    }

    for( int i = 0; i < cnt; ++i )
    {
        if( numFaces[i] )
            normals[i]  /= numFaces[i];
    }

    delete [] numFaces;
    vertNorms = true;
}
