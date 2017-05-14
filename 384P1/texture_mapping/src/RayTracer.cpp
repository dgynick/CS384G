// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"
#include "TextureMapper.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>

extern TraceUI* traceUI;

#include <iostream>
#include <fstream>

using namespace std;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = true;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

Vec3d RayTracer::trace(double x, double y)
{
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) scene->intersectCache.clear();
  ray r(Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY);
  scene->getCamera().rayThrough(x,y,r);
  Vec3d ret = traceRay(r, traceUI->getDepth(), x, y);
  // TextureMapper textureMapper;
  // Vec3d ret = textureMapper.marble(Vec2d(x , y ));
  ret.clamp();
  return ret;
}

Vec3d RayTracer::tracePixel(int i, int j){
        return tracePixel(i, j, 1);
}

Vec3d RayTracer::tracePixel(int i, int j, int supersamplePixels)
{
	Vec3d col(0,0,0);

	if( ! sceneLoaded() ) return col;
        
    for(int r = 0; r < supersamplePixels; r++){
            for(int c = 0; c < supersamplePixels; c++){
            	double x = (i + (r + 0.5)/supersamplePixels - 0.5)/double(buffer_width);
		double y = (j + (c + 0.5)/supersamplePixels - 0.5)/double(buffer_height);
                    col += trace(x , y);
            }
    }
	col /= (supersamplePixels * supersamplePixels);
        
    unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;
	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
	return col;
}


// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
Vec3d RayTracer::traceRay(ray& r, int depth, double x, double y)
{
	isect i;
	Vec3d colorC;

	if(scene->intersect(r, i)) {
		// YOUR CODE HERE

		// An intersection occurred!  We've got work to do.  For now,
		// this code gets the material for the surface that was intersected,
		// and asks that material to provide a color for the ray.  

		// This is a great place to insert code for recursive ray tracing.
		// Instead of just returning the result of shade(), add some
		// more steps: add in the contributions from reflected and refracted
		// rays.
	  const Material& mo = i.getMaterial();
	  string textureName = mo.getTextureName();
	  TextureMapper textureMapper;
	  Vec3d textureColor;
	  bool changed = true;
	  if (textureName == "marble") {
	  	textureColor = textureMapper.marble(Vec2d(x , y ));
	  }
	  else if (textureName == "fire") {
	  	textureColor = textureMapper.fire(Vec2d(x*50 , y *50));
	  }
	  else if (textureName == "cloud") {
	  	textureColor = textureMapper.cloud(Vec2d(x*20, y*20 ));
	  } else {
	  	changed = false;
	  }
	  Material m;
	  if (changed) {
	  	m = Material(mo.ke(i), mo.ka(i), mo.ks(i), textureColor, mo.kr(i), mo.kt(i), mo.shininess(i), mo.index(i), textureName);
	  } else {
	  	m = mo;
	  }
	   	  
	  colorC = m.shade(scene, r, i);
      if(depth > 0){
	    //reflection
	    Vec3d d = r.getDirection();
	    Vec3d q = r.getPosition() + d * i.t;
	    Vec3d n = i.N;
	    Vec3d r1 = d + 2 * (-d * n) * n; // reflection direction
            ray temp1(q, r1, ray::REFLECTION);
            Vec3d refl = traceRay(temp1, depth - 1, x, y);
            refl *= m.kr(i);
	    colorC += refl;

	    //transmissive
        Vec3d si = d + (-d * n) * n;
        Vec3d st;
	    if(d * n > 0){
	      //leaving
	      st = si * m.index(i);
	    }
	    else{
	      //entering
	      st = si / m.index(i);
	    }
	    if(st.length() < 1){
	      //no TIR
              ray temp2(q, st - n * sqrt(1 - st.length2()), ray::REFRACTION);
              Vec3d refr = traceRay(temp2, depth - 1, x, y);
              refr *= m.kt(i);
	      colorC += refr;
	    }
	    
	  }
	} else {
		// No intersection.  This ray travels to infinity, so we color
		// it according to the background color, which in this (simple) case
		// is just black.
	  if(haveCubeMap() && traceUI->getUsingCubeMap()){
	    colorC = cubemap -> getColor(r);
	  }
	  else{
		colorC = Vec3d(0.0, 0.0, 0.0);
	  }
	}
	return colorC;
}

RayTracer::RayTracer()
	: scene(0), buffer(0), buffer_width(256), buffer_height(256), m_bBufferReady(false), cubemap(0)
{}

RayTracer::~RayTracer()
{
	delete scene;
	delete [] buffer;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene( char* fn ) {
	ifstream ifs( fn );
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}
	
	// Strip off filename, leaving only the path:
	string path( fn );
	if( path.find_last_of( "\\/" ) == string::npos ) path = ".";
	else path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
    Parser parser( tokenizer, path );
	try {
		delete scene;
		scene = 0;
		scene = parser.parseScene();
	} 
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	}
	catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	}
	catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}

	if( !sceneLoaded() ) return false;

	scene -> buildKdTree();
	return true;
}

void RayTracer::traceSetup(int w, int h)
{
	if (buffer_width != w || buffer_height != h)
	{
		buffer_width = w;
		buffer_height = h;
		bufferSize = buffer_width * buffer_height * 3;
		delete[] buffer;
		buffer = new unsigned char[bufferSize];
	}
	memset(buffer, 0, w*h*3);
	m_bBufferReady = true;
}

