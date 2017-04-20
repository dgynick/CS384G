#include <cmath>

#include "light.h"
#include <stdio.h>


using namespace std;

double DirectionalLight::distanceAttenuation(const Vec3d& P) const
{
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}


Vec3d DirectionalLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  Scene* s = getScene();
  Vec3d d = getDirection(p);
  isect i;
  Vec3d Atten(1,1,1);
  ray temp(p, d, ray::SHADOW);

  //assume opacity = 1 for now
  if(s -> intersect(temp, i)){
      return(Vec3d());
  }
  
  /*
  while(s -> intersect(temp, i)){
	//only times kt when entering???????
	temp.p = temp.p + i.t * d;
	Atten *= ((i.material) -> transparency(?));
  }
  */
  return Atten;
}

Vec3d DirectionalLight::getColor() const
{
  return color;
}

Vec3d DirectionalLight::getDirection(const Vec3d& P) const
{
  // for directional light, direction doesn't depend on P
  return -orientation;
}

double PointLight::distanceAttenuation(const Vec3d& P) const
{
  Vec3d d = P - position;
  
  return (1/(constantTerm + linearTerm * d.length() + quadraticTerm * d.length2()));
}

Vec3d PointLight::getColor() const
{
  return color;
}

Vec3d PointLight::getDirection(const Vec3d& P) const
{
  Vec3d ret = position - P;
  ret.normalize();
  return ret;
}


Vec3d PointLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  Scene* s = getScene();
  Vec3d d = getDirection(p);
  isect i;
  Vec3d Atten(1,1,1);
  //problem: for cube.ray shadowattenuation always return 0
  ray temp(p, d, ray::VISIBILITY); 
  double t0 = (position - p).length();
  //assume opacity of objects = 1 for now
  if(s -> intersect(temp, i) && i.t < t0){
     return Vec3d();
  }
  
  
  return Atten;
}
