#ifndef _TEXTUREMAPPER_H__
#define _TEXTUREMAPPER_H__


#define B 0x100
#define BM 0xff
#define NN 0x1000
#define NP 12   /* 2^N */
#define NM 0xfff
#define PI 3.141592653

#define s_curve(t) ( t * t * (3. - 2. * t) )
#define lerp(t, a, b) ( a + t * (b - a) )
#define setup(i,b0,b1,r0,r1)\
        t = arg[i] + NN;\
        b0 = ((int)t) & BM;\
        b1 = (b0+1) & BM;\
        r0 = t - (int)t;\
        r1 = r0 - 1.;
#define at2(rx,ry) ( rx * q[0] + ry * q[1] )
#define at3(rx,ry,rz) ( rx * q[0] + ry * q[1] + rz * q[2] )
// Define the functions for texture mapping
// Reference: An Image Synthesizer, Ken Perlin

#include "vecmath/vec.h"

class TextureMapper
{
public:
	TextureMapper();
		~TextureMapper();

	double turbulence(Vec2d pos, double pixelSize);
	Vec3d marble(Vec2d pos);
	Vec3d cloud(Vec2d pos);
	Vec3d fire(Vec2d pos);
};

#endif // _TEXTUREMAPPER_H__
