#include <cmath>

#include "scene.h"
#include "light.h"
#include "../ui/TraceUI.h"
#include <queue>
#include <utility>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>

using namespace std;

extern TraceUI* traceUI;

bool Geometry::intersect(ray& r, isect& i) const {
	double tmin, tmax;
	if (hasBoundingBoxCapability() && !(bounds.intersect(r, tmin, tmax))) return false;
	// Transform the ray into the object's local coordinate space
	Vec3d pos = transform->globalToLocalCoords(r.p);
	Vec3d dir = transform->globalToLocalCoords(r.p + r.d) - pos;
	double length = dir.length();
	dir /= length;
	Vec3d Wpos = r.p;
	Vec3d Wdir = r.d;
	r.p = pos;
	r.d = dir;
	bool rtrn = false;
	if (intersectLocal(r, i))
	{
		// Transform the intersection point & normal returned back into global space.
		i.N = transform->localToGlobalCoordsNormal(i.N);
                i.tangent = transform->localToGlobalCoordsNormal(i.tangent);
		i.t /= length;
		rtrn = true;
	}
	r.p = Wpos;
	r.d = Wdir;
	return rtrn;
}

bool Geometry::hasBoundingBoxCapability() const {
	// by default, primitives do not have to specify a bounding box.
	// If this method returns true for a primitive, then either the ComputeBoundingBox() or
    // the ComputeLocalBoundingBox() method must be implemented.

	// If no bounding box capability is supported for an object, that object will
	// be checked against every single ray drawn.  This should be avoided whenever possible,
	// but this possibility exists so that new primitives will not have to have bounding
	// boxes implemented for them.
	return false;
}

Scene::~Scene() {
  delete kdtree;
    giter g;
    liter l;
    tmap::iterator t;
    for( g = objects.begin(); g != objects.end(); ++g ) delete (*g);
    for( l = lights.begin(); l != lights.end(); ++l ) delete (*l);
    for( t = textureCache.begin(); t != textureCache.end(); t++ ) delete (*t).second;
}

// Get any intersection with an object.  Return information about the 
// intersection through the reference parameter.
bool Scene::intersect(ray& r, isect& i) const {
	
	bool have_one = false;

	//nonbounded objects
	typedef vector<Geometry*>::const_iterator iter;
	for(iter j = nonboundedobjects.begin(); j != nonboundedobjects.end(); ++j) {
		isect cur;
		if( (*j)->intersect(r, cur) ) {
			if(!have_one || (cur.t < i.t)) {
				i = cur;
				have_one = true;
			}
		}
	}

        if(!have_one){
                i.t = 1.0e308;
        }

        //bounded objects
        double tmin = 0.0;
	double tmax = 0.0;
        
	if(sceneBounds.intersect(r, tmin, tmax) && kdtree -> intersect(r, i, max(tmin, 0.0), tmax)){
           have_one = true;
	}

	if(!have_one) i.setT(1000.0);
	// if debugging,
	if (TraceUI::m_debug) intersectCache.push_back(std::make_pair(new ray(r), new isect(i)));
	return have_one;
}

TextureMap* Scene::getTexture(string name) {
	tmap::const_iterator itr = textureCache.find(name);
	if(itr == textureCache.end()) {
		textureCache[name] = new TextureMap(name);
		return textureCache[name];
	} else return (*itr).second;
}
int compareBoundary(const pair<int, double>& p1, const pair<int, double>& p2){
  return (p1.second) < (p2.second);
  
}

void Scene::buildAreaLight(RayTracer* rt){
  cout<<"building Layer Attenuation map"<<endl;
  //layeredAttenuation.clear();
  //sample area light position 
  //for each position, trace a ray
  //warping to the (x',y',z') in the center
  //construct layered map
  PointLight* l = (PointLight*) lights[0];
  Vec3d lightPos = l->position;
  center = camera;
  center.setEye(lightPos);

  cout<<"center: "<<lightPos<<endl;
  cout<<center.getU()<<endl;
  cout<<center.getV()<<endl;
  for(int i = 0; i<width; i++){
     for(int j = 0; j<height; j++){
        for(int k = 0; k<numSample; k++){
           layeredAttenuation[i][j][k] = -1.0;
        }
     }
  }

  //cout<<"building Layer Attenuation map"<<endl;
  //build LDI
  for(int i = 0; i<numSample; i++){
     Camera current = camera;
     Vec3d randL = lightPos + areaScale * (rand()*1.0/RAND_MAX * 2 - 1) * current.getU() +  areaScale * (rand()*1.0/RAND_MAX * 2 - 1) * current.getV();
     current.setEye(randL);
     //cout<<"sample: "<<randL<<endl;
     for(int m = 0; m<width; m++){
        for(int n = 0; n<height; n++){

              if (TraceUI::m_debug) intersectCache.clear();
              ray r(Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY);
              current.rayThrough((double) m, (double) n, r);
              //cout<<"position" << r.p<<" direction"<<r.d<<endl;
  
              isect is;
              is.t = -1.0;
              rt->traceRay(r, 0, is);
              if(is.t < 0.0001 || is.t > 500){
                  continue;
              }
              //cout<<is.t<<endl; always 1000?
              //warping tweak for planar area lights
              Vec3d p = r.at(is.t);
              Vec3d temp = (p - lightPos);
              double z = temp.length();
              temp = temp/(temp * center.getLook());
              double x = temp * center.getU();
              double y = temp * center.getV();
              int xc = ceil(x);
              int xf = floor(x);
              int yc = ceil(y);
              int yf = floor(y);
              
              if(xc >=0 && xc < width && yc >=0 && yc <height && layeredAttenuation[xc][yc][i] < z){
                  //cout<<"modifying: "<<xc<<" "<<yc<<" "<<z<<endl;
                  layeredAttenuation[xc][yc][i] = z;
              }
              if(xf >=0 && xf < width && yc >=0 && yc <height && layeredAttenuation[xf][yc][i] < z){
                  layeredAttenuation[xf][yc][i] = z;
              }
              if(xc >=0 && xc < width && yf >=0 && yf <height && layeredAttenuation[xc][yf][i] < z){
                  layeredAttenuation[xc][yf][i] = z;
              }
              if(xf >=0 && xf < width && yf >=0 && yf <height && layeredAttenuation[xf][yf][i] < z){
                  layeredAttenuation[xf][yf][i] = z;
              }
        }
     }
  }

}

double Scene::computeAreaAttenuation(double px, double py, double pz, RayTracer* rt){
  //cout<<"computing attenuation"<<endl;

  if(pz < 0.0001){
     return 0.0;
  }

  if (TraceUI::m_debug) intersectCache.clear();
  ray r(Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY);
  getCamera().rayThrough(px,py,r);
  Vec3d p = r.at(pz);

  Vec3d temp = (p - center.getEye());
  double z = temp.length();
  temp = temp/(temp * center.getLook());
  int x = round(temp * center.getU());
  int y = round(temp * center.getV());

  if(x < 0 || x >= width || y<0 || y>=height){

     return 0.0;
  } 
  double attenuation = 0.0;
  for(int k = 0; k< numSample; k++){
     //cout<<"z "<<z<<" attenuation: "<<layeredAttenuation[x][y][k];
     if(z < layeredAttenuation[x][y][k] + 0.0001){
         attenuation += 1.0/numSample;
     }
  }
  return attenuation;

}
void Scene::buildKdTree(){
  kdtree = new KdTree();
  kdtree->objects = boundedobjects;

  cout<<"building kd tree"<<endl;
  if(boundedobjects.size() <= (traceUI -> getKdMaxLeafSize()) || traceUI -> getKdMaxDepth() == 0){
    kdtree->leaf = true;
    return;
  }
  
  
  std::queue<KdTree*> decomposable;
  std::queue<BoundingBox> boundingboxes;
  std::queue<int> objectCounts;
  std::queue<int> levels;
  std::queue<vector<pair<int, double> > > orderedLeftBoundaryLists[3];
  std::queue<vector<pair<int, double> > > orderedRightBoundaryLists[3];

  
  decomposable.push(kdtree);
  boundingboxes.push(sceneBounds);
  objectCounts.push(boundedobjects.size());
  levels.push(0);

  vector<pair<int, double> >leftBoundaryOrder[3];
  vector<pair<int, double> >rightBoundaryOrder[3];

  for(int dim = 0; dim < 3; dim++){
    for(int i = 0; i < boundedobjects.size(); i++){
      pair<int, double> pleft(i, (boundedobjects[i]->getBoundingBox()).getMin()[dim]);
      leftBoundaryOrder[dim].push_back(pleft);
      pair<int, double> pright(i, (boundedobjects[i]->getBoundingBox()).getMax()[dim]);
      rightBoundaryOrder[dim].push_back(pright);
    }
    sort(leftBoundaryOrder[dim].begin(), leftBoundaryOrder[dim].end(), compareBoundary);
    sort(rightBoundaryOrder[dim].begin(), rightBoundaryOrder[dim].end(), compareBoundary);
    
    orderedLeftBoundaryLists[dim].push(leftBoundaryOrder[dim]);
    orderedRightBoundaryLists[dim].push(rightBoundaryOrder[dim]);
  }

  

  //on queue condition: objects already set, depth < maxdepth and leaves > maxleaves, can be not splitting though
  while(!decomposable.empty()){
    KdTree* node = decomposable.front();
    int objectCount = objectCounts.front();
    BoundingBox currentBox = boundingboxes.front();
    int nextLevel = levels.front() + 1;
    vector<pair <int, double> > currentOrderedLeftBoundaries[3];
    vector<pair <int, double> > currentOrderedRightBoundaries[3];

    
    for(int dim = 0; dim < 3; dim++){
      currentOrderedLeftBoundaries[dim] = orderedLeftBoundaryLists[dim].front();
      currentOrderedRightBoundaries[dim] = orderedRightBoundaryLists[dim].front();
      orderedLeftBoundaryLists[dim].pop();
      orderedRightBoundaryLists[dim].pop();
    }
    decomposable.pop();
    objectCounts.pop();
    boundingboxes.pop();
    levels.pop();
    
    double heuristic = objectCount * currentBox.area();//the cost when no splitting
    bool splitting = false;
    
    int bestSeparationDimension;
    bool bestSplitIsLeftBoundary;
    double bestSplitBoundary;
    
    int bestLeftCount;
    int bestRightCount;
    BoundingBox bestLeftBox;
    BoundingBox bestRightBox;
    //for each dimension

    for(int dim = 0; dim < 3; dim++){
      vector<pair<int,double> > orderedLeftBoundaryList = currentOrderedLeftBoundaries[dim];
      vector<pair<int,double> > orderedRightBoundaryList = currentOrderedRightBoundaries[dim];


      BoundingBox leftBox = currentBox;
      BoundingBox rightBox = currentBox;
      leftBox.setMax(dim, leftBox.getMin()[dim]);

      int na = 0;//need to set na correctly
      for(Geometry* g : node -> objects){
	if(g->getBoundingBox().getMin()[dim] < currentBox.getMin()[dim]){
	  na++;
	}
      }
      
      int nb = objectCount;

      int leftindex = 0;
      int rightindex = 0;
      int leftend = orderedLeftBoundaryList.size();
      int rightend = orderedRightBoundaryList.size();

      while(leftindex < leftend || rightindex < rightend){
        if(rightindex >= rightend || leftindex < leftend && orderedLeftBoundaryList[leftindex].second < orderedRightBoundaryList[rightindex].second){
          //modify bounding box, calculate heuristics
	  leftBox.setMax(dim, orderedLeftBoundaryList[leftindex].second);
	  rightBox.setMin(dim, orderedLeftBoundaryList[leftindex].second);
	  double temp = 1.0/80 + na * leftBox.area() + nb * rightBox.area();
	  if( temp < heuristic){
	    bestSeparationDimension = dim;
	    bestSplitIsLeftBoundary = true;
	    bestSplitBoundary = orderedLeftBoundaryList[leftindex].second;
	    bestLeftCount = na;
	    bestRightCount = nb;
	    bestLeftBox = leftBox;
	    bestRightBox = rightBox;
	    splitting = true;
	    heuristic = temp;
	  }
	  na++;
	  leftindex++;
	}
	else{
	  nb--;
           //modify bounding box, calculate heuristics
	  leftBox.setMax(dim, orderedRightBoundaryList[rightindex].second);
	  rightBox.setMin(dim, orderedRightBoundaryList[rightindex].second);
	  double temp = 1.0/80 + na * leftBox.area() + nb * rightBox.area();
	  if( temp < heuristic){
 	    bestSeparationDimension = dim;
	    bestSplitIsLeftBoundary = false;
	    bestSplitBoundary = orderedRightBoundaryList[rightindex].second;
	    bestLeftCount = na;
	    bestRightCount = nb;
	    bestLeftBox = leftBox;
	    bestRightBox = rightBox;
	    splitting = true;
	    heuristic = temp;
	  }
	  rightindex++;
	}
      }
      
    }
    if(!splitting){
      node -> leaf = true;
      continue;
    }
    //set current node 
    KdTree *leftChild = new KdTree();
    KdTree *rightChild = new KdTree();
    node -> left = leftChild;
    node -> right = rightChild;
    node -> separationDim = bestSeparationDimension;
    node -> separationBoundary = bestSplitBoundary;
    node -> leaf = false;

    //initializing two children and add information to queue
    while(!(node -> objects).empty()){
      Geometry* g = (node -> objects).back();
      (node -> objects).pop_back();
      double left = g->getBoundingBox().getMin()[bestSeparationDimension];
      double right = g->getBoundingBox().getMax()[bestSeparationDimension];
      if(left < bestSplitBoundary){
	(leftChild -> objects).push_back(g);
      }
      if(right > bestSplitBoundary){
	(rightChild -> objects).push_back(g);
      }
    }

    if(bestLeftCount <= traceUI -> getKdMaxLeafSize() || nextLevel > traceUI -> getKdMaxDepth()){
      leftChild -> leaf = true;
    }
    else{
      //push leftChild on queue
      decomposable.push(leftChild);
      boundingboxes.push(bestLeftBox);
      objectCounts.push(bestLeftCount);
      levels.push(nextLevel);
      for(int dim = 0; dim < 3; dim++){
	vector<pair<int, double> > leftChildLeftBoundaryList;
	vector<pair<int, double> > leftChildRightBoundaryList;
	if(dim == bestSeparationDimension){
	  for(pair<int, double> p: currentOrderedLeftBoundaries[dim]){
	    if(p.second < bestSplitBoundary){
	      leftChildLeftBoundaryList.push_back(p);
	    }
	  }
	  //
	  for(pair<int, double> p: currentOrderedRightBoundaries[dim]){
	    if(p.second < bestSplitBoundary){
	      leftChildRightBoundaryList.push_back(p);
	    }
	  }
	}
	else{
	  for(pair<int, double> p: currentOrderedLeftBoundaries[dim]){
	    BoundingBox temp = boundedobjects[p.first]-> getBoundingBox();
	    if(temp.getMin()[bestSeparationDimension] < bestSplitBoundary){
	      leftChildLeftBoundaryList.push_back(p);
	    }
	  }
	  for(pair<int, double> p: currentOrderedRightBoundaries[dim]){
	    BoundingBox temp = boundedobjects[p.first]-> getBoundingBox();
	    if(temp.getMin()[bestSeparationDimension] < bestSplitBoundary){
	      leftChildRightBoundaryList.push_back(p);
	    }
	  }
	}
	orderedLeftBoundaryLists[dim].push(leftChildLeftBoundaryList);
	orderedRightBoundaryLists[dim].push(leftChildRightBoundaryList);
      }
    }

    if(bestRightCount <= traceUI -> getKdMaxLeafSize() || nextLevel > traceUI -> getKdMaxDepth()){
      rightChild -> leaf = true;
    }
    else{
      decomposable.push(rightChild);
      boundingboxes.push(bestRightBox);
      objectCounts.push(bestRightCount);
      levels.push(nextLevel);

      for(int dim = 0; dim < 3; dim++){
	vector<pair<int, double> > rightChildLeftBoundaryList;
	vector<pair<int, double> > rightChildRightBoundaryList;
	if(dim == bestSeparationDimension){
	  for(pair<int, double> p: currentOrderedLeftBoundaries[dim]){
	    if(p.second > bestSplitBoundary){
	      rightChildLeftBoundaryList.push_back(p);
	    }
	  }
	  //
	  for(pair<int, double> p: currentOrderedRightBoundaries[dim]){
	    if(p.second > bestSplitBoundary){
	      rightChildRightBoundaryList.push_back(p);
	    }
	  }
	}
	else{
	  for(pair<int, double> p: currentOrderedLeftBoundaries[dim]){
	    BoundingBox temp = boundedobjects[p.first]-> getBoundingBox();
	    if(temp.getMax()[bestSeparationDimension] > bestSplitBoundary){
	      rightChildLeftBoundaryList.push_back(p);
	    }
	  }
	  for(pair<int, double> p: currentOrderedRightBoundaries[dim]){
	    BoundingBox temp = boundedobjects[p.first]-> getBoundingBox();
	    if(temp.getMax()[bestSeparationDimension] > bestSplitBoundary){
	      rightChildRightBoundaryList.push_back(p);
	    }
	  }
	}
	orderedLeftBoundaryLists[dim].push(rightChildLeftBoundaryList);
	orderedRightBoundaryLists[dim].push(rightChildRightBoundaryList);
      }
    }
    
  }

}


