//
// GraphicalUI.cpp
//
// Handles FLTK integration and other user interface tasks
//
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdarg.h>
#include <thread>
#include <queue>
#ifndef COMMAND_LINE_ONLY

#include <FL/fl_ask.H>
#include "debuggingView.h"

#include "GraphicalUI.h"
#include "../RayTracer.h"
#include "../scene/cubeMap.h"

#define MAX_INTERVAL 500

#ifdef _WIN32
#define print sprintf_s
#else
#define print sprintf
#endif

bool GraphicalUI::stopTrace = false;
bool GraphicalUI::doneTrace = true;
GraphicalUI* GraphicalUI::pUI = NULL;
char* GraphicalUI::traceWindowLabel = "Raytraced Image";
bool TraceUI::m_debug = false;

//------------------------------------- Help Functions --------------------------------------------
GraphicalUI* GraphicalUI::whoami(Fl_Menu_* o)	// from menu item back to UI itself
{
	return ((GraphicalUI*)(o->parent()->user_data()));
}

//--------------------------------- Callback Functions --------------------------------------------
void GraphicalUI::cb_load_scene(Fl_Menu_* o, void* v) 
{
	pUI = whoami(o);

	static char* lastFile = 0;
	char* newfile = fl_file_chooser("Open Scene?", "*.ray", NULL );

	if (newfile != NULL) {
		char buf[256];

		if (pUI->raytracer->loadScene(newfile)) {
			print(buf, "Ray <%s>", newfile);
			stopTracing();	// terminate the previous rendering
		} else print(buf, "Ray <Not Loaded>");

		pUI->m_mainWindow->label(buf);
		pUI->m_debuggingWindow->m_debuggingView->setDirty();

		if( lastFile != 0 && strcmp(newfile, lastFile) != 0 )
			pUI->m_debuggingWindow->m_debuggingView->resetCamera();

		pUI->m_debuggingWindow->redraw();
	}
}

void GraphicalUI::cb_choose_cubemap(Fl_Menu_* o, void* v) {
        pUI = whoami(o);
        (pUI -> m_cubeMapChooser) -> show();
}

void GraphicalUI::cb_save_image(Fl_Menu_* o, void* v) 
{
	pUI = whoami(o);

	char* savefile = fl_file_chooser("Save Image?", "*.bmp", "save.bmp" );
	if (savefile != NULL) {
		pUI->m_traceGlWindow->saveImage(savefile);
	}
}

void GraphicalUI::cb_exit(Fl_Menu_* o, void* v)
{
	pUI = whoami(o);

	// terminate the rendering
	stopTracing();

	pUI->m_traceGlWindow->hide();
	pUI->m_mainWindow->hide();
	pUI->m_debuggingWindow->hide();
	TraceUI::m_debug = false;
}

void GraphicalUI::cb_exit2(Fl_Widget* o, void* v) 
{
	pUI = (GraphicalUI *)(o->user_data());

	// terminate the rendering
	stopTracing();

	pUI->m_traceGlWindow->hide();
	pUI->m_mainWindow->hide();
	pUI->m_debuggingWindow->hide();
	TraceUI::m_debug = false;
}

void GraphicalUI::cb_about(Fl_Menu_* o, void* v) 
{
	fl_message("RayTracer Project for CS384g.");
}

void GraphicalUI::cb_sizeSlides(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());

	// terminate the rendering so we don't get crashes
	stopTracing();

	pUI->m_nSize=int(((Fl_Slider *)o)->value());
	int width = (int)(pUI->getSize());
	int height = (int)(width / pUI->raytracer->aspectRatio() + 0.5);
	pUI->m_traceGlWindow->resizeWindow(width, height);
}

void GraphicalUI::cb_depthSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nDepth=int( ((Fl_Slider *)o)->value() ) ;
}

void GraphicalUI::cb_threadSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nThreads = int ( ((Fl_Slider *)o)->value() ) ;
}

void GraphicalUI::cb_supersamplingSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nSupersamplingPixels = int ( ((Fl_Slider *)o)->value() ) ;
}

void GraphicalUI::cb_filterWidth(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nFilterWidth = int ( ((Fl_Slider *)o)->value() ) ;
}

void GraphicalUI::cb_refreshSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->refreshInterval=clock_t(((Fl_Slider *)o)->value()) ;
}

void GraphicalUI::cb_debuggingDisplayCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_displayDebuggingInfo = (((Fl_Check_Button*)o)->value() == 1);
	if (pUI->m_displayDebuggingInfo)
	  {
	    pUI->m_debuggingWindow->show();
	    pUI->m_debug = true;
	  }
	else
	  {
	    pUI->m_debuggingWindow->hide();
	    pUI->m_debug = false;
	  }
}

void GraphicalUI::cb_checkantialiasing(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_antialiasing = (((Fl_Check_Button*)o)->value() == 1);
	if (pUI->m_antialiasing)
	  {
	    pUI->m_supersamplingSlider->activate();
	  }
	else
	  {
	    pUI->m_supersamplingSlider->deactivate();
	  }
}

void GraphicalUI::cb_checkcubemap(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_usingCubeMap = (((Fl_Check_Button*)o)->value() == 1);
	if (pUI->m_usingCubeMap)
	  {
	    (pUI -> m_filterSlider) -> activate();
	  }
	else
	  {
	    (pUI -> m_filterSlider) -> deactivate();
	  }
}

void GraphicalUI::renderPixels(RayTracer * raytracer, const int& startx, const int& starty, const int& endx, const int& endy, const int& width, const int& height, const int& supersamplePixels){
         if(starty > endy){
              return;
         }
         
         int x = startx;
         int y = starty;
         while(y < endy){
             raytracer -> tracePixel(x, y, supersamplePixels);
             x++;
             if(x == width){
                 x = 0;
                 y++;
             }
         }
         while(x <= endx){
             raytracer -> tracePixel(x, y, supersamplePixels);
             x++;
         }

}
void GraphicalUI::cb_render(Fl_Widget* o, void* v) {

	char buffer[256];

	pUI = (GraphicalUI*)(o->user_data());
        int supersamplePixels = pUI->m_antialiasing?(pUI -> m_nSupersamplingPixels):1;
        
	doneTrace = stopTrace = false;
	if (pUI->raytracer->sceneLoaded())
	  {
		int width = pUI->getSize();
		int height = (int)(width / pUI->raytracer->aspectRatio() + 0.5);
		int origPixels = width * height;
		pUI->m_traceGlWindow->resizeWindow(width, height);
		pUI->m_traceGlWindow->show();
		pUI->raytracer->traceSetup(width, height);

		// Save the window label
                const char *old_label = pUI->m_traceGlWindow->label();

		clock_t now, prev;
		now = prev = clock();
		clock_t intervalMS = pUI->refreshInterval * 100;


                //multithreading
                std::queue<std::thread> threads; 
                int rowsPerThread = height/pUI->m_nThreads;
                int leftCount = height%pUI->m_nThreads;
                int prevrow = 0;
                int nextrow = rowsPerThread - 1 + (leftCount-- > 0 ? 1:0);
                int endx = 0;//endx and endy marks the last pixel that's ready for displaying
                int endy = -1;
                


		for (int y = 0; y < height; y++)
		  {

		    for (int x = 0; x < width; x++)
		      {
                        
			if (stopTrace){
                            threads.emplace(renderPixels, pUI->raytracer, 0, prevrow , endx , endy, width, height, supersamplePixels);
                            break;
                        }
			// check for input and refresh view every so often while tracing
			now = clock();
			if ((now - prev)/CLOCKS_PER_SEC * 1000 >= intervalMS)
			  {
			    prev = now;
			    sprintf(buffer, "(%d%%) %s", (int)((double)y / (double)height * 100.0), old_label);
			    pUI->m_traceGlWindow->label(buffer);
			    pUI->m_traceGlWindow->refresh();
			    Fl::check();
			    if (Fl::damage()) { Fl::flush(); }
			  }
			// look for input and refresh window
			//pUI->raytracer->tracePixel(x, y);
                        endx = x;
                        endy = y;

			pUI->m_debuggingWindow->m_debuggingView->setDirty();
		      }
		    if (stopTrace){
                            threads.emplace(renderPixels, pUI->raytracer, 0, prevrow, endx , endy, width, height, supersamplePixels);
                            break;
                    }
                    if(y == nextrow){
                       threads.emplace(renderPixels, pUI->raytracer, 0, prevrow , endx, endy, width, height, supersamplePixels);
                       if(y < height - 1){
                             prevrow = nextrow + 1;
                             nextrow = nextrow + rowsPerThread + (leftCount-- > 0 ? 1:0);
                             

                       }
                    }
		  }
                
                while(!threads.empty()){
      		       threads.front().join();
      		       threads.pop();    
    		}
 
		doneTrace = true;
		stopTrace = false;
		// Restore the window label
		pUI->m_traceGlWindow->label(old_label);
		pUI->m_traceGlWindow->refresh();
	  }
}

void GraphicalUI::cb_stop(Fl_Widget* o, void* v)
{
	pUI = (GraphicalUI*)(o->user_data());
	stopTracing();
}

int GraphicalUI::run()
{



	Fl::visual(FL_DOUBLE|FL_INDEX);

	m_mainWindow->show();

	return Fl::run();
}

void GraphicalUI::alert( const string& msg )
{
	fl_alert( "%s", msg.c_str() );
}

void GraphicalUI::setRayTracer(RayTracer *tracer)
{
	TraceUI::setRayTracer(tracer);
	m_traceGlWindow->setRayTracer(tracer);
	m_debuggingWindow->m_debuggingView->setRayTracer(tracer);
}

// menu definition
Fl_Menu_Item GraphicalUI::menuitems[] = {
	{ "&File", 0, 0, 0, FL_SUBMENU },
	{ "&Load Scene...",	FL_ALT + 'l', (Fl_Callback *)GraphicalUI::cb_load_scene },
	{ "&Load CubeMap...",	FL_ALT + 'l', (Fl_Callback *)GraphicalUI::cb_choose_cubemap },
	{ "&Save Image...", FL_ALT + 's', (Fl_Callback *)GraphicalUI::cb_save_image },
	{ "&Exit", FL_ALT + 'e', (Fl_Callback *)GraphicalUI::cb_exit },
	{ 0 },

	{ "&Help",		0, 0, 0, FL_SUBMENU },
	{ "&About",	FL_ALT + 'a', (Fl_Callback *)GraphicalUI::cb_about },
	{ 0 },

	{ 0 }
};

void GraphicalUI::stopTracing()
{
	stopTrace = true;
}

GraphicalUI::GraphicalUI() : refreshInterval(10) {
        
        
        
	// init.
	m_mainWindow = new Fl_Window(100, 40, 450, 459, "Ray <Not Loaded>");
	m_mainWindow->user_data((void*)(this));	// record self to be used by static callback functions
	// install menu bar
	m_menubar = new Fl_Menu_Bar(0, 0, 440, 25);
	m_menubar->menu(menuitems);
        
	
	// set up "render" button
	m_renderButton = new Fl_Button(360, 37, 70, 25, "&Render");
	m_renderButton->user_data((void*)(this));
	m_renderButton->callback(cb_render);

	// set up "stop" button
	m_stopButton = new Fl_Button(360, 65, 70, 25, "&Stop");
	m_stopButton->user_data((void*)(this));
	m_stopButton->callback(cb_stop);

	// install depth slider
	m_depthSlider = new Fl_Value_Slider(10, 40, 180, 20, "Recursion Depth");
	m_depthSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_depthSlider->type(FL_HOR_NICE_SLIDER);
	m_depthSlider->labelfont(FL_COURIER);
	m_depthSlider->labelsize(12);
	m_depthSlider->minimum(0);
	m_depthSlider->maximum(10);
	m_depthSlider->step(1);
	m_depthSlider->value(m_nDepth);
	m_depthSlider->align(FL_ALIGN_RIGHT);
	m_depthSlider->callback(cb_depthSlides);

	// install size slider
	m_sizeSlider = new Fl_Value_Slider(10, 65, 180, 20, "Screen Size");
	m_sizeSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_sizeSlider->type(FL_HOR_NICE_SLIDER);
	m_sizeSlider->labelfont(FL_COURIER);
	m_sizeSlider->labelsize(12);
	m_sizeSlider->minimum(64);
	m_sizeSlider->maximum(1024);
	m_sizeSlider->step(2);
	m_sizeSlider->value(m_nSize);
	m_sizeSlider->align(FL_ALIGN_RIGHT);
	m_sizeSlider->callback(cb_sizeSlides);

	// install refresh interval slider
	m_refreshSlider = new Fl_Value_Slider(10, 90, 180, 20, "Screen Refresh Interval (0.1 sec)");
	m_refreshSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_refreshSlider->type(FL_HOR_NICE_SLIDER);
	m_refreshSlider->labelfont(FL_COURIER);
	m_refreshSlider->labelsize(12);
	m_refreshSlider->minimum(1);
	m_refreshSlider->maximum(300);
	m_refreshSlider->step(1);
	m_refreshSlider->value(refreshInterval);
	m_refreshSlider->align(FL_ALIGN_RIGHT);
	m_refreshSlider->callback(cb_refreshSlides);

	//install multi-threads slider
        m_nThreads = (int) std::thread::hardware_concurrency();
	m_threadSlider = new Fl_Value_Slider(10, 115, 180, 20, "threads");
	m_threadSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_threadSlider->type(FL_HOR_NICE_SLIDER);
	m_threadSlider->labelfont(FL_COURIER);
	m_threadSlider->labelsize(12);
	m_threadSlider->minimum(1);
	m_threadSlider->maximum(m_nThreads);
	m_threadSlider->step(1);
	m_threadSlider->value(m_nThreads);
	m_threadSlider->align(FL_ALIGN_RIGHT);
	m_threadSlider->callback(cb_threadSlides);

        
        //anti aliasing checkbox
        m_antialiasing = false;
        m_antialiasingButton = new Fl_Check_Button(10, 140, 140, 20, "antialiasing");
	m_antialiasingButton->user_data((void*)(this));
	m_antialiasingButton->callback(cb_checkantialiasing);
	m_antialiasingButton->value(m_antialiasing);

        //anti aliasing supersampling pixel samples
        m_nSupersamplingPixels = 1;
	m_supersamplingSlider = new Fl_Value_Slider(150, 140, 180, 20, "supersampling pixels");
	m_supersamplingSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_supersamplingSlider->type(FL_HOR_NICE_SLIDER);
	m_supersamplingSlider->labelfont(FL_COURIER);
	m_supersamplingSlider->labelsize(12);
	m_supersamplingSlider->minimum(1);
	m_supersamplingSlider->maximum(4);
	m_supersamplingSlider->step(1);
	m_supersamplingSlider->value(m_nSupersamplingPixels);
	m_supersamplingSlider->align(FL_ALIGN_RIGHT);
	m_supersamplingSlider->callback(cb_supersamplingSlides);
        if(!m_antialiasing){ 
              m_supersamplingSlider->deactivate();
        }

        
	//cubemap checkbox
        setCubeMap(false);
	useCubeMap(false);
	m_cubeMapCheckButton = new Fl_Check_Button(10, 165, 140, 20, "using cubemap");
	m_cubeMapCheckButton->user_data((void*)(this));
        m_cubeMapCheckButton->callback(cb_checkcubemap);
        m_cubeMapCheckButton->value(m_usingCubeMap);
	m_cubeMapCheckButton->deactivate();
	
        //cubemap filterslider
        m_nFilterWidth = 1;
	m_filterSlider = new Fl_Value_Slider(150, 165, 180, 20, "cubemap filter size");
        m_filterSlider->user_data((void*)(this));	// record self to be used by static callback functions
        m_filterSlider->type(FL_HOR_NICE_SLIDER);
        m_filterSlider->labelfont(FL_COURIER);
        m_filterSlider->labelsize(12);
        m_filterSlider->minimum(1);
        m_filterSlider->maximum(17);
        m_filterSlider->step(1);
        m_filterSlider->value(m_nFilterWidth);
        m_filterSlider->align(FL_ALIGN_RIGHT);
        m_filterSlider->callback(cb_filterWidth);
        m_filterSlider->deactivate();

        
	// set up debugging display checkbox
	m_debuggingDisplayCheckButton = new Fl_Check_Button(10, 429, 140, 20, "Debugging display");
	m_debuggingDisplayCheckButton->user_data((void*)(this));
	m_debuggingDisplayCheckButton->callback(cb_debuggingDisplayCheckButton);
	m_debuggingDisplayCheckButton->value(m_displayDebuggingInfo);

	m_mainWindow->callback(cb_exit2);
	m_mainWindow->when(FL_HIDE);
	m_mainWindow->end();

	// image view
	m_traceGlWindow = new TraceGLWindow(100, 150, m_nSize, m_nSize, traceWindowLabel);
	m_traceGlWindow->end();
	m_traceGlWindow->resizable(m_traceGlWindow);

	// debugging view
	m_debuggingWindow = new DebuggingWindow();

        //initialize cubemapchoose
	m_cubeMapChooser = new CubeMapChooser();
	m_cubeMapChooser->setCaller(this);
}

#endif
