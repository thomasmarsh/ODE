// main window and event handling for X11

#include <stdio.h>
#include <stdarg.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <GL/glx.h>

#include "drawstuff/drawstuff.h"
#include "drawstuff/version.h"
#include "internal.h"

//***************************************************************************
// error handling for unix

static void printMessage (char *msg1, char *msg2, va_list ap)
{
  fflush (stderr);
  fflush (stdout);
  fprintf (stderr,"\n%s: ",msg1);
  vfprintf (stderr,msg2,ap);
  fprintf (stderr,"\n");
  fflush (stderr);
}


extern "C" void dsError (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  printMessage ("Error",msg,ap);
  exit (1);
}


extern "C" void dsDebug (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  printMessage ("INTERNAL ERROR",msg,ap);
  // *((char *)0) = 0;	 ... commit SEGVicide ?
  abort();
}


extern "C" void dsPrint (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  vprintf (msg,ap);
}

//***************************************************************************
// openGL window

// X11 display info
static Display *display=0;
static int screen=0;
static XVisualInfo *visual=0;		// best visual for openGL
static Colormap colormap=0;		// window's colormap
static Atom wm_protocols_atom = 0;
static Atom wm_delete_window_atom = 0;

// window and openGL
static Window win=0;			// X11 window, 0 if not initialized
static int width=0,height=0;		// window size
static GLXContext glx_context=0;	// openGL rendering context
static int last_key_pressed=0;		// last key pressed in the window
static int run=1;			// 1 if simulation running
static int pause=0;			// 1 if in `pause' mode
static int singlestep=0;		// 1 if single step key pressed


static void createMainWindow (int _width, int _height)
{
  // create X11 display connection
  display = XOpenDisplay (0);
  if (!display) dsError (0,"can not open X11 display");
  screen = DefaultScreen(display);

  // get GL visual
  static int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE,16,
			     GLX_RED_SIZE,4, GLX_GREEN_SIZE,4,
			     GLX_BLUE_SIZE,4, None};
  visual = glXChooseVisual (display,screen,attribList);
  if (!visual) dsError ("no good X11 visual found for OpenGL");

  // create colormap
  colormap = XCreateColormap (display,RootWindow(display,screen),
			      visual->visual,AllocNone);

  // initialize variables
  win = 0;
  width = _width;
  height = _height;
  glx_context = 0;
  last_key_pressed = 0;

  if (width < 1 || height < 1) dsDebug (0,"bad window width or height");

  // create the window
  XSetWindowAttributes attributes;
  attributes.background_pixel = BlackPixel(display,screen);
  attributes.colormap = colormap;
  attributes.event_mask = ButtonPressMask | ButtonReleaseMask |
    KeyPressMask | KeyReleaseMask | ButtonMotionMask | PointerMotionHintMask |
    StructureNotifyMask;
  win = XCreateWindow (display,RootWindow(display,screen),50,50,width,height,
		       0,visual->depth, InputOutput,visual->visual,
		       CWBackPixel | CWColormap | CWEventMask,&attributes);

  // associate a GLX context with the window
  glx_context = glXCreateContext (display,visual,0,GL_TRUE);
  if (!glx_context) dsError ("can't make an OpenGL context");

  // set the window title
  XTextProperty window_name;
  window_name.value = (unsigned char *) "Simulation";
  window_name.encoding = XA_STRING;
  window_name.format = 8;
  window_name.nitems = strlen((char *) window_name.value);
  XSetWMName (display,win,&window_name);

  // participate in the window manager 'delete yourself' protocol
  wm_protocols_atom = XInternAtom (display,"WM_PROTOCOLS",False);
  wm_delete_window_atom = XInternAtom (display,"WM_DELETE_WINDOW",False);
  if (XSetWMProtocols (display,win,&wm_delete_window_atom,1)==0)
    dsError ("XSetWMProtocols() call failed");

  // pop up the window
  XMapWindow (display,win);
  XSync (display,win);
}


static void destroyMainWindow()
{
  glXDestroyContext (display,glx_context);
  XDestroyWindow (display,win);
  XSync (display,0);
  display = 0;
  win = 0;
  glx_context = 0;
}


static void handleEvent (XEvent &event, dsFunctions *fn)
{
  static int mx=0,my=0; 	// mouse position
  static int mode = 0;		// mouse button bits

  switch (event.type) {

  case ButtonPress: {
    if (event.xbutton.button == Button1) mode |= 1;
    if (event.xbutton.button == Button2) mode |= 2;
    if (event.xbutton.button == Button3) mode |= 4;
    mx = event.xbutton.x;
    my = event.xbutton.y;
  }
  return;

  case ButtonRelease: {
    if (event.xbutton.button == Button1) mode &= (~1);
    if (event.xbutton.button == Button2) mode &= (~2);
    if (event.xbutton.button == Button3) mode &= (~4);
    mx = event.xbutton.x;
    my = event.xbutton.x;
  }
  return;

  case MotionNotify: {
    if (event.xmotion.is_hint) {
      Window root,child;
      unsigned int mask;
      XQueryPointer (display,win,&root,&child,&event.xbutton.x_root,
		     &event.xbutton.y_root,&event.xbutton.x,&event.xbutton.y,
		     &mask);
    }
    dsMotion (mode, event.xmotion.x - mx, event.xmotion.y - my);
    mx = event.xmotion.x;
    my = event.xmotion.y;
  }
  return;

  case KeyPress: {
    KeySym key;
    XLookupString (&event.xkey,NULL,0,&key,0);
    if ( (event.xkey.state & (ShiftMask | ControlMask)) == 0) {
      if (key >= ' ' && key <= 126) fn->command (key);
    }
    else if (event.xkey.state & ControlMask) {
      switch (key) {
      case 't': case 'T':
	dsSetTextures (dsGetTextures() ^ 1);
	break;
      case 's': case 'S':
	dsSetShadows (dsGetShadows() ^ 1);
	break;
      case 'x': case 'X':
	run = 0;
	break;
      case 'p': case 'P':
	pause ^= 1;
	singlestep = 0;
	break;
      case 'o': case 'O':
	if (pause) singlestep = 1;
	break;
      case 'v': case 'V': {
	float xyz[3],hpr[3];
	dsGetViewpoint (xyz,hpr);
	printf ("Viewpoint = (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\n",
		xyz[0],xyz[1],xyz[2],hpr[0],hpr[1],hpr[2]);
	break;
      }
      }
    }
    last_key_pressed = key;		// a kludgy place to put this...
  }
  return;

  case KeyRelease: {
    // hmmmm...
  }
  return;

  case ClientMessage:
    if (event.xclient.message_type == wm_protocols_atom &&
	event.xclient.format == 32 &&
	Atom(event.xclient.data.l[0]) == wm_delete_window_atom) {
      run = 0;
      return;
    }
    return;

  case ConfigureNotify:
    width = event.xconfigure.width;
    height = event.xconfigure.height;
    return;
  }
}


void dsPlatformSimLoop (int window_width, int window_height, dsFunctions *fn)
{
  createMainWindow (window_width, window_height);
  glXMakeCurrent (display,win,glx_context);

  dsStartGraphics (window_width,window_height,fn);
  if (fn->start) fn->start();

  fprintf (stderr,
	   "\n"
	   "Simulation test environment v%d.%02d\n"
	   "   Ctrl-P : pause / unpause.\n"
	   "   Ctrl-O : single step when paused.\n"
	   "   Ctrl-T : toggle textures (or say `-notex' on command line).\n"
	   "   Ctrl-S : toggle shadows (or say `-noshadow' on command line).\n"
	   "   Ctrl-V : print current viewpoint coordinates (x,y,z,h,p,r).\n"
	   "   Ctrl-X : exit.\n"
	   "\n",DS_VERSION >> 8,DS_VERSION & 0xff);

  run = 1;
  while (run) {
    // read in and process all pending events for the main window
    XEvent event;
    while (run && XPending (display)) {
      XNextEvent (display,&event);
      handleEvent (event,fn);
    }

    dsDrawFrame (width,height,fn,pause && !singlestep);
    singlestep = 0;

    glFlush();
    glXSwapBuffers (display,win);
    XSync (display,0);
  };

  if (fn->stop) fn->stop();
  dsStopGraphics();

  destroyMainWindow();
}
