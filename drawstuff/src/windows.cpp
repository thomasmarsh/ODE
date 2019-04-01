/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#if defined(WIN32) || defined(__CYGWIN__)// this prevents warnings when dependencies built
#include <windows.h>
#endif
#include <process.h>
#include <ode/odeconfig.h>
#include <GL/gl.h>

#include "config.h"
#include "common.h"
#include "resource.h"
#include "internal.h"

#include <tchar.h>
#include <assert.h>


 //***************************************************************************
 // application globals

static HINSTANCE g_instance = NULL;
static HACCEL g_accelerators = NULL;
static int g_cmdShow = 0;


//***************************************************************************
// error and message handling

static void errorBox(const char *title, const char *msg, va_list ap)
{
    char s[1000];
    vsprintf(s, msg, ap);
    MessageBox(0, s, title, MB_OK | MB_APPLMODAL | MB_ICONEXCLAMATION);
}


static void dsWarning(const char *msg, ...)
{
    va_list ap;
    va_start(ap, msg);
    errorBox("Warning", msg, ap);
    va_end(ap);
}


extern "C" void dsError(const char *msg, ...)
{
    va_list ap;
    va_start(ap, msg);
    errorBox("Error", msg, ap);
    va_end(ap);
    exit(1);
}


extern "C" void dsDebug(const char *msg, ...)
{
    va_list ap;
    va_start(ap, msg);
    errorBox("INTERNAL ERROR", msg, ap);
    va_end(ap);
    // *((char *)0) = 0;	 ... commit SEGVicide ?
    abort();
    exit(1);	  // should never get here, but just in case...
}


extern "C" void dsPrint(const char *msg, ...)
{
    va_list ap;
    va_start(ap, msg);
    vprintf(msg, ap);
    va_end(ap);
}

//***************************************************************************
// rendering thread

// globals used to communicate with rendering thread

struct RenderingThreadParams
{
    RenderingThreadParams(bool initialPause, HDC rendererDC, int rendererWidth, int rendererHeight, dsFunctions *rendererFunctions):
        m_rendererExitRequest(false),
        m_rendererPause(initialPause),
        m_rendererSingleStep(false),
        m_rendererWidth(rendererWidth),
        m_rendererHeight(rendererHeight),
        m_rendererFn(rendererFunctions),
        m_rendererDC(rendererDC),
        m_keyBufferHead(0),
        m_keyBufferTail(0)
    {
    }

    void pushBufferedChar(int charCode)
    {
        int nextHead = (m_keyBufferHead + 1) % dARRAY_SIZE(m_keyBufferStorage);
        if (nextHead != m_keyBufferTail) {
            m_keyBufferStorage[m_keyBufferHead] = charCode;
            m_keyBufferHead = nextHead;
        }
    }

    void popBufferedChar()
    {
        assert(areAnyBufferedChars());

        m_keyBufferTail = (m_keyBufferTail + 1) % dARRAY_SIZE(m_keyBufferStorage);
    }

    int peekBufferedChar() const 
    {
        assert(areAnyBufferedChars());

        return m_keyBufferStorage[m_keyBufferTail];
    }

    bool areAnyBufferedChars() const
    {
        return m_keyBufferHead != m_keyBufferTail;
    }

    bool m_rendererExitRequest;
    bool m_rendererPause;	  // 0=run, 1=pause
    bool m_rendererSingleStep;	  // single step command
    int m_rendererWidth;
    int m_rendererHeight;
    dsFunctions *m_rendererFn;
    HDC m_rendererDC;
    int m_keyBufferHead;	  // index of next key to put in (modified by GUI)
    int m_keyBufferTail;	  // index of next key to take out (modified by renderer)
    int m_keyBufferStorage[16];	  // fifo ring buffer for keypresses
};


static void performThreadedRendering(RenderingThreadParams *const pRendererParams);

static 
unsigned CALLBACK renderingThread(LPVOID lpParam)
{
    RenderingThreadParams *const pRendererParams = (RenderingThreadParams *)lpParam;

    bool executionSucceeded = false;

    HGLRC glc = NULL;
    bool contextCreated = false, currentMade = false;

    do {
        // create openGL context and make it current
        glc = wglCreateContext(pRendererParams->m_rendererDC);
        if (glc == NULL) {
            dsError("could not create OpenGL context");
            break;
        }
        contextCreated = true;

        if (!wglMakeCurrent(pRendererParams->m_rendererDC, glc)) {
            dsError("could not make OpenGL context current");
            break;
        }
        currentMade = true;

        // test openGL capabilities
        int maxTextureSize = 0;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
        if (maxTextureSize < 128) {
            dsWarning("Maximal texture size is too small (%dx%d)", maxTextureSize, maxTextureSize);
            break;
        }

        performThreadedRendering(pRendererParams);

        // delete openGL context
        wglMakeCurrent(NULL, NULL);
        wglDeleteContext(glc);

        executionSucceeded = true;
    }
    while (false);

    if (!executionSucceeded) {
        if (contextCreated) {
            if (currentMade) {
                wglMakeCurrent(NULL, NULL);
            }

            wglDeleteContext(glc);
        }
    }

    return executionSucceeded;
}

static 
void performThreadedRendering(RenderingThreadParams *const pRendererParams)
{
    dsFunctions *const rendererFn = pRendererParams->m_rendererFn;

    dsStartGraphics(pRendererParams->m_rendererWidth, pRendererParams->m_rendererHeight, rendererFn);

    if (rendererFn->start != NULL) {
        rendererFn->start();
    }

    while (!pRendererParams->m_rendererExitRequest) {
        // need to make local copy of renderer_ss to help prevent races
        bool singleStep = pRendererParams->m_rendererSingleStep;
        dsDrawFrame(pRendererParams->m_rendererWidth, pRendererParams->m_rendererHeight, rendererFn, pRendererParams->m_rendererPause && !singleStep);
        if (singleStep) { 
            pRendererParams->m_rendererSingleStep = false;
        }

        // read keys out of ring buffer and feed them to the command function
        for (; pRendererParams->areAnyBufferedChars(); pRendererParams->popBufferedChar()) {
            if (rendererFn->command != NULL) {
                int nextBufferedChar = pRendererParams->peekBufferedChar();
                rendererFn->command(nextBufferedChar);
            }
        }

        // swap buffers
        SwapBuffers(pRendererParams->m_rendererDC);
    }

    if (rendererFn->stop != NULL) {
        rendererFn->stop();
    }

    dsStopGraphics();
}


//***************************************************************************
// MainWindowExternalAborter

class MainWindowExternalAborter
{
public:
    static void registerForAborts()
    {
        MSG msg;
        // Remove old WM_QUIT message that might remain in the thread's queue
        while (PeekMessage(&msg, (HWND)(-1), WM_QUIT, WM_QUIT, PM_REMOVE | PM_NOYIELD)) {}

        m_mainWindowThreadID = GetCurrentThreadId();
    }

    static void unregisterFromAborts()
    {
        m_mainWindowThreadID = 0;
    }

    static bool requestAbort()
    {
        bool fault = false;

        DWORD mainWindowThreadID = m_mainWindowThreadID;
        if (mainWindowThreadID != 0) {
            m_mainWindowThreadID = 0;

            if (!PostThreadMessage(mainWindowThreadID, WM_QUIT, 0, 0)) {
                fault = true;
            }
        }

        bool result = !fault;
        return result;
    }

    static volatile DWORD m_mainWindowThreadID;
};

/*static */volatile DWORD MainWindowExternalAborter::m_mainWindowThreadID = 0;


//***************************************************************************
// window handling


struct MainWindowParameters
{
    MainWindowParameters():
        m_mouseButtonStates(0),
        m_lastMouseX(0),
        m_lastMouseY(0),
        m_hideTextures(0),
        m_hideShadows(0)
    {
    }

    enum
    {
        MBS_LBUTTONDOWN = dsMOTIONMODE_LBUTTONDOWN,
        MBS_MBUTTONDOWN = dsMOTIONMODE_MBUTTONDOWN,
        MBS_RBUTTONDOWN = dsMOTIONMODE_RBUTTONDOWN,
    };

    unsigned m_mouseButtonStates;
    int m_lastMouseX;
    int m_lastMouseY;
    bool m_hideTextures;
    bool m_hideShadows;
};

enum EMAINWINDOWWINDOWEXTRA
{
    MWE__MIN,

    MWE_RENDERER_PARAM_PTR = MWE__MIN,
    MWE__RENDERER_PARAM_PTR_END = MWE_RENDERER_PARAM_PTR + sizeof(LONG_PTR), // RenderingThreadParams *

    MWE_WINDOW_PARAM_PTR = MWE__RENDERER_PARAM_PTR_END,
    MWE__WINDOW_PARAM_PTR_END = MWE_WINDOW_PARAM_PTR + sizeof(LONG_PTR), // MainWindowParameters *

    _MWE__MAX,
    MWE__MAX = _MWE__MAX - 1,
};
dSASSERT(sizeof(LONG_PTR) >= sizeof(RenderingThreadParams *));
dSASSERT(sizeof(LONG_PTR) >= sizeof(MainWindowParameters *));

static inline 
void assignMainWindowRendererParams(HWND mainWindow, RenderingThreadParams *pRendererParams)
{
    SetWindowLongPtr(mainWindow, MWE_RENDERER_PARAM_PTR, (LONG_PTR)pRendererParams);
    dSASSERT(sizeof(LONG_PTR) >= sizeof(RenderingThreadParams *));
}

static inline 
RenderingThreadParams *retrieveMainWindowRendererThreadParams(HWND mainWindow)
{
    dSASSERT(sizeof(LONG_PTR) >= sizeof(RenderingThreadParams *));

    return (RenderingThreadParams *)GetWindowLongPtr(mainWindow, MWE_RENDERER_PARAM_PTR);
}

static inline 
void assignMainWindowWindowParameters(HWND mainWindow, MainWindowParameters *pWindowParameters)
{
    SetWindowLongPtr(mainWindow, MWE_WINDOW_PARAM_PTR, (LONG_PTR)pWindowParameters);
    dSASSERT(sizeof(LONG_PTR) >= sizeof(MainWindowParameters *));
}

static inline 
MainWindowParameters *retrieveMainWindowWindowParameters(HWND mainWindow)
{
    dSASSERT(sizeof(LONG_PTR) >= sizeof(MainWindowParameters *));

    return (MainWindowParameters *)GetWindowLongPtr(mainWindow, MWE_WINDOW_PARAM_PTR);
}


// callback function for "about" dialog box

static 
LRESULT CALLBACK AboutDlgProc(HWND hDlg, UINT uMsg, WPARAM wParam,
    LPARAM lParam)
{
    switch (uMsg) {
        case WM_INITDIALOG: {
            return TRUE;
        }

        case WM_COMMAND: {
            switch (wParam) {
            case IDOK:
                EndDialog(hDlg, TRUE);
                return TRUE;
            }
            break;
        }
    }

    return FALSE;
}


static LRESULT handleMainWindowCommand(HWND hWnd, WPARAM wParam, LPARAM lParam);

// callback function for the main window
static 
LRESULT CALLBACK mainWndProc(HWND hWnd, UINT msg, WPARAM wParam,
    LPARAM lParam)
{
    LRESULT result = 0;

    switch (msg) {
        case WM_CREATE: {
            MainWindowParameters *pWindowParameters = new MainWindowParameters();
            if (pWindowParameters == NULL) {
                result = -1;
                break;
            }

            assignMainWindowWindowParameters(hWnd, pWindowParameters);
            break;
        }

        case WM_LBUTTONDOWN:
        case WM_MBUTTONDOWN:
        case WM_RBUTTONDOWN: {
            MainWindowParameters *pWindowParameters = retrieveMainWindowWindowParameters(hWnd);
            if (pWindowParameters != NULL) {
                if (msg == WM_LBUTTONDOWN) {
                    pWindowParameters->m_mouseButtonStates |= MainWindowParameters::MBS_LBUTTONDOWN;
                }
                else if (msg == WM_MBUTTONDOWN) {
                    pWindowParameters->m_mouseButtonStates |= MainWindowParameters::MBS_MBUTTONDOWN;
                }
                else {
                    pWindowParameters->m_mouseButtonStates |= MainWindowParameters::MBS_RBUTTONDOWN;
                }

                pWindowParameters->m_lastMouseX = SHORT(LOWORD(lParam));
                pWindowParameters->m_lastMouseY = SHORT(HIWORD(lParam));
            }

            SetCapture(hWnd);
            break;
        }

        case WM_LBUTTONUP:
        case WM_MBUTTONUP:
        case WM_RBUTTONUP: {
            MainWindowParameters *pWindowParameters = retrieveMainWindowWindowParameters(hWnd);
            if (pWindowParameters != NULL) {
                if (msg == WM_LBUTTONUP) {
                    pWindowParameters->m_mouseButtonStates &= ~MainWindowParameters::MBS_LBUTTONDOWN;
                }
                else if (msg == WM_MBUTTONUP) {
                    pWindowParameters->m_mouseButtonStates &= ~MainWindowParameters::MBS_MBUTTONDOWN;
                }
                else {
                    pWindowParameters->m_mouseButtonStates &= ~MainWindowParameters::MBS_RBUTTONDOWN;
                }
            }

            if (pWindowParameters == NULL || pWindowParameters->m_mouseButtonStates == 0) {
                ReleaseCapture();
            }

            break;
        }

        case WM_MOUSEMOVE: {
            MainWindowParameters *pWindowParameters = retrieveMainWindowWindowParameters(hWnd);
            if (pWindowParameters != NULL) {
                int x = SHORT(LOWORD(lParam));
                int y = SHORT(HIWORD(lParam));
                if (pWindowParameters->m_mouseButtonStates != 0) {
                    dsMotion(pWindowParameters->m_mouseButtonStates, x - pWindowParameters->m_lastMouseX, y - pWindowParameters->m_lastMouseY);
                }

                pWindowParameters->m_lastMouseX = x;
                pWindowParameters->m_lastMouseY = y;
            }

            break;
        }

        case WM_CHAR: {
            if (wParam >= ' ' && wParam <= 126) {
                RenderingThreadParams *pRendererParams = retrieveMainWindowRendererThreadParams(hWnd);
                if (pRendererParams != NULL) {
                    pRendererParams->pushBufferedChar(int(wParam));
                }
            }
            break;
        }

        case WM_SIZE: {
            // lParam will contain the size of the *client* area!
            RenderingThreadParams *pRendererParams = retrieveMainWindowRendererThreadParams(hWnd);
            if (pRendererParams != NULL) {
                pRendererParams->m_rendererWidth = LOWORD(lParam);
                pRendererParams->m_rendererHeight = HIWORD(lParam);
            }
            break;
        }

        case WM_COMMAND: {
            result = handleMainWindowCommand(hWnd, wParam, lParam);
            break;
        }

        case WM_DESTROY: {
            MainWindowParameters *pWindowParameters = retrieveMainWindowWindowParameters(hWnd);
            if (pWindowParameters != NULL) {
                delete pWindowParameters;
                assignMainWindowWindowParameters(hWnd, NULL);
            }

            // PostQuitMessage(0); -- The WM_QUIT from PostQuitMessage() call is, for some reason, not removed by the PeekMessage() loop in MainWindowExternalAborter::registerForAborts() on next window creations
            //                     -- Probably the PostQuitMessage() was intentively made the way so that its WM_QUIT is not removed by PeekMessage() to make the call final and avoid accidental posted message loss in careless programming.
            PostMessage(NULL, WM_QUIT, 0, 0);
            break;
        }

        default: {
            result = DefWindowProc(hWnd, msg, wParam, lParam);
            break;
        }
    }

    return result;
}

static 
LRESULT handleMainWindowCommand(HWND hWnd, WPARAM wParam, LPARAM lParam)
{
    LRESULT result = 0;

    switch (LOWORD(wParam)) {
        case IDM_ABOUT: {
            DialogBox(g_instance, MAKEINTRESOURCE(IDD_ABOUT), hWnd, (DLGPROC)&AboutDlgProc);
            break;
        }

        case IDM_PAUSE: {
            RenderingThreadParams *pRendererParams = retrieveMainWindowRendererThreadParams(hWnd);
            if (pRendererParams != NULL) {
                bool rendererPause = (pRendererParams->m_rendererPause = !pRendererParams->m_rendererPause);
                CheckMenuItem(GetMenu(hWnd), IDM_PAUSE, rendererPause ? MF_CHECKED : MF_UNCHECKED);
                if (rendererPause) { 
                    pRendererParams->m_rendererSingleStep = false;
                }
            }

            break;
        }

        case IDM_SINGLE_STEP: {
            RenderingThreadParams *pRendererParams = retrieveMainWindowRendererThreadParams(hWnd);
            if (pRendererParams != NULL) {
                if (pRendererParams->m_rendererPause) {
                    pRendererParams->m_rendererSingleStep = true;
                }
                else {
                    SendMessage(hWnd, WM_COMMAND, IDM_PAUSE, 0);
                }
            }

            break;
        }

        case IDM_PERF_MONITOR: {
            dsWarning("Performance monitor is not implemented yet");
            break;
        }

        case IDM_TEXTURES: {
            MainWindowParameters *pWindowParameters = retrieveMainWindowWindowParameters(hWnd);
            if (pWindowParameters != NULL) {
                bool hideTextures = (pWindowParameters->m_hideTextures = !pWindowParameters->m_hideTextures);
                CheckMenuItem(GetMenu(hWnd), IDM_TEXTURES, hideTextures ? MF_UNCHECKED : MF_CHECKED);
                dsSetTextures(!hideTextures);
            }

            break;
        }

        case IDM_SHADOWS: {
            MainWindowParameters *pWindowParameters = retrieveMainWindowWindowParameters(hWnd);
            if (pWindowParameters != NULL) {
                bool hideShadows = (pWindowParameters->m_hideShadows = !pWindowParameters->m_hideShadows);
                CheckMenuItem(GetMenu(hWnd), IDM_SHADOWS, hideShadows ? MF_UNCHECKED : MF_CHECKED);
                dsSetShadows(!hideShadows);
            }

            break;
        }

        case IDM_SAVE_SETTINGS: {
            dsWarning("\"Save Settings\" not yet implemented.");
            break;
        }

        case IDM_EXIT: {
            PostMessage(hWnd, WM_CLOSE, 0, 0);
            break;
        }
    }

    return result;
}


static HWND getConsoleHwnd();
static void allocateConsole();
static void promptAKeyToExit();

static bool g_drawstuffInitialized = false;

static LPCTSTR g_mainWindowClassName = _T("SimAppClass");

static
bool startupStuff()
{
    bool result = false;

    do {
        if (!g_drawstuffInitialized) {
            g_cmdShow = SW_SHOWNORMAL;		// @@@ fix this later

            // The instance should normally be assigned in the DllMain
            if (g_instance == NULL) {
                g_instance = GetModuleHandle(NULL);
            }

            // load accelerators
            g_accelerators = LoadAccelerators(g_instance, MAKEINTRESOURCE(IDR_ACCELERATOR1));
            if (g_accelerators == NULL) {
                dsError("could not load accelerators");
                break;
            }

            // register the window class
            WNDCLASS wc;
            wc.style = CS_OWNDC | CS_VREDRAW | CS_HREDRAW;
            wc.lpfnWndProc = &mainWndProc;
            wc.cbClsExtra = 0;
            wc.cbWndExtra = MWE__MAX;
            wc.hInstance = g_instance;
            wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
            wc.hCursor = LoadCursor(NULL, IDC_ARROW);
            wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
            wc.lpszMenuName = MAKEINTRESOURCE(IDR_MENU1);
            wc.lpszClassName = g_mainWindowClassName;
            if (RegisterClass(&wc) == 0) {
                dsError("could not register window class");
                break;
            }

            allocateConsole();

            g_drawstuffInitialized = true;
        }
    	
        result = true;
    }
    while (false);

    if (!result) {
        // Nothing to be freed
    }

    return result;
}

// this comes from an MSDN example. believe it or not, this is the recommended
// way to get the console window handle.

static
HWND getConsoleHwnd()
{
    // the console window title to a "unique" value, then find the window
    // that has this title.
    char title[1024];
    wsprintf(title, "DrawStuff:%d/%d", GetTickCount(), GetCurrentProcessId());
    SetConsoleTitle(title);
    Sleep(40);			// ensure window title has been updated
    return FindWindow(NULL, title);
}

static
void allocateConsole()
{
    if (!AllocConsole()) {
        dsError("AllocConsole() failed");
    }

    BringWindowToTop(getConsoleHwnd());
    SetConsoleTitle("DrawStuff Messages");

    if (freopen("CONOUT$", "wt", stdout) == 0) {
        dsError("could not open stdout");
    }
    if (freopen("CONIN$", "rt", stdin) == 0) {
        dsError("could not open stdin");
    }
    if (freopen("CONOUT$", "wt", stderr) == 0) {
        dsError("could not open stderr");
    }

}

static 
void promptAKeyToExit()
{
    HANDLE consoleInput = GetStdHandle(STD_INPUT_HANDLE);
    if (consoleInput != INVALID_HANDLE_VALUE && consoleInput != NULL) {
        FlushConsoleInputBuffer(consoleInput);

        fprintf(stderr, "Press any key to close this window . . .");

        INPUT_RECORD inputEvent;
        for (DWORD eventsRead = 0; ReadConsoleInput(consoleInput, &inputEvent, 1, &eventsRead); eventsRead = 0) {
            if (eventsRead == 1 && inputEvent.EventType == KEY_EVENT && inputEvent.Event.KeyEvent.bKeyDown) {
                break;
            }
        }
    }
}


/*extern */
void dsPlatformInitializeConsole()
{
    allocateConsole();
}

/*extern */
void dsPlatformFinalizeConsole()
{
    promptAKeyToExit();
}


static void handleMessageLoop();

/*extern */
void dsPlatformSimLoop(int window_width, int window_height,
    dsFunctions *fn, int initial_pause)
{
    bool result = false;
    
    HWND mainWindow = NULL;
    HDC windowDC = NULL;
    bool windowCreated = false, dcObtained = false;

    do {
        if (!startupStuff()) {
            break;
        }

        // create window - but first get window size for desired size of client area.
        // if this adjustment isn't made then the openGL area will be shifted into
        // the nonclient area and determining the frame buffer coordinate from the
        // client area coordinate will be hard.
        RECT winrect;
        winrect.left = 50;
        winrect.top = 80;
        winrect.right = winrect.left + window_width;
        winrect.bottom = winrect.top + window_height;
        DWORD style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
        AdjustWindowRect(&winrect, style, 1);

        char title[100];
        sprintf(title, "Simulation test environment v%d.%02d",
            DS_VERSION >> 8, DS_VERSION & 0xff);

        mainWindow = CreateWindow(g_mainWindowClassName, title, style,
            winrect.left, winrect.top, winrect.right - winrect.left, winrect.bottom - winrect.top,
            NULL, NULL, g_instance, NULL);
        if (mainWindow == NULL) {
            dsError("could not create main window");
            break;
        }
        windowCreated = true;

        windowDC = GetDC(mainWindow);			// get DC for this window
        if (windowDC == NULL) {
            dsError("could not get window DC");
            break;
        }
        dcObtained = true;

        // set pixel format for DC

        PIXELFORMATDESCRIPTOR pfd = {
            sizeof(PIXELFORMATDESCRIPTOR),   // size of this pfd
            1,				     // version number
            PFD_DRAW_TO_WINDOW |	     // support window
            PFD_SUPPORT_OPENGL |	     // support OpenGL
            PFD_DOUBLEBUFFER,		     // double buffered
            PFD_TYPE_RGBA,		     // RGBA type
            24, 			     // 24-bit color depth
            0, 0, 0, 0, 0, 0,		     // color bits ignored
            0,				     // no alpha buffer
            0,				     // shift bit ignored
            0,				     // no accumulation buffer
            0, 0, 0, 0, 		     // accum bits ignored
            32, 			     // 32-bit z-buffer
            0,				     // no stencil buffer
            0,				     // no auxiliary buffer
            PFD_MAIN_PLANE,		     // main layer
            0,				     // reserved
            0, 0, 0			     // layer masks ignored
        };
        // get the best available match of pixel format for the device context
        int iPixelFormat = ChoosePixelFormat(windowDC, &pfd);
        if (iPixelFormat == 0) {
            dsError("could not find a good OpenGL pixel format");
            break;
        }
        // set the pixel format of the device context
        if (!SetPixelFormat(windowDC, iPixelFormat, &pfd)) {
            dsError("could not set DC pixel format for OpenGL");
            break;
        }

        // Preapre renderer parameters
        RenderingThreadParams rendererParams(initial_pause != 0, windowDC, window_width, window_height, fn);

        assignMainWindowRendererParams(mainWindow, &rendererParams);

        ShowWindow(mainWindow, g_cmdShow);

        // **********
        // start the rendering thread

        unsigned threadId;
        HANDLE hThread;

        hThread = (HANDLE)_beginthreadex(
            NULL,			     // no security attributes
            0,			     // use default stack size
            &renderingThread,	     // thread function
            &rendererParams,		     // argument to thread function
            0,			     // use default creation flags
            &threadId);		     // returns the thread identifier

        if (hThread == NULL) {
            dsError("Could not create rendering thread");
            break;
        }

        MainWindowExternalAborter::registerForAborts();

        // **********
        // start GUI message processing
        handleMessageLoop();

        // terminate rendering thread
        rendererParams.m_rendererExitRequest = true;

        MainWindowExternalAborter::unregisterFromAborts();

        WaitForSingleObject(hThread, INFINITE);
        CloseHandle(hThread);

        ReleaseDC(mainWindow, windowDC);
        // destroy window
        DestroyWindow(mainWindow);

        result = true;
    }
    while (false);

    if (!result) {
        if (windowCreated) {
            if (dcObtained) {
                ReleaseDC(mainWindow, windowDC);
            }

            DestroyWindow(mainWindow);
        }
    }
}

static 
void handleMessageLoop()
{
    MSG msg;
    BOOL retrievalResult;   
    while ((retrievalResult = GetMessage(&msg, NULL, 0, 0)) != FALSE) {
        if (retrievalResult == -1) {
            dsError("Error retrieving GUI thread messages");
            break;
        }

        if (!TranslateAccelerator(msg.hwnd, g_accelerators, &msg)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }
}

/*extern */
void dsStop()
{
    MainWindowExternalAborter::requestAbort();
}


static double g_prevElapsedTime = 0.0;

/*extern */
double dsElapsedTime()
{
    double curr = timeGetTime() / 1000.0;
    if (!g_prevElapsedTime) {
        g_prevElapsedTime = curr;
    }

    double retval = curr - g_prevElapsedTime;
    g_prevElapsedTime = curr;
    
    retval = dCLAMP(retval, dEpsilon, 1.0);

    return retval;
}


// JPerkins: if running as a DLL, grab my module handle at load time so
// I can find the accelerators table later

/*extern */
BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpReserved)
{
    switch (fdwReason)
    {
        case DLL_PROCESS_ATTACH: {
            g_instance = hinstDLL;
            break;
        }
    }

    return TRUE;
}


// JPerkins: the new build system can set the entry point of the tests to
// main(); this code is no longer necessary
/*

//***************************************************************************
// windows entry point
//
// NOTE: WinMain is not guaranteed to be called with MinGW, because MinGW
// always calls main if it is defined and most users of this library will
// define their own main. So the startup functionality is kept in
// zDriverStartup(), which is also called when dsSimulationLoop() is called.

extern "C" int main (int argc, char **argv);


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
           LPSTR lpCmdLine, int nCmdShow)
{
  drawStuffStartup();
  return main (0,0);	// @@@ should really pass cmd line arguments
}

*/



