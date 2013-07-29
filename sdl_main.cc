/*
 * This file is part of the Electron Orbital Explorer. The Electron
 * Orbital Explorer is distributed under the Simplified BSD License
 * (also called the "BSD 2-Clause License"), in hopes that these
 * rendering techniques might be used by other programmers in
 * applications such as scientific visualization, video gaming, and so
 * on. If you find value in this software and use its technologies for
 * another purpose, I would love to hear back from you at bjthinks (at)
 * gmail (dot) com. If you improve this software and agree to release
 * your modifications under the below license, I encourage you to fork
 * the development tree on github and push your modifications. The
 * Electron Orbital Explorer's development URL is:
 * https://github.com/bjthinks/orbital-explorer
 * (This paragraph is not part of the software license and may be
 * removed.)
 *
 * Copyright (c) 2013, Brian W. Johnson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * + Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * + Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdexcept>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <complex>
#include <cstdlib>
#include <cmath>
#include <unistd.h>
#include <SDL.h>

#include "glprocs.hh"
#include "callbacks.hh"
#include "array.hh"
#include "genericops.hh"
#include "vector.hh"
#include "matrix.hh"
#include "quaternion.hh"
#include "function.hh"
#include "polynomial.hh"
#include "wavefunction.hh"
#include "mouseevents.hh"
#include "controls.hh"
#include "util.hh"

using namespace std;

void set_sdl_attr(SDL_GLattr attr, int value)
{
  if (SDL_GL_SetAttribute(attr, value) < 0) {
    fprintf(stderr, "SDL_GL_SetAttribute(): %s\n", SDL_GetError());
    exit(1);
  }
}

static int go()
{
  //
  // Initialize SDL
  //

  if (SDL_Init(SDL_INIT_VIDEO) < 0 ) {
    fprintf(stderr, "SDL_Init(): %s\n", SDL_GetError());
    return 1;
  }
  atexit(SDL_Quit);

  // Request at least 24-bit color
  set_sdl_attr(SDL_GL_RED_SIZE, 8);
  set_sdl_attr(SDL_GL_GREEN_SIZE, 8);
  set_sdl_attr(SDL_GL_BLUE_SIZE, 8);

  // Tell OpenGL we don't need a depth buffer
  set_sdl_attr(SDL_GL_DEPTH_SIZE, 0);

#if SDL_MAJOR_VERSION == 1
  if (SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0) < 0 ||
      SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) < 0) {
    fprintf(stderr, "STL_GL_SetAttribute(): %s\n", SDL_GetError());
    return 1;
  }
#endif

#ifdef __APPLE__
#if SDL_MAJOR_VERSION == 1
#error The Orbital Explorer requires SDL 2 on OSX
#endif
  // Apple defaults to an OpenGL 2.1 Compatibility context unless you
  // specify otherwise.
  set_sdl_attr(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  set_sdl_attr(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  set_sdl_attr(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
#endif

  int starting_width = 640;
  int starting_height = 480;

#if SDL_MAJOR_VERSION == 1
  const SDL_VideoInfo *video = SDL_GetVideoInfo();
  int bpp = video->vfmt->BitsPerPixel;
  const Uint32 videoModeFlags = SDL_OPENGL | SDL_RESIZABLE;
  resize(starting_width, starting_height);
  SDL_Surface *screen = SDL_SetVideoMode(getWidth(), getHeight(),
                                         bpp, videoModeFlags);
  if (screen == NULL) {
    fprintf(stderr, "SDL_SetVideoMode(): %s\n", SDL_GetError());
    return 1;
  }
#else
  // Create a window
  SDL_Window *window =
    SDL_CreateWindow("Electron Orbital Explorer",
                     SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                     starting_width, starting_height,
                     SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  if (!window) {
    fprintf(stderr, "SDL_CreateWindow(): %s\n", SDL_GetError());
    exit(1);
  }
#endif

#if SDL_MAJOR_VERSION == 1
  SDL_WM_SetCaption("Electron Orbital Explorer", "EOE"); // Can't fail
#else
  // Create an OpenGL context associated with the window
  SDL_GLContext glcontext = SDL_GL_CreateContext(window);
  if (!glcontext) {
    fprintf(stderr, "SDL_GL_CreateContext(): %s\n", SDL_GetError());
    exit(1);
  }

  resize(starting_width, starting_height);
#endif

#if SDL_MAJOR_VERSION == 1
  SDL_EnableUNICODE(1); // Can't fail

  if (SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY,
                          SDL_DEFAULT_REPEAT_INTERVAL) < 0) {
    fprintf(stderr, "SDL_EnableKeyRepeat(): %s\n", SDL_GetError());
    // This is not a fatal error, so keep going
  }
#endif

  //
  // Get access to OpenGL functions
  //

  initGLProcs();

  //
  // Initialize orbital rendering pipeline
  //

  initialize();
  resizeTextures();

  //
  // Initialize controls
  //

  initControls();

  //
  // Main loop
  //

  SDL_Event event;
  while (1) {
    // Clear the event queue, then redraw a frame

    while (SDL_PollEvent(&event)) {

      // Can controls handle the event?
      int handled = handleControls(event);

      // If event hasn't been fully handled by controls, process it
      if (!handled) {
        switch (event.type) {
#if SDL_MAJOR_VERSION == 1
        case SDL_VIDEORESIZE:
          resize(event.resize.w, event.resize.h);
          SDL_SetVideoMode(getWidth(), getHeight(), bpp, videoModeFlags);
          resizeTextures();
          break;
#else
        case SDL_WINDOWEVENT:
          if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
            resize(event.window.data1, event.window.data2);
            resizeTextures();
          }
          break;
#endif
        case SDL_MOUSEMOTION:
          // TODO: detect mouse button down, and put the mouse into
          // relative mode.
          if (event.motion.state == SDL_BUTTON_LMASK)
            mouse_drag_left(event.motion.xrel, event.motion.yrel);
          if (event.motion.state == SDL_BUTTON_RMASK)
            mouse_drag_right(event.motion.xrel, event.motion.yrel);
          break;
#if SDL_MAJOR_VERSION == 1
        case SDL_MOUSEBUTTONDOWN:
          if (event.button.button == SDL_BUTTON_WHEELUP)
            mouse_wheel(1);
          if (event.button.button == SDL_BUTTON_WHEELDOWN)
            mouse_wheel(-1);
          break;
#else
        case SDL_MOUSEWHEEL:
          int amount;
          amount = event.wheel.y;
          if (amount)
            mouse_wheel(amount);
          break;
#endif
        case SDL_QUIT:
          return 0;
        }
      }
    }

    display();
    drawControls();
#if SDL_MAJOR_VERSION == 1
    SDL_GL_SwapBuffers();
#else
    SDL_GL_SwapWindow(window);
#endif
  }

  return 0;
}

int main(int argc, char *argv[])
{

#ifdef __APPLE__

  //
  // Compensate for a bug in some versions of AntTweakBar. ATB
  // attempts to dynamically link with the OpenGL framework without
  // specifying a full pathname. Setting DYLD_LIBRARY_PATH fixes the
  // issue, but this environment variable must be set prior to running
  // the app. So we check it, and, if needed, set it properly and
  // re-exec ourself. :-(
  //

  const char *pathvar = "DYLD_LIBRARY_PATH";
  string new_path("/System/Library/Frameworks/OpenGL.framework/"
                  "Versions/Current");
  const char *actual_path_cstr = getenv(pathvar);
  string actual_path(actual_path_cstr ? actual_path_cstr : "");
  if (new_path != actual_path.substr(0, new_path.length()) &&
      // Try to prevent an infinite loop, in case something goes wrong
      actual_path.length() < 4000) {
    if (actual_path != "")
      new_path += ":" + actual_path;
    setenv(pathvar, new_path.c_str(), 1);
    execvp(argv[0], argv);
    // If the exec fails, the best we can do is simply continue...
  }

#endif

  const string polite_error_message =
    "I\'m sorry, Electron Orbital Explorer has crashed.\n"
    "This should never happen and is a bug in the program.\n"
    "Please copy and paste the following error message,\n"
    "and send it to the developer.\n";

  try {
    return go();
  } catch (exception &e) {
    cerr << polite_error_message;
    cerr << "Exception: " << e.what() << "\n";
    return 1;
  } catch (...) {
    cerr << polite_error_message;
    cerr << "Exception: unknown uncaught object thrown\n";
    return 1;
  }
}
