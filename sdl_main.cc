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
#include <cstdlib>
#include <SDL.h>

#include "config.hh"
#include "glprocs.hh"
#include "render.hh"
#include "viewport.hh"
#include "controls.hh"
#include "icon.hh"

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

  // Tell OpenGL we don't need a depth or alpha buffer
  set_sdl_attr(SDL_GL_DEPTH_SIZE, 0);
  set_sdl_attr(SDL_GL_ALPHA_SIZE, 0);

  // Request double buffering
  set_sdl_attr(SDL_GL_DOUBLEBUFFER, 1);

  // Apple defaults to an OpenGL 2.1 Compatibility context unless you
  // specify otherwise.
  set_sdl_attr(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  set_sdl_attr(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  set_sdl_attr(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

  Viewport viewport(640, 480);

  // Create a window
  SDL_Window *window =
    SDL_CreateWindow("Electron Orbital Explorer",
                     SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                     viewport.getWidth(), viewport.getHeight(),
                     SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  if (!window) {
    fprintf(stderr, "SDL_CreateWindow(): %s\n", SDL_GetError());
    exit(1);
  }
  SDL_Surface *icon = getIcon();
  SDL_SetWindowIcon(window, icon);
  SDL_FreeSurface(icon);

  // Create an OpenGL context associated with the window
  SDL_GLContext glcontext = SDL_GL_CreateContext(window);
  if (!glcontext) {
    fprintf(stderr, "SDL_GL_CreateContext(): %s\n", SDL_GetError());
    exit(1);
  }

  //
  // Get access to OpenGL functions
  //

  initGLProcs();

  //
  // Initialize orbital rendering pipeline
  //

  initialize();
  resizeTextures(viewport);

  //
  // Initialize controls
  //

  initControls(viewport);

  //
  // Main loop
  //

  Camera camera;
  SDL_Event event;
  bool show_controls = true;
  while (1) {
    // Clear the event queue, then redraw a frame

    while (SDL_PollEvent(&event)) {

      // Can controls handle the event, or is it a resize event?
      int handled = false;
      if (show_controls || event.type == SDL_WINDOWEVENT)
        handled = handleControls(event);

      // If event hasn't been fully handled by controls, process it
      if (!handled) {
        switch (event.type) {
        case SDL_WINDOWEVENT:
          if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
            viewport.resize(event.window.data1, event.window.data2);
            resizeTextures(viewport);
          }
          break;
        case SDL_MOUSEMOTION:
          if (event.motion.state == SDL_BUTTON_LMASK) {
            camera.rotate(double(event.motion.xrel) / viewport.getWidth(),
                          double(event.motion.yrel) / viewport.getHeight());
          }
          else if (event.motion.state == SDL_BUTTON_RMASK) {
            camera.spin(-double(event.motion.xrel) / viewport.getWidth());
            camera.zoom(double(event.motion.yrel) / viewport.getHeight());
          }
          break;
        case SDL_MOUSEWHEEL:
          camera.zoom(-DISCRETE_ZOOM_SIZE * event.wheel.y);
          break;
        case SDL_KEYDOWN:
          int key, mod;
          key = event.key.keysym.sym;
          // Ignore the Num Lock key state
          mod = event.key.keysym.mod & ~KMOD_NUM;
          if (mod == KMOD_NONE) {
            switch (key) {
            case SDLK_LEFT:
              camera.rotate(-DISCRETE_ROTATION_SIZE, 0);
              break;
            case SDLK_RIGHT:
              camera.rotate(DISCRETE_ROTATION_SIZE, 0);
              break;
            case SDLK_UP:
              camera.rotate(0, -DISCRETE_ROTATION_SIZE);
              break;
            case SDLK_DOWN:
              camera.rotate(0, DISCRETE_ROTATION_SIZE);
              break;
            case SDLK_PAGEUP:
              camera.zoom(-DISCRETE_ZOOM_SIZE);
              break;
            case SDLK_PAGEDOWN:
              camera.zoom(DISCRETE_ZOOM_SIZE);
              break;
            case SDLK_F11:
              static int fullscreen_mode = 0;
              fullscreen_mode ^= SDL_WINDOW_FULLSCREEN_DESKTOP;
              SDL_SetWindowFullscreen(window, fullscreen_mode);
              break;
            case SDLK_F10:
              show_controls = !show_controls;
              break;
            default:
              break;
            }
          }
          else if (mod == KMOD_LSHIFT || mod == KMOD_RSHIFT ||
                   mod == KMOD_SHIFT) {
            switch (key) {
            case SDLK_LEFT:
              camera.spin(DISCRETE_ROTATION_SIZE);
              break;
            case SDLK_RIGHT:
              camera.spin(-DISCRETE_ROTATION_SIZE);
              break;
            case SDLK_UP:
              camera.zoom(-DISCRETE_ZOOM_SIZE);
              break;
            case SDLK_DOWN:
              camera.zoom(DISCRETE_ZOOM_SIZE);
              break;
            default:
              break;
            }
          }
          break;
        case SDL_QUIT:
          return 0;
        }
      }
    }

    display(viewport, camera);
    if (show_controls)
      drawControls();
    SDL_GL_SwapWindow(window);
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
