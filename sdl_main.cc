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

  if (SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0) < 0 ||
      SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) < 0) {
    fprintf(stderr, "STL_GL_SetAttribute(): %s\n", SDL_GetError());
    return 1;
  }

  int starting_width = 640;
  int starting_height = 480;

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

  SDL_WM_SetCaption("Electron Orbital Explorer", "EOE"); // Can't fail

  SDL_EnableUNICODE(1); // Can't fail

  if (SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY,
                          SDL_DEFAULT_REPEAT_INTERVAL) < 0) {
    fprintf(stderr, "SDL_EnableKeyRepeat(): %s\n", SDL_GetError());
    // This is not a fatal error, so keep going
  }

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
      // (Note that this takes care of resizing the controls)
      int handled = handleControls(event);

      // If event has not been handled by controls, process it
      if (!handled) {
        switch (event.type) {
        case SDL_VIDEORESIZE:
          resize(event.resize.w, event.resize.h);
          SDL_SetVideoMode(getWidth(), getHeight(), bpp, videoModeFlags);
          resizeTextures();
          break;
        case SDL_MOUSEMOTION:
          if (event.motion.state == SDL_BUTTON(1)) // Left button down
            mouse_drag_left(event.motion.xrel, event.motion.yrel);
          if (event.motion.state == SDL_BUTTON(3)) // Right button down
            mouse_drag_right(event.motion.xrel, event.motion.yrel);
          break;
        case SDL_MOUSEBUTTONDOWN:
          if (event.button.button == 4) // Scroll wheel up
            mouse_wheel(1);
          if (event.button.button == 5) // Scroll wheel down
            mouse_wheel(-1);
          break;
        case SDL_QUIT:
          return 0;
        }
      }
    }

    display();
    drawControls();
    SDL_GL_SwapBuffers();
  }

  return 0;
}

int main(int argc, char *argv[])
{
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
