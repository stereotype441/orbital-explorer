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

/*
 * This file is a derivative work of Philippe Decaudin's AntTweakBar
 * library, version 1.16, which is distributed under the following terms:
 *
 * Copyright (C) 2005-2013 Philippe Decaudin
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must
 * not claim that you wrote the original software. If you use this
 * software in a product, an acknowledgment in the product documentation
 * would be appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must
 * not be misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source
 * distribution.
 */

#include <AntTweakBar.h>

#include "SDLtoATB.hh"

// TwEventSDL returns zero if msg has not been handled,
// and a non-zero value if it has been handled by the AntTweakBar library.
int myTwEventSDL20(const SDL_Event &event)
{
  int handled = 0;
  static int s_KeyMod = 0;

  switch (event.type) {

  case SDL_TEXTINPUT:
    if (event.text.text[0] != 0 && event.text.text[1] == 0) {
      if (s_KeyMod & TW_KMOD_CTRL && event.text.text[0] < 32)
        handled = TwKeyPressed(event.text.text[0] + 'a' - 1, s_KeyMod);
      else {
        if (s_KeyMod & KMOD_RALT)
          s_KeyMod &= ~KMOD_CTRL;
        handled = TwKeyPressed(event.text.text[0], s_KeyMod);
      }
    }
    break;

  case SDL_KEYDOWN:
    if (event.key.keysym.sym & SDLK_SCANCODE_MASK) {
      int key = 0;
      switch (event.key.keysym.sym) {
      case SDLK_UP:
        key = TW_KEY_UP;
        break;
      case SDLK_DOWN:
        key = TW_KEY_DOWN;
        break;
      case SDLK_RIGHT:
        key = TW_KEY_RIGHT;
        break;
      case SDLK_LEFT:
        key = TW_KEY_LEFT;
        break;
      case SDLK_INSERT:
        key = TW_KEY_INSERT;
        break;
      case SDLK_HOME:
        key = TW_KEY_HOME;
        break;
      case SDLK_END:
        key = TW_KEY_END;
        break;
      case SDLK_PAGEUP:
        key = TW_KEY_PAGE_UP;
        break;
      case SDLK_PAGEDOWN:
        key = TW_KEY_PAGE_DOWN;
        break;
      default:
        if (event.key.keysym.sym >= SDLK_F1 &&
            event.key.keysym.sym <= SDLK_F12)
          key = event.key.keysym.sym + TW_KEY_F1 - SDLK_F1;
      }
      if (key != 0)
        handled = TwKeyPressed(key, event.key.keysym.mod);
    }
    else if (event.key.keysym.mod & TW_KMOD_ALT)
      handled = TwKeyPressed(event.key.keysym.sym & 0xFF,
                             event.key.keysym.mod);
    s_KeyMod = event.key.keysym.mod;
    break;

  case SDL_KEYUP:
    s_KeyMod = event.key.keysym.mod;
    break;

  case SDL_MOUSEMOTION:
    handled = TwMouseMotion(event.motion.x, event.motion.y);
    break;

  case SDL_MOUSEWHEEL:
    {
      static int s_WheelPos = 0;
      s_WheelPos += event.wheel.y;
      handled = TwMouseWheel(s_WheelPos);
    }
    break;

  case SDL_MOUSEBUTTONUP:
  case SDL_MOUSEBUTTONDOWN:
    TwMouseAction action;
    if (event.button.state == SDL_PRESSED)
      action = TW_MOUSE_PRESSED;
    else if (event.button.state == SDL_RELEASED)
      action = TW_MOUSE_RELEASED;
    else
      break;

    TwMouseButtonID button;
    if (event.button.button == SDL_BUTTON_LMASK)
      button = TW_MOUSE_LEFT;
    else if (event.button.button == SDL_BUTTON_MMASK)
      button = TW_MOUSE_MIDDLE;
    else if (event.button.button == SDL_BUTTON_RMASK)
      button = TW_MOUSE_RIGHT;
    else
      break;

    handled = TwMouseButton(action, button);

    break;

  case SDL_WINDOWEVENT:
    if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
      // tell the new size to TweakBar
      TwWindowSize(event.window.data1, event.window.data2);
      // do not set 'handled', SDL_VIDEORESIZE may be also processed by
      // the calling application
    }
    break;
  }

  return handled;
}
