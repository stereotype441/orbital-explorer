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
#include <vector>
#include <map>
#include <complex>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <SDL2/SDL.h>

#include "glprocs.hh"
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

#include "AntTweakBar.h"

using namespace std;

static int Z = 1;
static int N = 1;
static int L = 0;
static int M = 0;
static int absM = 0;
static double E = 0.0; // in eV
static const int minZ = 1;
static const int maxZ = 100; // FIXME
static const int minN = 1;
static const int maxN = 16;
static const int minL = 0;
static       int maxL = 0;
static       int minM = 0;
static       int maxM = 0;
static const int minAbsM = 0;
static       int maxAbsM = 0;
static bool basisReal = false;
static bool comboDiff = false;
static bool orbital = true;
static double brightness = 1.0;
static int detail = 5;
static bool colorPhase = true;
static int cycleRate = 1;

static void changeZ(int);
static void changeN(int);
static void changeL(int);
static void changeM(int);
static void changeAbsM(int);
static void changeBasis(bool);
static void changeCombo(bool);

static void setEnergy()
{
  E = 13.60569253 * double(Z) * double(Z) / (double(N) * double(N));
}

static void changeZ(int newZ)
{
  if (newZ > maxZ) newZ = maxZ;
  if (newZ < minZ) newZ = minZ;
  Z = newZ;

  setEnergy();
}

static void changeN(int newN)
{
  if (newN > maxN) newN = maxN;
  if (newN < minN) newN = minN;
  N = newN;

  maxL = N - 1;
  changeL(L);

  setEnergy();
}

static void changeL(int newL)
{
  if (newL > maxL) newL = maxL;
  if (newL < minL) newL = minL;
  L = newL;

  minM = -L;
  maxM = L;
  changeM(M);

  maxAbsM = L;
  changeAbsM(absM);
}

static void changeM(int newM)
{
  if (newM > maxM) newM = maxM;
  if (newM < minM) newM = minM;
  M = newM;
  absM = abs(M);
}

static void changeAbsM(int newAbsM)
{
  if (newAbsM > maxAbsM) newAbsM = maxAbsM;
  if (newAbsM < minAbsM) newAbsM = minAbsM;
  absM = newAbsM;
  if (M < 0)
    M = -absM;
  else
    M = absM;
}

static void changeBasis(bool newBasis)
{
  basisReal = newBasis;
}

static void changeCombo(bool newCombo)
{
  comboDiff = newCombo;
}

static TwBar *physics = NULL;
static TwBar *graphics = NULL;

static void myTwTerminate()
{
  // Returns an int, so can't register with atexit()...
  TwTerminate();
}

static void TW_CALL setZ(const void *data, void *)
{
  changeZ(*(const int *)data);
}

static void TW_CALL getZ(void *data, void *)
{
  *(int *)data = Z;
}

static void TW_CALL setN(const void *data, void *)
{
  changeN(*(const int *)data);

  TwSetParam(physics, "L", "max", TW_PARAM_INT32, 1, &maxL);
  TwSetParam(physics, "M", "min", TW_PARAM_INT32, 1, &minM);
  TwSetParam(physics, "M", "max", TW_PARAM_INT32, 1, &maxM);
  TwSetParam(physics, "|M|", "max", TW_PARAM_INT32, 1, &maxAbsM);
}

static void TW_CALL getN(void *data, void *)
{
  *(int *)data = N;
}

static void TW_CALL setL(const void *data, void *)
{
  changeL(*(const int *)data);

  TwSetParam(physics, "M", "min", TW_PARAM_INT32, 1, &minM);
  TwSetParam(physics, "M", "max", TW_PARAM_INT32, 1, &maxM);
  TwSetParam(physics, "|M|", "max", TW_PARAM_INT32, 1, &maxAbsM);
}

static void TW_CALL getL(void *data, void *)
{
  *(int *)data = L;
}

static void TW_CALL setM(const void *data, void *)
{
  changeM(*(const int *)data);
}

static void TW_CALL getM(void *data, void *)
{
  *(int *)data = M;
}

static void TW_CALL setAbsM(const void *data, void *)
{
  changeAbsM(*(const int *)data);
}

static void TW_CALL getAbsM(void *data, void *)
{
  *(int *)data = absM;
}

static void TW_CALL setBasis(const void *data, void *)
{
  bool val = *(const bool *)data;
  changeBasis(val);

  int f = 0, t = 1;
  if (val) {
    TwSetParam(physics, "M", "visible", TW_PARAM_INT32, 1, &f);
    TwSetParam(physics, "|M|", "visible", TW_PARAM_INT32, 1, &t);
    TwSetParam(physics, "Combo", "visible", TW_PARAM_INT32, 1, &t);
  } else {
    TwSetParam(physics, "M", "visible", TW_PARAM_INT32, 1, &t);
    TwSetParam(physics, "|M|", "visible", TW_PARAM_INT32, 1, &f);
    TwSetParam(physics, "Combo", "visible", TW_PARAM_INT32, 1, &f);
  }
}

static void TW_CALL getBasis(void *data, void *)
{
  *(bool *)data = basisReal;
}

static void TW_CALL setCombo(const void *data, void *)
{
  bool val = *(const bool *)data;
  changeCombo(val);
}

static void TW_CALL getCombo(void *data, void *)
{
  *(bool *)data = comboDiff;
}

static int fps = 0;
static int vertices = 0;
static int tetrahedra = 0;

void initControls()
{
  int t = 1;
  int f = 0;

  //
  // Initialize AntTweakBar controls
  //

  if (TwInit(TW_OPENGL_CORE, NULL) == 0) {
    fprintf(stderr, "TwInit(): %s\n", TwGetLastError());
    exit(1);
  }
  atexit(myTwTerminate);

  TwWindowSize(getWidth(), getHeight());

  physics = TwNewBar("Physics");
  graphics = TwNewBar("Graphics");
  TwSetParam(graphics, NULL, "iconified", TW_PARAM_INT32, 1, &t);

  if (sizeof(int) != 4) {
    fprintf(stderr, "sizeof(int) != 4\n");
    exit(1);
  }

  // Quantum Numbers

  TwAddVarCB(physics, "Z", TW_TYPE_INT32, setZ, getZ, NULL,
             "help=`Nuclear charge`"
             " group=`Orbital`"
             " keydecr=z keyincr=Z"
             );
  TwSetParam(physics, "Z", "min", TW_PARAM_INT32, 1, &minZ);
  TwSetParam(physics, "Z", "max", TW_PARAM_INT32, 1, &maxZ);

  TwAddVarCB(physics, "N", TW_TYPE_INT32, setN, getN, NULL,
             "help=`Principal quantum number or \"energy level\"`"
             " group=`Orbital`"
             " keydecr=n keyincr=N"
             );
  TwSetParam(physics, "N", "min", TW_PARAM_INT32, 1, &minN);
  TwSetParam(physics, "N", "max", TW_PARAM_INT32, 1, &maxN);

  TwAddVarCB(physics, "L", TW_TYPE_INT32, setL, getL, NULL,
             "help=`Angular momentum quantum number`"
             " group=`Orbital`"
             " keydecr=l keyincr=L"
             );
  TwSetParam(physics, "L", "min", TW_PARAM_INT32, 1, &minL);
  TwSetParam(physics, "L", "max", TW_PARAM_INT32, 1, &maxL);

  TwAddVarCB(physics, "M", TW_TYPE_INT32, setM, getM, NULL,
             "help=`Angular momentum z-projection quantum number`"
             " group=`Orbital`"
             " keydecr=m keyincr=M"
             );
  TwSetParam(physics, "M", "min", TW_PARAM_INT32, 1, &minM);
  TwSetParam(physics, "M", "max", TW_PARAM_INT32, 1, &maxM);

  TwAddVarCB(physics, "|M|", TW_TYPE_INT32, setAbsM, getAbsM, NULL,
             "help=`Absolute value of angular momentum z-projection"
             " quantum number`"
             " group=`Orbital`"
             " keydecr=a keyincr=A"
             );
  TwSetParam(physics, "|M|", "min", TW_PARAM_INT32, 1, &minAbsM);
  TwSetParam(physics, "|M|", "max", TW_PARAM_INT32, 1, &maxAbsM);
  // Real starts set to false, so this starts hidden
  TwSetParam(physics, "|M|", "visible", TW_PARAM_INT32, 1, &f);

  // Which function

  TwAddVarRW(physics, "Function", TW_TYPE_BOOLCPP, &orbital,
             "help=`Specifies whether the probability distribution or "
             " underlying wave function is rendered.`"
             " group=`Orbital`"
             " false=`WAVE` true=`PROB`"
             " key=f"
             );

  TwAddVarCB(physics, "Basis", TW_TYPE_BOOLCPP,
             setBasis, getBasis, NULL,
             "help=`Complex wave functions are eigenfunctions for"
             " energy, angular momentum, and the z-projection of angular"
             " momentum. Real wave functions are eigenfunctions"
             " for energy, angular momentum, and the absolute value"
             " of the z-projection of angular momentum.`"
             " group=`Orbital`"
             " false=`COMPLEX` true=`REAL`"
             " key=b"
             );

  TwAddVarCB(physics, "Combo", TW_TYPE_BOOLCPP,
             setCombo, getCombo, NULL,
             "help=`Specifies whether a sum or difference of complex"
             " eigenfunctions for +/-M should be used to generate a"
             " real-valued eigenfunction for |M|.`"
             " group=`Orbital`"
             " false=`SUM` true=`DIFF`"
             " key=c"
             );
  // Real starts set to false, so this starts hidden
  TwSetParam(physics, "Combo", "visible", TW_PARAM_INT32, 1, &f);

  TwAddVarRO(physics, "Energy (eV)", TW_TYPE_DOUBLE, &E,
             "help=`Orbital energy`"
             " group=`Orbital`"
             );

  // Misc display parameters

  TwAddVarRW(physics, "Color", TW_TYPE_BOOLCPP, &colorPhase,
             "help=`Select whether the phase of the wave function"
             " is shown with color.`"
             " group=`Display`");

  TwAddVarRW(physics, "Cycle rate", TW_TYPE_INT32, &cycleRate,
             "help=`Select how fast the colors cycle, in units of angular"
             " frequency (rad/s).`"
             " group=`Display`"
             " min=0 max=10");

  TwAddVarRW(physics, "Brightness", TW_TYPE_DOUBLE, &brightness,
             "help=`Adjust the brightness of the orbital.`"
             " group=`Display`"
             " min=-10.0 max=10.0 step=0.1 precision=2");

  TwAddVarRW(physics, "Detail", TW_TYPE_INT32, &detail,
             "help=`Adjust the level of detail in the rendering."
             " The number of graphics primitives increases exponentially"
             " as this setting is increased.`"
             " group=`Display`"
             " min=1 max=10");

  // Rendering

  TwAddVarRO(graphics, "FPS", TW_TYPE_INT32, &fps,
             "help=`Frames per second`"
             " group=`Rendering`"
             );

  TwAddVarRO(graphics, "Vertices", TW_TYPE_INT32, &vertices,
             "help=`Number of vertices describing the orbital`"
             " group=`Rendering`"
             );

  TwAddVarRO(graphics, "Tetrahedra", TW_TYPE_INT32, &tetrahedra,
             "help=`Number of tetrahedra describing the orbital`"
             " group=`Rendering`"
             );

  // GPU & driver info

  static const string gpu(reinterpret_cast<const char *>
                          (glGetString(GL_RENDERER)));
  TwAddVarRO(graphics, "GPU", TW_TYPE_STDSTRING, &gpu,
             "help=`Driver version, as reported by glGetString(GL_RENDERER)`"
             " group=`GPU & Driver`");

  static const string gl(reinterpret_cast<const char *>
                         (glGetString(GL_VERSION)));
  TwAddVarRO(graphics, "GL", TW_TYPE_STDSTRING, &gl,
             "help=`OpenGL version, as reported by glGetString(GL_VERSION)`"
             " group=`GPU & Driver`");

  static const string glsl(reinterpret_cast<const char *>
                           (glGetString(GL_SHADING_LANGUAGE_VERSION)));
  TwAddVarRO(graphics, "GLSL", TW_TYPE_STDSTRING, &glsl,
             "help=`GLSL version, as reported by "
             "glGetString(GL_SHADING_LANGUAGE_VERSION)`"
             " group=`GPU & Driver`");

  TwSetParam(graphics, "GPU & Driver", "opened", TW_PARAM_INT32, 1, &f);

  setEnergy();
}

int handleControls(SDL_Event &event)
{
  // For now, don't try to send SDL 2 events to the AntTweakBar handler
  // that expects SDL 1 events...
  return false;
  //return TwEventSDL(&event, SDL_MAJOR_VERSION, SDL_MINOR_VERSION);
}

int keyPressed(int key)
{
  return TwKeyPressed(key, 0);
}

void drawControls()
{
  static time_t then = time(NULL);
  time_t now = time(NULL);
  static int frame_count = 0;
  ++frame_count;
  if (then != now) {
    fps = frame_count;
    frame_count = 0;
    then = now;
  }

  TwDraw();
}

void setVerticesTetrahedra(int v, int t)
{
  vertices = v;
  tetrahedra = t;
}

Orbital getOrbital()
{
  int m = basisReal ? absM : M;

  return Orbital(Z, N, L, m, basisReal, comboDiff, orbital);
}

double getBrightness()
{
  return brightness;
}

int getDetail()
{
  return detail;
}

bool getColorPhase()
{
  return colorPhase;
}

int getCycleRate()
{
  return cycleRate;
}
