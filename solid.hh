#ifndef SOLID_HH
#define SOLID_HH

#include <GL/glew.h>

#include "matrix.hh"

void initSolids();
void drawSolids(const Matrix<4,4> &mvpm, int width, int height,
                GLuint solidFBO);

#endif
