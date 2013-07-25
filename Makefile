OPT_OR_DEBUG = -O3

CPP = g++
CPPFLAGS := -pthread -Wall -Wshadow -Werror $(OPT_OR_DEBUG) $(shell sdl2-config --cflags)
LINKFLAGS := -lAntTweakBar -lX11 -pthread -lGLEW -lGLU -lGL $(shell sdl2-config --libs)

OFILES=\
	sdl_main.o \
	oopengl.o \
	callbacks.o \
	mouseevents.o \
	controls.o \
	transform.o \
	shaders.o \
	tetrahedralize.o \
	wavefunction.o \
	radial_data.o \
	util.o \
	solid.o \
	final.o \
	glprocs.o

PROG = orbital-explorer
TEST = unittests

all: $(PROG)

$(PROG): $(OFILES)
	$(CPP) $(CPPFLAGS) $(OFILES) -o $@ $(LINKFLAGS)

$(TEST): unittests.o
	$(CPP) $(CPPFLAGS) unittests.o -o $@ $(LINKFLAGS) -lgtest -lgtest_main

%.o: %.cc
	$(CPP) $(CPPFLAGS) -MMD -MP -MF $(<:%.cc=.%.d) -c $<

shaders.cc: *.vert *.geom *.frag
	rm -f shaders.cc
	for shader in *.vert *.geom *.frag ; do \
	  ./wrap_shader.sh $$shader >> shaders.cc ; \
	done

radial_data.cc: radial_analyzer.py
	rm -f radial_data.cc
	python3 -B radial_analyzer.py > radial_data.cc

.PHONY: clean
clean:
	rm -f *~ *.o $(PROG) $(TEST)

.PHONY: cleanall
cleanall: clean
	rm -f .*.d shaders.cc radial_data.cc

# Import dependences
-include $(OFILES:%.o=.%.d)
-include .unittests.d
