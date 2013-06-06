#!/bin/sh

shaderVarName=`echo ${1##*/} | sed -e 's/\.vert$/Vertex/' | sed -e 's/\.geom$/Geometry/' | sed -e 's/\.frag$/Fragment/'`ShaderSource

sed -e '
# Escape backslashes first
s/\\/\\\\/g
# Now escape double quotes
s/"/\\"/g
# Wrap each line in "...\n"
s/^/"/
s/$/\\n"/
# Add header and footer
1s/^/const char *'$shaderVarName' = /
$s/$/;/' < $1
