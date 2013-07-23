#!/bin/sh

if [ "$1" != "" ]
then
    headers=$1
else
    headers="*.hh"
fi

for header in $headers
do
    rm -f testheader.cc testheader.o
    echo '#include "'$header'"' >> testheader.cc
    echo 'Compiling '$header
    make testheader.o || { rm -f testheader.cc testheader.o; exit 1; }
done
rm -f testheader.cc testheader.o
