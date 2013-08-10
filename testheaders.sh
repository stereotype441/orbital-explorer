#!/bin/sh

if [ "$1" != "" ]
then
    headers=$1
else
    headers="*.hh"
fi

crap='testheader.cc testheader.o .testheader.d'

for header in $headers
do
    rm -f $crap
    echo '#include "'$header'"' >> testheader.cc
    echo 'Compiling '$header
    make testheader.o || { rm -f $crap; echo Failed on $header; exit 1; }
done
rm -f $crap
