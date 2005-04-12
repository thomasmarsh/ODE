#!/bin/sh
#
echo "Running aclocal"
aclocal -I .
echo "Running autoheader"
autoheader
echo "Running automake"
automake --foreign --include-deps --add-missing --copy
echo "Running autoconf"
autoconf

#./configure $*
echo "Now you are ready to run ./configure"
