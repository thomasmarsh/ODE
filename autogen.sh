#!/bin/sh

# The reason this uses sed instead of "grep --only-matches"
# is because MinGW's grep is an old version and does not contain that flag.
automake_version=`automake --version | grep --regexp='[+0-9].[+0-9].[+0-9]' | sed -n 's/[* ()A-Za-z]//g;p'`
automake_mayor=${automake_version%.*.*}
automake_minor=${automake_version%.*}
automake_minor=${automake_minor##*.}
automake_revision=${automake_version##*.}
echo "AutoMake Version: $automake_mayor.$automake_minor.$automake_revision"

if [ $automake_mayor -eq 1 ]; then
    if [ $automake_minor -lt 8 ]; then
	echo "Automake must be 1.8.2 or higher, please upgrade"
	exit
    else
	if [ $automake_minor -eq 8 ] && [ $automake_revision -lt 2 ]; then
	    echo "Automake must be 1.8.2 or higher, please upgrade"
	    exit
	fi
    fi
fi
 
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
