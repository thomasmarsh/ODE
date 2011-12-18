#!/bin/sh

# The version nr detection is not working.
# It fails if there is a '-' char in the version nr e.g.
# We should not use it until fixed. - Bram

## The reason this uses sed instead of "grep --only-matches"
## is because MinGW's grep is an old version and does not contain that flag.
#automake_version=`automake --version | grep --regexp='[+0-9].[+0-9].[+0-9]' | sed -n 's/[* ()A-Za-z]//g;p'`
#automake_mayor=${automake_version%.*.*}
#automake_minor=${automake_version%.*}
#automake_minor=${automake_minor##*.}
#automake_revision=${automake_version##*.}
#echo "AutoMake Version: $automake_mayor.$automake_minor.$automake_revision"
#
#if [ $automake_mayor -eq 1 ]; then
#    if [ $automake_minor -lt 8 ]; then
#	echo "Automake must be 1.8.2 or higher, please upgrade"
#	exit
#    else
#	if [ $automake_minor -eq 8 ] && [ $automake_revision -lt 2 ]; then
#	    echo "Automake must be 1.8.2 or higher, please upgrade"
#	    exit
#	fi
#    fi
#fi

echo "Please make sure that you use automake 1.11 or later"
echo "On OSX, install latest libtool, as the OS provided glibtoolize will not work"
echo "Warnings about underquoted definitions are harmless"
 
echo "Running aclocal"
aclocal -I m4 --install || exit 1
# on Mac libtoolize is called glibtoolize
LIBTOOLIZE=libtoolize
#if [ `uname -s` = Darwin ]; then
#    LIBTOOLIZE=glibtoolize
#fi
echo "Running $LIBTOOLIZE"
$LIBTOOLIZE --copy --force --automake || exit 1
echo "Running autoheader"
autoheader || exit 1
echo "Running automake"
automake --foreign --add-missing --copy || exit 1
echo "Running autoconf"
autoconf || exit 1

echo "Running bootstrap in ou directory"
(cd ou && ./bootstrap)

#./configure $*

if [ -d libccd ]; then
    echo "Running bootstrap in libccd directory"
    (cd libccd && ./bootstrap)
fi;

echo "Now you are ready to run ./configure"
