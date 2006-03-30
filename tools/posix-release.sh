#!/bin/sh
###################################################################
# ODE Windows Release Script
# Originally written by Jason Perkins (starkos@gmail.com)
#
# Prerequisites:
#  svn, zip, and C++ build tools
###################################################################

# Check arguments
if [ $# -ne 2 ]; then
  echo 1>&2 "Usage: $0 version_number branch_name"
  exit 1
fi


###################################################################
# Pre-build checklist
###################################################################

echo "" 
echo "STARTING PREBUILD CHECKLIST, PRESS ^^C TO ABORT."
echo ""
echo "Is the version number '$1' correct?"
read line
echo ""
echo "Have you created a release branch named '$2' in SVN?"
read line
echo ""
echo "Have you run all of the tests?"
read line
echo ""
echo "Is the Changelog up to date?"
read line
echo ""
echo "Okay, ready to build the POSIX packages for version $1!"
read line


###################################################################
# Prepare source code
###################################################################

echo ""
echo "RETRIEVING SOURCE CODE FROM REPOSITORY..."
echo ""

svn export https://svn.sourceforge.net/svnroot/opende/branches/$2 ode-$1
cp ode-$1/build/config-default.h ode-$1/include/ode/config.h


