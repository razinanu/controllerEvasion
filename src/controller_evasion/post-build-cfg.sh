#!/bin/bash

BASEDIR=$(dirname $0)/..

# move generated file from src to scripts folder
mv $BASEDIR/src/controller_evasion/cfg/*Config.py $BASEDIR/src/

# remove unnecessary but generated folders
rm -rf $BASEDIR/cfg/cpp
rm $BASEDIR/docs/*Config*
rmdir $BASEDIR/docs
exit 0