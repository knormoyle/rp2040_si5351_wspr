#!/usr/bin/env bash 
source /home/kevin/Downloads/spy/errorhandle
GIT_LIB_DIR='/home/kevin/rp2040_si5351_wspr/libraries'
LIB_DIR="/home/kevin/Arduino/libraries"
ls -ltr $LIB_DIR
echo ""

cd $LIB_DIR
rm -f -r $LIB_DIR/*

function cpit {
    if [ ! -d $GIT_LIB_DIR/$1 ] ; then
        echo "ERROR: directory $GIT_LIB_DIR/$1 is missing"
        exit 1
    fi
    rm -f -r $1
    if false ; then
        cp -p -r $GIT_LIB_DIR/$1 .
        echo "cp -p -r $GIT_LIB_DIR/$1 ."
    else
        echo "ln -s $GIT_LIB_DIR/$1 ."
        ln -s $GIT_LIB_DIR/$1 .
    fi
}

pushd $GIT_LIB_DIR
dirs=$(ls -1)
popd

for d in $dirs ; do
    if [[ "$d" == "not_used" ]] ; then
        continue
    fi
    echo "$d"
    if [ ! -d $GIT_LIB_DIR/$d ] ; then
        echo "ERROR: directory $GIT_LIB_DIR/$d is not a directory"
        exit 1
    fi
    cpit $d
done


echo ""
ls -ltr $LIB_DIR
