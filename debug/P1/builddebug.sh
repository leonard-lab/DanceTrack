#!/bin/bash

MT_SRC_DIR=~/src/MADTraC/MT
MT_INC_DIR=~/src/MADTraC/build/include
OTHERSRCS="$MT_SRC_DIR/MT_Tracking/trackers/GY/GYBlobs.cpp $MT_SRC_DIR/MT_Tracking/trackers/GY/MixGaussians.cpp $MT_SRC_DIR/MT_Tracking/trackers/YA/YABlobber.cpp $MT_SRC_DIR/MT_Core/support/mathsupport.cpp $MT_SRC_DIR/MT_Tracking/cv/MT_HungarianMatcher.cpp"
CV_FLAGS=`pkg-config opencv --cflags`
CV_LIBS=`pkg-config opencv --libs`

MT_LIBS_DIR=~/src/MADTraC/build/lib
MT_LIBS="${MT_LIBS_DIR}/libMT_Core.a ${MT_LIBS_DIR}/libMT_Tracking.a"

DSGYA_SRCS="../../src/BiCC.cpp ../../src/DSGYA_Segmenter.cpp ../../src/DSGYBlobber.cpp"

HUNG_SRCS="$MT_SRC_DIR/MT_Tracking/3rdparty/libhungarian-0.3/hungarian.c $MT_SRC_DIR/MT_Tracking/3rdparty/libhungarian-0.3/makeprob.c"

gcc -c $HUNG_SRCS -I $MT_INC_DIR
ar rs libHungarian.a hungarian.o makeprob.o

#g++ testDSGYBlobber.cpp ../src/DSGYBlobber.cpp $OTHERSRCS $CV_FLAGS -I $MT_INC_DIR -I ../src $CV_LIBS libHungarian.a -o testDSGYBlobber
#g++ testDSGYBlobberWHist.cpp BiCC.cpp ../src/DSGYBlobber.cpp $OTHERSRCS $CV_FLAGS -I $MT_INC_DIR -I ../src $CV_LIBS libHungarian.a -o testDSGYBlobberWHist

#g++ testDSGYA_Segmenter.cpp BiCC.cpp DSGYA_Segmenter.cpp ../src/DSGYBlobber.cpp $OTHERSRCS $CV_FLAGS -I $MT_INC_DIR -I ../src $CV_LIBS libHungarian.a -o testDSGYA_Segmenter

g++ debug.cpp $DSGYA_SRCS $CV_FLAGS -I ../../src -I $MT_INC_DIR $CV_LIBS $MT_LIBS -o debug