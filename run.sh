#!/usr/bin/env bash

pushd ./bin/*.app/Contents/MacOS/
LIBFREENECT2_PIPELINE=cl ./Kinect2TestDebug
popd

