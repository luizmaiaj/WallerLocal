#!/bin/bash

mkdir -p build
cd build
cmake ..
make -j$(nproc)

# Copy executable to root directory
cp waller ..