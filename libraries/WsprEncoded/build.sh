#!env bash
CXX=g++ cmake -S . -B build -G "MinGW Makefiles"
cmake --build build
