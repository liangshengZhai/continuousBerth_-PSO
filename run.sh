#!/bin/zsh

g++ -std=c++11 -O2 -o main main.cpp PSO/particle.cpp
if [[ $? -eq 0 ]]; then
    echo "Compile success. Running program..."
    ./main
else
    echo "Compile failed."
fi
