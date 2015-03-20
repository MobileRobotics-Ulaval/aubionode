#!/bin/bash
clear

./configure
make
wait

cd tests
make
