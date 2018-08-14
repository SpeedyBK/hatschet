#!/bin/sh

#
# 'example.graphml' and 'example_RM.xml' encode the same modulo scheduling problem as constructed in 'example.cpp'.
#
# The following command uses the HatScheT command-line application to read the dependence graph and the resource
# model from the specified files, and uses the Moovac formulation to compute a schedule. Afterwards, an HTML
# visualization of the schedule is emitted.
#
# (Please run this script from the 'example' directory)
#

cd ..; build/hatschet \
    --graph=example/example.graphml \
    --resource=example/example_RM.xml \
    --scheduler=MOOVAC \
    --html=example/example.html
