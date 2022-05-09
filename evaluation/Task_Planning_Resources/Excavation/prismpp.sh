#!/bin/sh

# Startup script for PRISM preprocessor (beta)

# Launch using main PRISM script
PRISM_MAINCLASS="prism.Preprocessor"
export PRISM_MAINCLASS
#/Applications/prism-games-2.1-osx64/bin/prism "$@" # Replace 
prism "$@"
