#!/usr/bin/env bash

#EDITABLE=0
#while getopts 'e' opt; do
#    case $opt in
#        e) EDITABLE=1 ;;
#        *) echo 'Error in command line parsing' >&2
#           exit 1
#    esac
#done
#shift "$(( OPTIND - 1 ))"
#
#COMMAND="pip install"
#if [[ "$EDITABLE" == 1 ]]; then
#  COMMAND="pip install -e"
#fi
#echo "$COMMAND"
#
#eval $COMMAND src/controllers
#eval $COMMAND src/interfaces
#eval $COMMAND src/robots
#eval $COMMAND src/simulation

pip install src/controllers
pip install src/interfaces
pip install src/robots
pip install src/simulation
