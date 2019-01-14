#!/bin/bash

if [ $(dpkg-query -W -f='${Status}' scons 2>/dev/null | grep -c "ok installed") -eq 0 ];
then
  sudo apt-get install scons;
fi

if python -c 'import pkgutil; exit(not pkgutil.find_loader("pddlpy"))'; then
   echo "pddlpy FOUND"
else
    echo 'pddlpy not found'
    sudo -H pip3 install pddlpy
fi

