#!/bin/bash

if [ $(dpkg-query -W -f='${Status}' scons 2>/dev/null | grep -c "ok installed") -eq 0 ];
then
  sudo apt-get install scons;
fi

if python -c 'import pkgutil; exit(not pkgutil.find_loader("pddlpy"))'; then
   echo "pddlpy FOUND"
else
    echo 'pddlpy not found'
    sudo pip install pddlpy
fi

cd ..
cd VAL
make clean
make
cd ..
#git clone https://github.com/aig-upf/universal-pddl-parser.git
cd universal-pddl-parser
scons
cd ..
#git clone https://github.com/aig-upf/temporal-planning.git
cd temporal-planning
python fd_copy/build.py release64
scons
cd ..
