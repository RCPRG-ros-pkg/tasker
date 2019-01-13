#!/bin/bash

sudo pip install pddlpy
sudo apt-get install scons
git clone https://github.com/KCL-Planning/VAL
cd VAL
make clean
make
cd ..
git clone https://github.com/aig-upf/universal-pddl-parser.git
cd universal-pddl-parser
scons
cd ..
git clone https://github.com/aig-upf/temporal-planning.git
cd temporal-planning
python fd_copy/build.py release64
scons
cd ..
