#!/usr/bin/env python3
import subprocess
import sys
import os
DIR = os.path.dirname(os.path.abspath(__file__))

domain_file=sys.argv[1]
problem_file=sys.argv[2]
plan_file=sys.argv[3]
cmd =DIR+"/../../temporal-planning/fd_copy/fast-downward.py --build release64 --alias seq-sat-lama-2011 --overall-time-limit 3600s --overall-memory-limit 4096 --plan-file "+plan_file+" "+domain_file+" "+problem_file
print (cmd.split())
subprocess.call(cmd.split())
