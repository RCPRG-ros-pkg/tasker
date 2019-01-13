#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
domain_file=$1
problem_file=$2
plan_file=$3

echo $domain_file
python $DIR/../temporal-planning/fd_copy/fast-downward.py --build release64 --alias seq-sat-lama-2011 --overall-time-limit 3600s \
		--overall-memory-limit 4096 --plan-file $plan_file $domain_file $problem_file
