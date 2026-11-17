#!/bin/bash
# get the location of the OmniSim binary, even if defined into a relative symlink
# NOTE: on macOS the simulator binary itself is still named Contents/MacOS/webots
# (see TARGET in src/omnisim/Makefile). macOS is an untested platform, so the
# install layout is deliberately left as-is here; only the shell-local names and
# this script's own filename were rebranded.
omnisimBinaryFullPath="$0"
while [ -h "$omnisimBinaryFullPath" ] ; do
  omnisimBinaryFullPath=$(readlink "$omnisimBinaryFullPath")
done
omnisimBinaryDir=$(dirname "$omnisimBinaryFullPath")

# execute the real OmniSim binary in a child process
"$omnisimBinaryDir"/Contents/MacOS/webots "$@" &

omnisim_pid=$!

trap 'kill $omnisim_pid &> /dev/null' EXIT

wait $omnisim_pid
