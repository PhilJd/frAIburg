#!/bin/bash


if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit 1
fi

update-alternatives --set gcc "/usr/bin/gcc-4.8"
update-alternatives --set g++ "/usr/bin/g++-4.8"

gcc --version
g++ --version

echo "gcc/g++ set to version 4.8"