#!/bin/bash

./switchGCC5.sh
apt-get -y purge nvidia-384
apt-get -y install nvidia-384
./switchGCC4.8.sh
