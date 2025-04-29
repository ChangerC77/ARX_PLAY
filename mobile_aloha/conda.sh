#!/bin/bash

sudo apt install -y python3.8
sudo apt install -y python3-pip python3-venv

conda create -n arx python=3.8 -y
conda activate arx

pip install --upgrade pip