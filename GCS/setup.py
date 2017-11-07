#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['src/pre_flight_node.py', 'src/web_interface_node.py', 'src/gsm_talker.py'],
)

setup(**d)
