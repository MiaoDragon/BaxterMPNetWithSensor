#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['baxter_mpnet_with_sensor'],
     package_dir={'': 'src'}
)

setup(**setup_args)
