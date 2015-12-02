#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['squirrel_object_perception'],
    scripts=['src/squirrel_object_perception/squirrel_look_for_objects.py'],
    package_dir={'': 'src'}
)

setup(**d)
