# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['area_common', 'area_command'],
    package_dir={'': 'src'},
    scripts=[],
)

setup(**setup_args)
