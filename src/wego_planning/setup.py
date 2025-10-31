#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Setup configuration for wego_planning package
# 빈 패키지 설정
setup_args = generate_distutils_setup(
    packages=[],
    package_dir={'': 'src'},
)

setup(**setup_args)

