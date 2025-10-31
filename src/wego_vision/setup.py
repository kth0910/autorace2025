#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Setup configuration for wego_vision package
# 빈 패키지 설정 (메시지만 사용)
setup_args = generate_distutils_setup(
    packages=[],
    package_dir={'': 'src'},
)

setup(**setup_args)

