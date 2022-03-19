#!/usr/bin/env python

from setuptools import setup

setup(
    name='cq-cam',
    version='0.1',
    description='Cadquery CAM',
    author='Matti Eiden',
    author_email='snaipperi@gmail.com',
    packages=['cq_cam'],  #same as name
    install_requires=['pyclipper', 'numpy'],
)
