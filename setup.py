#creates the setup file for rrt mavxim
from setuptools import setup, find_packages
from setuptools.command.build_ext import build_ext
import subprocess
import os
import shutil
import multiprocessing


#creates the function to build the external 


#setup function
setup(
    name="rrt_mavsim",
    version="0.1.0",
    packages=find_packages(include=["eVTOL_BSplines"]),
    include_package_data=True,
    install_requires=["numpy"]
)