from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['pythonLibs'],
    package_dir={'pythonLibs': 'pythonLibs'}
)

setup(**setup_args)
