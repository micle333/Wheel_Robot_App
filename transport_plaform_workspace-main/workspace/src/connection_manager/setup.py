from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['pythonLibsTCP'],
    package_dir={'pythonLibsTCP': 'pythonLibsTCP'}
)

setup(**setup_args)
