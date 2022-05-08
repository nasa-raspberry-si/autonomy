from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['prism-planning', 'plan-translation'],
    package_dir={'': 'src'}
)

setup(**d)
