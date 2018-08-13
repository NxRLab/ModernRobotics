from setuptools import setup

exec(open('modern_robotics/__version__.py').read())

setup(
    name = "modern_robotics",
    version = __version__,
    author = "Huan Weng, Mikhail Todes, Jarvis Schultz, Bill Hunt",
    author_email = "huanweng@u.northwestern.edu",
    description = ("Modern Robotics: Mechanics, Planning, and Control: Code Library"),
    license = "MIT",
    keywords = "kinematics robotics dynamics",
    url = "http://modernrobotics.org/",
    packages=['modern_robotics'],
    classifiers=[
        "Development Status :: 3 - Alpha",
	"Intended Audience :: Education",
	"Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
	"Natural Language :: English",
	"Programming Language :: Python :: 2",
	"Programming Language :: Python :: 3",
	"Topic :: Education",
	"Topic :: Scientific/Engineering",
    ],
    install_requires=[
        'numpy',
    ],
    platforms='Linux, Mac, Windows',
)
