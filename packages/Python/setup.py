from setuptools import setup

setup(
    name = "modern_robotics",
    version = "1.0.0",
    author = "Huan Weng, Mikhail Todes, Jarvis Schultz, Bill Hunt",
    author_email = "huanweng@u.northwestern.edu",
    description = ("Modern Robotics: Mechanics, Planning, and Control: Code Library"),
    license = "MIT",
    keywords = "kinematics robotics dynamics",
    url = "http://modernrobotics.org/",
    packages=['modern_robotics'],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Utilities",
        "License :: OSI Approved :: BSD License",
    ],
    install_requires=[
        'numpy',
        'scipy',
    ],
    platforms='Linux, Mac, Windows',
)
