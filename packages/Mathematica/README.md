# "ModernRobotics" Mathematica Package Instructions #

This package is the code library for _Modern Robotics: Mechanics, Planning, 
and Control_. [Here](../../doc/MRlib.pdf) is the introduction. For more 
details please see the [website](http://modernrobotics.org/).

If you'd like to be able to use this package inside of any notebook, 
regardless of the notebook's location on your filesystem, then you can use 
Mathematica's front end to install this package. Use the following steps:

1. Click `File -> Install...`
2. Select `Package` for the _Type of Item to Install_
3. Chose `From File...` for the _Source_
4. Navigate to the `ModernRobotics.m` and select it as the source for the 
   package
5. The _Install Name_ should default to `ModernRobotics`; if it doesn't then
   fill in the install name to be `ModernRobotics`.
6. Choose whether you want the package installed for a single user or for all
   users (may require administrative privileges)
7. Click `OK`

Now from any notebook you should be able to load the library by running:

```
<<ModernRobotics`
```

If you'd like to uninstall the package, you can just delete the file 
`ModernRobotics.wl` or `ModernRobotics.m` in the following directory.

```sh
$MATHPATH/Applications/ModernRobotics/
```

The value of `$MATHPATH` will depend on your system. You can determine it by
looking at the `$Path` variable inside Mathematica. Here are some paths for
common operating systems:

+ OS-X:                `~/Library/Mathematica/`
+ Linux (Debian):      `~/.Mathematica/`
+ Windows:             `%APPDATA%\Mathematica\`


## Manual Installation Instructions ##

If you have any troubles with the front-end installation described above, you
can try manually installing the package by copying the `ModernRobotics.m` 
file into the same directory mentioned for uninstalling.


# Using the Package Locally #

If you don't want to install the package at all, it is possible to place it
anywhere on your filesystem and then load it into a notebook using

```
SetDirectory["<PATH-TO-DIRECTORY-CONTAINING-ModernRobotics.m>"]
<<ModernRobotics`
```

If the notebook and the package are located in the same directory you could 
use 

```
SetDirectory[NotebookDirectory[]]
<<ModernRobotics`
```
