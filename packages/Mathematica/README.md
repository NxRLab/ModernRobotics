# "ModernRobotics" Mathematica Package Instructions #

## Installing the Package ##

### Recommended Installation Instructions ###

If you'd like to be able to use this package inside of any notebook, 
regardless of the notebook's location on your filesystem, then you can use 
Mathematica's front end to install this package. Use the following steps:

1. Download this package
2. Click `File -> Install...`
3. Select `Package` for the _Type of Item to Install_
4. Chose `From File...` for the _Source_
5. Navigate to `ModernRobotics.m` and select it as the source for the 
   package
6. The _Install Name_ should default to `ModernRobotics`; if it doesn't then
   fill in the install name to be `ModernRobotics`
7. Choose whether you want the package installed for a single user or for all
   users (may require administrative privileges)
8. Click `OK`

### Manual Installation Instructions ###

If you have any troubles with the front-end installation described above, try
manually installing the package by copying the `ModernRobotics.m` file into 
the following directory.

```sh
$MATHPATH/Applications/ModernRobotics/
```

The value of `$MATHPATH` will depend on your system. You can determine it by
looking at the `$Path` variable inside Mathematica. Here are some paths for
common operating systems:

+ OS-X:                `~/Library/Mathematica/`
+ Linux (Debian):      `~/.Mathematica/`
+ Windows:             `%APPDATA%\Mathematica\`

## Loading the Package ##

After installing the package, load the library from any notebook by running

```
<<ModernRobotics`
```

This process is required for any notebook using this package.

## Using the Package ##

After loading the package, you should be able to use any function in the 
package. Taking the function `RotInv` for example, you can check the 
description and usage example of this function by running

```
?RotInv
```

As mentioned in the function usage example, you can try using this function
by running

```
invR = RotInv[{{0,0,1},{1,0,0},{0,1,0}}]
```

You should get the the same output as shown in the function usage example.

## Using the Package Locally ##

It is possible to use the package locally without installation. Download and
place the package anywhere on your filesystem and then run

```
SetDirectory["<PATH-TO-DIRECTORY-CONTAINING-ModernRobotics.m>"]
```

If the notebook and the package are located in the same directory you could 
use 

```
SetDirectory[NotebookDirectory[]]
```

Note that since the package is not installed, you need to set the directory
shown above in every notebook in which this package is used. Loading is still
required before using.

## Uninstalling the Package ##

To uninstall the package, delete the file `ModernRobotics.wl` or 
`ModernRobotics.m` in the following directory.

```sh
$MATHPATH/Applications/ModernRobotics/
```