# "mr" MATLAB Code Library Instructions #

## Installing the Library ##

Copy the "mr" folder to a known location on your computer. We will call the
path to this folder `$FOLDER_PATH/mr`.

## Importing the Library ##

To import the library, use `addpath` as

```
addpath('$FOLDER_PATH/mr')
```

This process is required for any program using this package.

## Using the Package ##

After importing the library, you should be able to use any function in the 
library. Taking the function `RotInv` for example, you can check the 
description and usage example of this function by running

```
help RotInv
```

As mentioned in the function usage example, you can try using this function
by running

```
R = [0, 0, 1; 1, 0, 0; 0, 1, 0];
invR = RotInv(R);
```

You should get the the variable `invR` whose value is the same as the output
shown in the function usage example.

To check the function list and which chapter in the book those functions 
belong to, use `help` as 

```
help mr
```

## Library Information ##

Author: Huan Weng, Bill Hunt, Mikhail Todes, Jarvis Schultz

Contact: huanweng@u.northwestern.edu

Package Version: 1.1.0

Matlab Version: R2017b

Tested in Matlab R2017b
