# ofxOusterLidar
openFrameworks addon to use the ouster lidars. 

## Install
Make sure you have installed Eigen before compiling. 
It should be in the following path

```
/usr/local/include/eigen3
```
If it is not installed in that path yet you know it is elsewhere
open the `addon_config.mk` file and change the following line so it correctly reflects the route to Eigen

```
ADDON_INCLUDES = /usr/local/include/eigen3
```
Update the project with Project Generator if you changed this path.

If Eigen is not installed, do so using Homebrew
```
brew install eigen
```

You will need the following addons

* [ofxDropdown](https://github.com/roymacdonald/ofxDropdown/)

Make sure you have it updated