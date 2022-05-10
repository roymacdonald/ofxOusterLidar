# ofxOusterLidar
openFrameworks addon to use the ouster lidars. 

## Install

Make sure you have installed Eigen before trying to compiling. 
The easiest way is to use [Homebrew](https://brew.sh/).

If Eigen is not installed use the following command.
```
brew install eigen
```
Find the install location for brew.
```
brew --prefix eigen
```
In my case it prints 
```
/usr/local/opt/eigen
```
Open the `addon_config.mk file and change the following line so it correctly reflects the route to Eigen.
It should have the following format.

```
ADDON_INCLUDES = [what brew --prefix eigen prints]/include/eigen3
```
Thus look something like
```
ADDON_INCLUDES = /usr/local/opt/eigen/include/eigen3
```

Double check that the path actually exists

### Dependencies
You will need the following addons

* [ofxDropdown](https://github.com/roymacdonald/ofxDropdown/)

Make sure you have it updated

### Running example

Before running the example make sure you update it with Project Generator.
Just drag the example folder into the Project Generator window and hit Update.

The example will run and you will see the following gui.

![](gui.png)
Set the correct IP addresses. Double click on it to edit and press enter when ready.

#### Lidar's IP
Set the Lidar's IP to the IP address to either the the static IP address it might have or the dynamic one assigned by the DHCP server (router) of your network. 

If you are not sure, you can connect to the lidar device http://os-XXXXXXXXXXXX.local or http://os1-XXXXXXXXXXXX.local where XXXXXXXXXXXX is the devices serial number which is printed on the top case of the device.

#### UDP dest IP: 
Most probably this is the IP of your computer. You can set this to a different IP if you want to stream to a different computer. 

If you are using a wired connection make sure you deactivate WiFi.

In order to connect the lidar directly to your computer you need to manually set the IP of both your computer and lidar. 

Once this is set just press connect and wait a few seconds and the point cloud will show up.

#### TIP:
Change the IP addresses in the ofApp.h file so you dont need to set these each time you run the app.


