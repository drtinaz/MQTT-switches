This service adds virtual switches to the gx device GUI v2 switch panel (this service was tested on a cerbo GX, but quite possibly will work on the ekrano and venus running on rpi as it does not interact with the gpio's. This service creates dbus services which natively interact with venus os)
that can be used to control external relay modules via mqtt. 
This can be manually installed as a service, or installed via Kevin Windrems
setup helper. The simplest method is setup helper. 

*********** MANUAL INSTALLATION *************
1. Download this package to the /data directory
2. If this is the first time being installed, run the configuration script ( /data/MQTT-Switches/config.py )
3. Install and start the service mqtt_switches.py


************ INSTALL USING KEVINS SETUP HELPER **************
1. Install Kevins setup helper ( https://github.com/kwindrem/SetupHelper )
2. Goto settings/package manager and turn off 'auto install'
3. From the package manager menu click on inactive packages and click 'new' at the top
4. enter the following:
5. package name : MQTT-switches
6. github user : drtinaz
7. github branch or tag : beta
8. click proceed
9. now go to 'active packages' and click on MQTT-switches and download
10. If this is the first time being installed, run the configuration script from terminal ( /data/MQTT-Switches/config.py )
11. you can now re-enable 'auto install' in setup helper menu or click 'install' in the active packages

************ PACKAGE UPDATES **********************

If you choose the manual installation method, updates to this service or venus firmware will require a re-install.
If you choose the setup helper method, updates to this service or venus firmware will automatically reinstall this service.
If you need to change any of the settings, number of devices, or number of switches...rerun the config script.
