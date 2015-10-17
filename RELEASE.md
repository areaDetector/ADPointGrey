ADPointGrey Releases
==================

The latest untagged master branch can be obtained at
https://github.com/areaDetector/ADPointGrey.

Tagged source code releases from R2-0 onward can be obtained at 
https://github.com/areaDetector/ADPointGrey/releases.

Tagged prebuilt binaries from R2-0 onward can be obtained at
http://cars.uchicago.edu/software/pub/ADPointGrey.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============

R2-2 (17-October-2015)
----
* Updated from version 2.7.3 to 2.8.3 of the Flycap2 library.
* Changed driver so that corrupt images are discarded and not passed to plugins.
* Changed the default values of StrobeDelay and StrobeDuration from 1 second to 0.01 second.
  1 second is larger than the maximum supported value and caused errors at iocInit.
* Displays (pointGrey.[adl,edl,ui,opi])
  * Changed the format of the StrobeDelay and StrobeDuration display widgets from decimal to
    exponential because they can range from microseconds to tens of milliseconds.
  * Added widget for FrameRateOnOff to the main pointGrey.adl display because this is frequently changed.
  * Fixed size of text widgets in adl file to improve conversion to caQtDM.


R2-1-1 (22-April-2015)
----
* Added missing arguments when loading pointGrey.template in st.cmd.


R2-1 (16-April-2015)
----
* Added support for OnOff and OnePush for camera properties.
* Improved pointGreyProperties screen so that widgets are hidden for unsupported features.
* Fixed bug in the white balance blue control.
* Fixed bug when memoryChannel > 0 was specified in pointGreyConfig().  The bug
  caused the driver to fail to embed the time stamp and frame counter information in the 
  images so the UniqueId was not updating.  This in turn caused ImageJ to not display new images.
* Updated from version 2.6.3 to 2.7.3 of the Flycap2 library.
* Changes for compatibility with ADCore R2-2.


R2-0 (4-April-2014)
----
* New driver added in this release.


Future Releases
===============
* Add performance to documentation
* Fix problem BlackFly camera not working on Linux when restarting IOC if any images were acquired.
* Make Hybrid time stamp a blend of EPICS and camera when seconds and microseconds are 
  not supported.  Set time to agree on first frame, use camera to update time after that. 
