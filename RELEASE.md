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

R2-8 (3-December-2018)
----
* Updated FlyCap2 SDK on Windows from 2.9.3 to 2.12.3.
  * The most important reason for the upgrade is that there is a bug in 2.9.3 where
    camera serial numbers are limited to 24 bits.  This can cause FlyCap2 to fail to find
    cameras whose serial number starts with 17 and higher, i.e. manufactured in 2017 and later.
  * Linux was left at SDK version 2.9.3 because later versions will not work0 with GCC 4.8 or earlier.
    This means they won't work with RHEL 7 or Centos 7.
  * So far cameras with serial numbers stating with 17 or 18 have worked on Linux for me, 
    but this may just be luck, since it may depend on uninitialized memory.
    Point Grey do provide a utility to "downgrade" the serial number of a camera by subtracting 17
    from the first 2 digits, so this can be used if there are problems with new cameras on Linux.
  * On Windows the name of the required FlyCap2 library now depends on what version of Visual Studio
    is being used.  It is FlyCapture2_v100.lib for VS 2010, FlyCapture2_v120.lib for VS 2013, 
    and FlyCapture2_v140.lib for VS 2015.
    I have not yet found a way to have the Makefile automatically detect the VS version, so the Makefiles 
    will need to be manually edited if VS 2015 is not being used.
* Tweaked medm screens, and thus the autoconverted opi, ui, and edl screens.


R2-7 (2-July-2018)
----
* Fixed camera timestamps to use EPICS epoch, not Posix epoch.
* Fixed problem with libflycapture.so in Makefile on Linux.
* Changed configure/RELEASE files for compatibility with areaDetector R3-3.
* Added support for new PVs in ADCore R3-3 in opi files (NumQueuedArrays, EmptyFreeList, etc.)
* Improved op/*/autoconvert/* files with better medm files and better converters.


R2-6 (29-January-2018)
----
* Removed the SerialNumber, FirmwareVersion, and SoftwareVersion parameters and records,
  since the equivalents are now in ADDriver.h and ADBase.template.
* Removed code that released the lock around the calls to doCallbacksGenericPointer.  
  This was not needed and could cause problems.
* Fixed medm adl files to improve the autoconversion to other display manager files.
* Added op/Makefile to automatically convert adl files to edl, ui, and opi files.
* Updated the edl, ui, and opi autoconvert directories to contain the conversions
  from the most recent adl files.


R2-5 (4-July-2017)
----
* Updated medm screen layout for ADCore R3-0.


R2-4 (21-February-2017)
----
* Updated from version 2.8.3 to 2.9.3 of the Flycap2 library.
* Replaced driver-specific parameters for serial number, firmware version and SDK version with the new
  base class parameters from ADCore R2-6.  Minor change to driver and medm screen.


R2-3 (12-December-2015)
----
* Fixed problem determining the maximum allowed packet size when calling SetFormat7Configuration.  
  This could lead to invalid packet size being passed, and could result in an error message 
  each time the PixelFormat or other Format7 parameters were changed.
* Added support for conversion from Raw12 to Raw16.  This is needed because the Point Grey library 
  does not support conversion from Raw12 to Mono16.  We want to run cameras in Raw12 mode because it is
  faster than Mono12 or Mono16 and we need to convert to a 16-bit format before sending arrays to
  plugins.
* Changed the logic when changing property values.  Previously when changing the integer or absolute value
  of a property the property On/Off setting was changed to On, and Automatic mode was always turned Off.
  This was often not the desired behavior, since if a property was turned off (e.g. FrameRate) then
  it would be automatically turned back on if the AcquirePeriod or FrameRate PV was processed.
  Now the Automatic and On/Off settings are not changed when the value of a property is modified.
  If the Automatic setting is On, or if the On/Off setting is Off then the property value will not 
  be modified.
* Changed the driver to return success rather than an error if a call is made to change an unsupported 
  property.  This eliminates error messages for unsupported properties during iocInit.
* Set the record PHAS field on some records to 1, 2, or 3 to control the order in which they process at 
  iocInit. Some records must process before others when changing PixelMode, etc.
* Set AcquirePeriod to have PINI=NO and info(asynREABACK, "1").  This makes AcquirePeriod slaved to 
  FrameRate, which is the preferred parameter on PointGrey cameras.
* Made the OnOff and AutoMode records for all Point Grey properties have PINI=YES.
* Changed pointGrey.substitutions in the iocPointGrey directory to set PINI=YES or PINI_ABS=YES for 
  most properties, even if they are not supported. The driver now silently ignores unsupported properties, 
  and it is a better example to use for cameras where those properties are supported.
* Added epicsThreadSleep and dbpf commands to st.cmd after iocInit.  These allow time for enum and soft 
  limit callbacks  to occur, and force some records to process again after PixelFormat and other mode 
  records process.   The IOC should now start up with no error messages, and with the actual camera state 
  reflecting the current value of all output records.
* Removed the PGHeartBeat parameters from the class definition, they were not being used.


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
