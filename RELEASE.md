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

R2-0
----
* New driver added in this release.

Future Releases
===============
* Add performance to documentation
* Fix problem GigE camera not working on Linux when restarting IOC if any images were acquired.
* Make Hybrid time stamp a blend of EPICS and camera when seconds and microseconds are 
  not supported.  Set time to agree on first frame, use camera to update time after that. 
