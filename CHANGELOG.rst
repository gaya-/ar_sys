^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ar_sys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix the single marker coordinates to match ROS
* Introducing system viewer; a node that is capable of handling input from multi-cameras and display all the boards and virtual relative points in 3D using Rviz
* Replaced arrays and its count by vectors of enums
* Fixed the rotation of ArUco and OpenCV coordinates to match the camera and ROS specifications
* Enhanced time and source tracking of images
* Prevented a buffer overflow and made a naming convention
* Enhanced the multi detector performance by drawing the markers once, calculating the markers once using the minimum indicated size
* Fixed a bug in identification due to changes made to the orginal image
* Added launch command for the multi boards detector
* Added multi-boards detection capability
* Added a test board image for testing purposes
* Initial revision: single board detection
* Contributors: Hamdi Sahloul
