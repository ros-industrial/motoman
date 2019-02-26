# Motoman

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium%20/%20vendor-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial][] Motoman metapackage. See the [ROS wiki][] page for more information.

The [motoman_experimental][] repository contains additional packages.


## Contents

Branch naming follows the ROS distribution they are compatible with. `-devel`
branches may be unstable. Releases are made from the distribution branches
(`indigo`, `kinetic`).

Older releases may be found in a mirror of the old ROS-Industrial [subversion repository][]
archive.


## ROS Distro Support

|         | Indigo | Kinetic |
|:-------:|:------:|:-------:|
| Branch  | [`indigo-devel`](https://github.com/ros-industrial/motoman/tree/indigo-devel) | [`kinetic-devel`](https://github.com/ros-industrial/motoman/tree/kinetic-devel) |
| Status  |  supported | supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=motoman) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=motoman) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.org/ros-industrial/motoman.svg?branch=kinetic-devel)](https://travis-ci.org/ros-industrial/motoman)

## ROS Buildfarm

|         | Indigo Source | Indigo Debian | Kinetic Source  |  Kinetic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| motoman | [![not released](http://build.ros.org/buildStatus/icon?job=Isrc_uT__motoman__ubuntu_trusty__source)](http://build.ros.org/view/Isrc_uT/job/Isrc_uT__motoman__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__motoman__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__motoman__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__motoman__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__motoman__ubuntu_xenial__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__motoman__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__motoman__ubuntu_xenial_amd64__binary/) |


[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/motoman
[motoman_experimental]: https://github.com/ros-industrial/motoman_experimental
[subversion repository]: https://github.com/ros-industrial/swri-ros-pkg
