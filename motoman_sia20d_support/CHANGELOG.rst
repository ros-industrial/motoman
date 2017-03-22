^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motoman_sia20d_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2017-03-21)
------------------
* No changes

0.3.6 (2017-03-20)
------------------
* sia20: add 'base' coordinate frame to xacro. For `#45 <https://github.com/ros-industrial/motoman/issues/45>`_.
  This corresponds to the Motoman 'Robot Frame' (not the 'Base Frame'). This
  frame has its origin at the intersection of the S axis and a horizontal
  plane going through the L axis.
* Fix for issue `#106 <https://github.com/ros-industrial/motoman/issues/106>`_: use full-precision PI for tool0 transform (`#110 <https://github.com/ros-industrial/motoman/issues/110>`_)
  * sia10f: don't round PI in joint_t-tool0 transform.
  Leads to incorrect 'base->tool0' transform orientations.
  * sia20: don't round PI in joint_t-tool0 transform. Fix `#106 <https://github.com/ros-industrial/motoman/issues/106>`_.
  Leads to incorrect 'base->tool0' transform orientations.
* Fix for issue `#102 <https://github.com/ros-industrial/motoman/issues/102>`_: fix incorrect joint limits in SIA20 urdf/xacro (`#109 <https://github.com/ros-industrial/motoman/issues/109>`_)
  * sia20d: fix joint limits S, R & T. Fix `#102 <https://github.com/ros-industrial/motoman/issues/102>`_.
  This floors PI, instead of rounding. This makes the joint limits slightly
  stricter than what is specced, but will avoid OOB errors on the controller.
  * sia20d: make all joint limits correspond to specsheet.
  All values truncated after the 4th digit.
* Contributors: G.A. vd. Hoorn, gavanderhoorn

0.3.5 (2016-07-03)
------------------
* No changes

0.3.4 (2016-07-03)
------------------
* support: mark SIAx(d|f) pkgs as deprecated.
  And point users to the 'motoman_sia_support' package, which will be
  introduced in Jade.
* Contributors: Shaun Edwards, gavanderhoorn, thiagodefreitas

0.3.3 (2014-02-07)
------------------
* No changes

0.3.2 (2014-01-31)
------------------
* Added build dependency on roslaunch to address missing roslaunch check missing macro
* Contributors: Shaun Edwards

0.3.1 (2014-01-30)
------------------
* Synchronized versions for bloom release
* Corrected motoman robot name (added motoman prefix)
* Added tool0 to sia20.  Matches motoman tool0
* Contributors: Shaun Edwards
