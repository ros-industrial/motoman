^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motoman_sia10d_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2017-03-21)
------------------
* No changes

0.3.6 (2017-03-20)
------------------
* sia10: add 'base' coordinate frame to xacro. For `#45 <https://github.com/ros-industrial/motoman/issues/45>`_.
  This corresponds to the Motoman 'Robot Frame' (not the 'Base Frame'). This
  frame has its origin at the intersection of the S axis and a horizontal
  plane going through the L axis.
  Note: the frame was added manually to the urdf, so the '.urdf' is still out
  of sync with the xacro macro (issue `#107 <https://github.com/ros-industrial/motoman/issues/107>`_).
* Fix for issue `#104 <https://github.com/ros-industrial/motoman/issues/104>`_: fix incorrect joint limits in SIA10 urdf/xacro (`#108 <https://github.com/ros-industrial/motoman/issues/108>`_)
  * sia10: fix joint limits S, R & T. Fix `#104 <https://github.com/ros-industrial/motoman/issues/104>`_.
  This floors PI, instead of rounding. This makes the joint limits slightly
  stricter than what is specced, but will avoid OOB errors on the controller.
  * sia10: make all joint limits correspond to specsheet.
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
* added missing prefix to links (batch replace)
* Contributors: Mathias Lüdtke, Shaun Edwards, gavanderhoorn, thiagodefreitas

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
* Added auto-generated sia10d, added urdf/meshes
* Contributors: Shaun Edwards
