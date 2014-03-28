^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motoman_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2014-02-07)
------------------
* No changes

0.3.2 (2014-01-31)
------------------
* No changes

0.3.1 (2014-01-30)
------------------
* Synchronized versions for bloom release
* driver: move DEPENDS to CATKIN_DEPENDS. Fix `#24 <https://github.com/shaun-edwards/motoman/issues/24>`_.
* driver: link against catkin_LIBRARIES. Fix `#23 <https://github.com/shaun-edwards/motoman/issues/23>`_.
* driver: avoid hardcoded python path. Fix `#19 <https://github.com/shaun-edwards/motoman/issues/19>`_.
* Update move_to_joint.py
* Add proper install targets to driver pkg.
  This fixes `#10 <https://github.com/shaun-edwards/motoman/issues/10>`_.
* Added binaries of motoplus driver.  These can be directly loaded on the controller
* Added controller specific INFORM files
* Commiting motoplus changes required to support DX100 using new incremental motion interface
* Renamed fs100 package to motoman_driver.  The new package now contains drivers for all controllers.  Package name reflects new naming convention
* Contributors: Shaun Edwards, Thomas Timm Andersen, gavanderhoorn
