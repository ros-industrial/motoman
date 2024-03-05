^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motoman_sia5d_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2016-07-03)
------------------
* No changes

0.3.4 (2016-07-03)
------------------
* support: mark SIAx(d|f) pkgs as deprecated.
  And point users to the 'motoman_sia_support' package, which will be
  introduced in Jade.
* sia5d: use meshes in support pkg. Fix `#81 <https://github.com/shaun-edwards/motoman/issues/81>`_.
  Stop using those in the config package. That has been deprecated since 2014.
* Contributors: Shaun Edwards, gavanderhoorn, thiagodefreitas

0.3.3 (2014-02-07)
------------------
* sia5d: remove incorrect safety ctrlr tags. Fix `#29 <https://github.com/shaun-edwards/motoman/issues/29>`_.
* Contributors: gavanderhoorn

0.3.2 (2014-01-31)
------------------
* Added build dependency on roslaunch to address missing roslaunch check missing macro
* Contributors: Shaun Edwards

0.3.1 (2014-01-30)
------------------
* Synchronized versions for bloom release
* Added auto-generated sia5d
* Contributors: Shaun Edwards
