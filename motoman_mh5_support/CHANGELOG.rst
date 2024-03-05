^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motoman_mh5_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2016-07-03)
------------------
* No changes

0.3.4 (2016-07-03)
------------------
* Fixed roslaunch test issues
* mh5: remove 'solid' from binary visual meshes. Fix `#60 <https://github.com/shaun-edwards/motoman/issues/60>`_.
  Also removes executable bit (not needed).
* Added comment with old joint limits and explanation.
* Reduced joint_u limits to values measured from an MH5F.
* Reversed MH5 base mesh.  Was on backwards.
* New concave meshes.  Visuals are full-res, collisions are decimated.
* added missing prefix to links (batch replace)
* Contributors: Dave Hershberger, David Hershberger, Mathias Lüdtke, Shaun Edwards, gavanderhoorn, thiagodefreitas

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
* Generated mh5 and created required urdfs/meshes
* Contributors: Shaun Edwards
