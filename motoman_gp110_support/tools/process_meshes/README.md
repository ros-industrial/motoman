# Mesh conversion

## Visual meshes

You can use the tool `stl_transform` (install from https://github.com/AllwineDesigns/stl_cmd) to process
original meshes saved from Yaskawa 3D model files (STEP file) as individual STL files named after
corresponding STEP file parts. You can prepare those individual STL files by opening the STEP file
in Fusion 360 and saving each component as an STL (right click on the component and choose Save as STL).

Save those files in the subdirectory `input` of the directory named after specific robot manipulator
variant:

* BASE_AXIS.stl
* S_AXIS.stl
* L_AXIS.stl
* U_AXIS.stl
* R_AXIS.stl
* B_AXIS.stl
* T_AXIS.stl

Now run the `./process1a-stlcmd.sh` script. It will do following things with all meshes:

* scale the mesh so that the units are "meters"
* rotate the mesh so that the Z+ axis points up and X+ axis points in the correct direction
* translate the origin point so that it's in the point of connection of the link to the previous link
* save the file with proper file name used in the URDF to `output` subdirectory

On Ubuntu 16.04 you can install stl_transform by downloading the 18.04 binary package `stlcmd` from
https://launchpad.net/ubuntu/bionic/amd64/stlcmd/1.1-1 and installing it with `dpkg -i <PACKAGE_FILENAME>`.

Then run `./process1b-simplify.sh` script to simplify visual meshes to 50% of faces. It uses MeshLab, see
comments about it below in section about collision meshes.

After processing just copy files from `output_visual` subdirectory to the subdirectory named after specific
robot manipulator variant of the `meshes` directory of this package. Use correct subdirectory `visual`.

## Collision meshes

You can use MeshLab to process visual meshes (created in previous step) to collision meshes automatically.
Unfortunately it's not as simple as processing visual meshes mentioned above. There are more versions of
MeshLab and they use different names and in some cases parameters for some of the filters used to do this
conversion. Newer versions of MeshLab are not so easy to get on Ubuntu. Meshlab 2016.12 from this Ubuntu PPA
works: https://launchpad.net/~zarquon42/+archive/ubuntu/meshlab.

Even with the right version of MeshLab some of the filters coredump when used from command line (through
meshlabserver) :(

Originally I was planning to do a convex hull, some simplification, inversion of faces orientation (sometimes
needed, depends probably on the location of the origin point, if it's on the inside of the mesh body or not)
and then do an offset to get slightly "bigger" mesh for collision checking. The filter used to perform the
offset (Uniform Mesh Resampling) always coredumps when used from command line. Therefore I made the collision
meshes without the offset for now.

You can run `./process2.sh` to process all visual meshes (from previous step) into collision meshes, then copy
them to correct `collision` subdirectory of this package.
