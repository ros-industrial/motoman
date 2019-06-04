# Mesh conversion

You can use the tool `stl_cmd` (install from https://github.com/AllwineDesigns/stl_cmd) to process
original meshes saved from Yaskawa 3D model files (STEP file) as individual STL files named after
corresponding STEP file parts.

Just save those files in the subdirectory `input` of the directory where this README file is located:

* BASE_AXIS.stl
* S_AXIS.stl
* L_AXIS.stl
* U_AXIS.stl
* R_AXIS.stl
* B_AXIS.stl
* T_AXIS.stl

And run `./process.sh` script. It will do following things with all meshes:

# scale the mesh so that the units are "meters"
# rotate the mesh so that the Z+ axis points up
# translate the origin point so that it's in the point of connection of the link to the previous link
# save the file with proper file name used in the URDF to `output` subdirectory

After processing just copy files from `output` subdirectory to the `meshes/gp180_120` directory of the package
to correct subdirectory (depends on which type of meshes you just processed - visual or collision).

Alternatively you can process just the visual meshes and then create the collision meshes from already
processed visual meshes.
