This model comes in two versions 'Not-equipped' and 'Equipped'. The 
equipped version has reduced motion range to account for restrictions
from extra hardware and cables routed outside of the robot. To use the 
'Equipped' version, edit the ./urdf/gp165r.xacro file and change the 
reference to 'gp165r_macro.xacro' file to 'gp165r_macro_equipped.xacro'

Also note that the 'Equipped' version has a reduce payload of 150 kg
and lower allowable moment and inertia for the wrist joints. Please 
refer to the manual YASKAWA MOTOMAN-GP165R,GP200R INSTRUCTIONS 
(HW1484396) 186044-1CD for further details.