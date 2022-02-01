#/bin/bash

stl_transform -rx 90 -rz 180 -s 0.001 input/BASE_AXIS.stl output_visual/base_link.stl
stl_transform -rx 90 -rz 180 -s 0.001 -tz -0.65 input/S_AXIS.stl output_visual/link_1_s.stl
stl_transform -rx 90 -rz 180 -s 0.001 -tx -0.325 -tz -0.65 input/L_AXIS.stl output_visual/link_2_l.stl
stl_transform -rx 90 -rz 180 -s 0.001 -tx -0.325 -tz -1.8 input/U_AXIS.stl output_visual/link_3_u.stl
stl_transform -rx 90 -rz 180 -s 0.001 -tx -0.325 -tz -2.1 input/R_AXIS.stl output_visual/link_4_r.stl
stl_transform -rx 90 -rz 180 -s 0.001 -tx -1.915 -tz -2.1 input/B_AXIS.stl output_visual/link_5_b.stl
stl_transform -rx 90 -rz 180 -s 0.001 -tx -2.140 -tz -2.1 input/T_AXIS.stl output_visual/link_6_t.stl
