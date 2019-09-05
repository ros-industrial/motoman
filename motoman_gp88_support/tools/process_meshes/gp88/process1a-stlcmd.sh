#/bin/bash

stl_transform -rx 0 -rz 0 -s 0.001 input/BASE_AXIS.stl output_visual/base_link.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tz -0.54 input/S_AXIS.stl output_visual/link_1_s.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.320 -tz -0.54 input/L_AXIS.stl output_visual/link_2_l.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.320 -tz -1.41 input/U_AXIS.stl output_visual/link_3_u.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.320 -tz -1.62 input/R_AXIS.stl output_visual/link_4_r.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -1.345 -tz -1.62 input/B_AXIS.stl output_visual/link_5_b.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -1.520 -tz -1.62 input/T_AXIS.stl output_visual/link_6_t.stl
