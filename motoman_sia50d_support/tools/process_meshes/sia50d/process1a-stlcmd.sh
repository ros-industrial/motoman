#/bin/bash

rm output_visual/*
stl_transform -rx 0 -rz 0 -s 0.001 input/BASE.stl output_visual/base_link.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tz -0.312000 input/S-AXIS.stl output_visual/link_1_s.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.145 -tz -0.54 input/L-AXIS.stl output_visual/link_2_l.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.145 -tz -1.415 input/7-AXIS.stl output_visual/link_3_e.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.145 -tz -1.415 input/U-AXIS.stl output_visual/link_4_u.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.755 -tz -1.415 input/R-AXIS.stl output_visual/link_5_r.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -0.755 -tz -1.415 input/B-AXIS.stl output_visual/link_6_b.stl
stl_transform -rx 0 -rz 0 -s 0.001 -tx -1.105 -tz -1.415 input/T-AXIS.stl output_visual/link_7_t.stl
