#/bin/bash

meshlabserver -i output_visual/base_link.stl -o tmp/0.stl -s ../meshlab_01_convexhull.mlx
meshlabserver -i tmp/0.stl -o tmp/1.stl -s ../meshlab_02_invert_faces.mlx
meshlabserver -i tmp/1.stl -o output_collision/base_link.stl -s ../meshlab_03_no_offset_1000t.mlx

echo "Press ENTER to continue"
read

meshlabserver -i output_visual/link_1_s.stl -o tmp/0.stl -s ../meshlab_01_convexhull.mlx
meshlabserver -i tmp/0.stl -o output_collision/link_1_s.stl -s ../meshlab_03_no_offset_1000t.mlx
#don't invert faces for this link
#meshlabserver -i tmp/0.stl -o tmp/1.stl -s ../meshlab_02_invert_faces.mlx
#meshlabserver -i tmp/1.stl -o output_collision/link_1_s.stl -s ../meshlab_03_no_offset_1000t.mlx

echo "Press ENTER to continue"
read

meshlabserver -i output_visual/link_2_l.stl -o tmp/0.stl -s ../meshlab_01_convexhull.mlx
meshlabserver -i tmp/0.stl -o tmp/1.stl -s ../meshlab_02_invert_faces.mlx
meshlabserver -i tmp/1.stl -o output_collision/link_2_l.stl -s ../meshlab_03_no_offset_300t.mlx

echo "Press ENTER to continue"
read

meshlabserver -i output_visual/link_3_u.stl -o tmp/0.stl -s ../meshlab_01_convexhull.mlx
meshlabserver -i tmp/0.stl -o tmp/1.stl -s ../meshlab_02_invert_faces.mlx
meshlabserver -i tmp/1.stl -o output_collision/link_3_u.stl -s ../meshlab_03_no_offset_300t.mlx

echo "Press ENTER to continue"
read

meshlabserver -i output_visual/link_4_r.stl -o tmp/0.stl -s ../meshlab_01_convexhull.mlx
meshlabserver -i tmp/0.stl -o tmp/1.stl -s ../meshlab_02_invert_faces.mlx
meshlabserver -i tmp/1.stl -o output_collision/link_4_r.stl -s ../meshlab_03_no_offset_300t.mlx

echo "Press ENTER to continue"
read

meshlabserver -i output_visual/link_5_b.stl -o tmp/0.stl -s ../meshlab_01_convexhull.mlx
meshlabserver -i tmp/0.stl -o tmp/1.stl -s ../meshlab_02_invert_faces.mlx
meshlabserver -i tmp/1.stl -o output_collision/link_5_b.stl -s ../meshlab_03_no_offset_300t.mlx

echo "Press ENTER to continue"
read

meshlabserver -i output_visual/link_6_t.stl -o tmp/0.stl -s ../meshlab_01_convexhull.mlx
meshlabserver -i tmp/0.stl -o tmp/1.stl -s ../meshlab_02_invert_faces.mlx
meshlabserver -i tmp/1.stl -o output_collision/link_6_t.stl -s ../meshlab_03_no_offset_300t.mlx
