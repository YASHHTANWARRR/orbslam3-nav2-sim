# Meshes

## `burger_deck1.stl`

TurtleBot3 Burger base with the upper two decks removed — bottom plate only.

Derived from `burger_base.stl` in `turtlebot3_gazebo`
(ROBOTIS, Apache-2.0) by cutting every triangle above z = 62 mm.
The source mesh has three decks at 0–60 mm, 80–105 mm, and 130–150 mm;
the cut keeps only the first, leaving 59954 of 96524 triangles.

Regenerate:

```bash
python3 vslam_description/scripts/cut_stl.py \
  /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_common/meshes/bases/burger_base.stl \
  vslam_description/meshes/burger_deck1.stl 62
```

The camera mast in `gz_slim_tb3.sdf.xacro` stands directly on the mesh top
(60.5 mm).
