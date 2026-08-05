#!/usr/bin/env python3
"""Cut a binary STL below a Z height, dropping triangles above it.

Used to strip the upper decks off the TurtleBot3 Burger base mesh, leaving the
bottom plate only. Source mesh is Apache-2.0 from ROBOTIS (turtlebot3_gazebo).

Regenerate the vendored mesh with:
  python3 vslam_description/scripts/cut_stl.py \
    /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_common/meshes/bases/burger_base.stl \
    vslam_description/meshes/burger_deck1.stl 62
"""
import struct
import sys


def cut(src, dst, z_max):
    data = open(src, 'rb').read()
    count = struct.unpack('<I', data[80:84])[0]
    kept = []
    for i in range(count):
        off = 84 + i * 50
        tri = struct.unpack('<12f', data[off:off + 48])
        # tri = (nx,ny,nz, v1x,v1y,v1z, v2x,v2y,v2z, v3x,v3y,v3z)
        if max(tri[5], tri[8], tri[11]) <= z_max:
            kept.append(data[off:off + 50])

    with open(dst, 'wb') as f:
        f.write(b'cut_stl: TurtleBot3 burger base, upper decks removed'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(kept)))
        f.writelines(kept)

    return count, len(kept)


def demo():
    """Self-check: a 2-triangle STL, one above the cut and one below."""
    import tempfile, os
    def tri(z):
        return struct.pack('<12f', 0, 0, 1, 0, 0, z, 1, 0, z, 0, 1, z) + b'\0\0'
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'a.stl'), os.path.join(d, 'b.stl')
        with open(src, 'wb') as f:
            f.write(b'\0' * 80 + struct.pack('<I', 2) + tri(10.0) + tri(90.0))
        total, kept = cut(src, dst, 62)
        assert (total, kept) == (2, 1), (total, kept)
        assert struct.unpack('<I', open(dst, 'rb').read()[80:84])[0] == 1
    print('demo ok')


if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'demo':
        demo()
    else:
        src, dst, z = sys.argv[1], sys.argv[2], float(sys.argv[3])
        total, kept = cut(src, dst, z)
        print(f'{src} -> {dst}')
        print(f'  cut at z={z}mm: kept {kept}/{total} triangles')
