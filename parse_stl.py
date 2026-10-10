import struct
import sys

def parse_stl(filename):
    with open(filename, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')
        
        for _ in range(num_triangles):
            chunk = f.read(50)
            if len(chunk) < 50:
                break
            verts = struct.unpack('<9f', chunk[12:48])
            for i in range(3):
                x, y, z = verts[i*3], verts[i*3+1], verts[i*3+2]
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
                min_z = min(min_z, z)
                max_z = max(max_z, z)
                
    print(f"Bounds: X:[{min_x:.3f}, {max_x:.3f}], Y:[{min_y:.3f}, {max_y:.3f}], Z:[{min_z:.3f}, {max_z:.3f}]")
    print(f"Size: X:{max_x-min_x:.3f}, Y:{max_y-min_y:.3f}, Z:{max_z-min_z:.3f}")
    print(f"Center: X:{(min_x+max_x)/2:.3f}, Y:{(min_y+max_y)/2:.3f}, Z:{(min_z+max_z)/2:.3f}")

parse_stl(sys.argv[1])
