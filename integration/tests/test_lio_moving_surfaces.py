"""Exercise FAST-LIO's scan-time filter for observed moving surfaces."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class LioMovingSurfaceTests(unittest.TestCase):
    def test_motion_extrapolation_static_landmarks_and_stale_tracks(self):
        root = Path(__file__).resolve().parents[2]
        source = (root/'external/runtime/FAST_LIO_Hesai/src/laserMapping.cpp').read_text()
        start = source.index('bool on_rejected_surface(')
        end = source.index('\nvoid rejected_surface_cbk', start)
        function = source[start:end]
        program = '''#include <Eigen/Core>
#include <vector>
#include <cassert>
struct PointType { float x,y,z; };
std::vector<Eigen::Vector4f> rejected_surfaces;
struct BoxPointType { float vertex_min[3], vertex_max[3]; };
double lidar_end_time=10.25, rejected_surface_stamp=10., filter_size_map_min=.1, cube_len=1000.;
''' + function + '''
int main() {
  rejected_surfaces.emplace_back(1.f,2.f,.4f,-.2f);
  assert(on_rejected_surface({1.1f,1.95f,.3f}));
  assert(!on_rejected_surface({1.8f,1.95f,.3f}));
  auto boxes = rejected_surface_boxes();
  assert(boxes.size()==1);
  assert(boxes[0].vertex_min[0]<.8 && boxes[0].vertex_max[0]>1.1);
  assert(boxes[0].vertex_min[1]<1.95 && boxes[0].vertex_max[1]>2.1);
  assert(boxes[0].vertex_max[0]<1.8);
  lidar_end_time=10.51;
  assert(rejected_surface_boxes().empty());
  assert(!on_rejected_surface({1.2f,1.9f,.3f}));
  lidar_end_time=9.99;
  assert(!on_rejected_surface({1.f,2.f,.3f}));
  lidar_end_time=10.25;
  rejected_surfaces.clear();
  assert(!on_rejected_surface({1.1f,1.95f,.3f}));
}
'''
        with tempfile.TemporaryDirectory() as temporary:
            cpp = Path(temporary)/'moving.cpp'
            binary = Path(temporary)/'moving'
            cpp.write_text(program)
            subprocess.run(['g++', '-std=c++17', '-I/usr/include/eigen3', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
