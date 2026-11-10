"""Exercise the upstream ring feature loop with missing LiDAR returns."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class LioPreprocessTests(unittest.TestCase):
    def test_empty_and_single_point_rings_are_skipped(self):
        root = Path(__file__).resolve().parents[2]
        source = (root/'external/runtime/FAST_LIO_Hesai/src/preprocess.cpp').read_text()
        start = source.index('PointCloudXYZI &pl = pl_buff[j];')
        end = source.index('give_feature(pl, types);', start) + len('give_feature(pl, types);')
        loop = source[start:end]
        program = '''#include <vector>
#include <cmath>
#include <cassert>
using std::vector;
struct Point { double x=1, y=0, z=0; };
struct PointCloudXYZI : vector<Point> {};
struct orgtype { double range=0, dista=0; };
int calls=0;
void give_feature(PointCloudXYZI &pl, vector<orgtype> &types) {
  assert(pl.size()>=2 && types.size()==pl.size());
  assert(types.back().range==1);
  ++calls;
}
int main() {
  PointCloudXYZI pl_buff[4];
  vector<orgtype> typess[4];
  double vx,vy,vz;
  for(int j=0;j<4;++j) pl_buff[j].resize(j);
  for(int j=0;j<4;++j) {
''' + loop + '''
  }
  assert(calls==2);
}
'''
        with tempfile.TemporaryDirectory() as temporary:
            cpp = Path(temporary)/'preprocess.cpp'
            binary = Path(temporary)/'preprocess'
            cpp.write_text(program)
            subprocess.run(['g++', '-std=c++17', '-D_GLIBCXX_ASSERTIONS', str(cpp), '-o', str(binary)], check=True)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
