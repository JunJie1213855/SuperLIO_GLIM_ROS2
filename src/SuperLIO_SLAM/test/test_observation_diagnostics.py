"""诊断统计的数值测试，不依赖 ROS graph 或正在运行的 LIO。"""
import importlib.util
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

script = Path(__file__).resolve().parents[3] / "tools" / "plot_observation_residuals.py"
spec = importlib.util.spec_from_file_location("observation_diagnostics", script)
diagnostics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostics)


class DiagnosticTest(unittest.TestCase):
    def test_weighted_information_and_cost_are_not_counts(self):
        dtype = [(name, "f4") for name in ("residual", "residual_after", "information", "nearest_distance",
                                          "jrx", "jry", "jrz", "jtx", "jty", "jtz")] + [("kind", "u4")]
        data = np.zeros(3, dtype=dtype)
        data["kind"] = [1,1,2]
        data["residual"] = [.01,-.02,.1]
        data["residual_after"] = [.005,-.01,.05]
        data["information"] = [1000,1000,25]
        data["nearest_distance"] = [.2,.3,.1]
        data["jtx"] = [1,0,0]; data["jty"] = [0,1,0]; data["jtz"] = [0,0,1]
        stats = diagnostics.residual_stats(data)
        self.assertEqual(stats["plane_count"], 2)
        self.assertAlmostEqual(stats["point_information_fraction"], 25/2025)
        self.assertAlmostEqual(stats["point_cost_fraction"], 1/3, places=6)
        self.assertAlmostEqual(stats["total_translation_ratio"], .025)
        self.assertEqual(stats["plane_translation_ratio"], 0)
        self.assertEqual(stats["pose_information_ratio"], 0)  # 转动仍完全未约束。
        self.assertAlmostEqual(stats["plane_mean_signed_m"], -.005)
        self.assertTrue(np.isnan(diagnostics.residual_stats(data[:0])["plane_rmse_m"]))

    def test_cloud_decoder_respects_row_padding(self):
        fields = [SimpleNamespace(name=name, offset=i*4, datatype=7 if name!="kind" else 6)
                  for i,name in enumerate(("x","residual","information","kind"))]
        raw = bytearray(2*40)
        for row in range(2):
            for col in range(2):
                struct.pack_into("<fffI",raw,row*40+col*16,row*2+col,.1,1000,1+col)
        msg = SimpleNamespace(fields=fields,point_step=16,row_step=40,width=2,height=2,
                              is_bigendian=False,data=raw)
        result = diagnostics.cloud_array(msg)
        np.testing.assert_equal(result["x"],[0,1,2,3])
        np.testing.assert_equal(result["kind"],[1,2,1,2])


if __name__ == "__main__": unittest.main()
