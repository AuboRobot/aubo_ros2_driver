import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aubo_ros2_driver.control_profile import resolve_update_rate, resolve_servo_period

class Profiles(unittest.TestCase):
    def test_auto_es_and_calibrated(self):
        for model in ('aubo_ES3','aubo_ES5','aubo_ES3_calibrated'):
            self.assertEqual(resolve_update_rate(model,'auto','',200),100)
    def test_auto_preserves_non_es_yaml(self):
        self.assertEqual(resolve_update_rate('aubo_i5','auto','',125),125)
    def test_explicit_controller_family_overrides_arm_model(self):
        self.assertEqual(resolve_update_rate('aubo_ES3','legacy','',100),200)
        self.assertEqual(resolve_update_rate('aubo_i5','es','',200),100)
    def test_explicit_rate_wins(self):
        self.assertEqual(resolve_update_rate('aubo_ES3','es','200',100),200)
    def test_custom_preserves_yaml(self):
        self.assertEqual(resolve_update_rate('aubo_ES3','custom','',125),125)
    def test_auto_execution_period_matches_rate(self):
        for rate in (100,110,120,200):
            self.assertEqual(resolve_servo_period('auto',rate),1/rate)
    def test_fixed_period_preserves_legacy(self):
        self.assertEqual(resolve_servo_period('0.01',200),.01)
    def test_invalid_servo_period(self):
        for value in ('nan','inf','-1','0','.0001','.1'):
            with self.assertRaises(ValueError):resolve_servo_period(value,100)
    def test_invalid_rate_or_profile(self):
        for value in ('0','-1','24','501','nan','100.1'):
            with self.assertRaises(ValueError):resolve_update_rate('aubo_ES3','auto',value,200)
        with self.assertRaises(ValueError):resolve_update_rate('aubo_ES3','unknown','',200)

if __name__=='__main__':unittest.main()
