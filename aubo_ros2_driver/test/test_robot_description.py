import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aubo_ros2_driver.robot_description import configure_hardware

XML = '<robot><link name="base"/><ros2_control name="aubo" type="system"><hardware><plugin>aubo_driver/AuboHardwareInterface</plugin></hardware><joint name="joint"/></ros2_control></robot>'


class HardwareDescription(unittest.TestCase):
    def test_fake_never_loads_real_driver(self):
        root = ET.fromstring(configure_hardware(XML, True, '192.0.2.1', .01, .01, 1))
        self.assertEqual(root.findtext('ros2_control/hardware/plugin'), 'mock_components/GenericSystem')
        self.assertIsNone(root.find('ros2_control/hardware/param[@name="robot_ip"]'))
        self.assertIsNotNone(root.find('ros2_control/joint'))

    def test_motion_status_is_declared_for_real_and_mock(self):
        for fake, expected in ((True, '0.0'), (False, '1.0')):
            xml = configure_hardware(XML, fake, '192.0.2.1', .01, .01, 1)
            root = ET.fromstring(xml)
            path = 'ros2_control/gpio[@name="motion_status"]/state_interface[@name="execution_state"]/param[@name="initial_value"]'
            self.assertEqual(root.findtext(path), expected)
            twice = ET.fromstring(configure_hardware(xml, fake, '192.0.2.1', .01, .01, 1))
            self.assertEqual(len(twice.findall('ros2_control/gpio[@name="motion_status"]')), 1)

    def test_mock_sensor_setting_is_forwarded(self):
        root = ET.fromstring(configure_hardware(XML, True, '', .01, .01, 1, 'true'))
        self.assertEqual(root.findtext('ros2_control/hardware/param[@name="mock_sensor_commands"]'), 'true')
        self.assertEqual(root.findtext('ros2_control/hardware/param[@name="mock_gpio_commands"]'), 'true')

    def test_real_hardware_receives_explicit_parameters(self):
        root = ET.fromstring(configure_hardware(XML, False, '192.0.2.1', .01, .008, 2))
        self.assertEqual(root.findtext('ros2_control/hardware/plugin'), 'aubo_driver/AuboHardwareInterface')
        params = {p.attrib['name']: p.text for p in root.findall('ros2_control/hardware/param')}
        self.assertEqual(params, dict(robot_ip='192.0.2.1', control_period='0.01', servo_period='0.008', servo_mode='2'))

    def test_missing_or_ambiguous_system_is_rejected(self):
        for xml in ('<robot/>', '<robot><ros2_control/><ros2_control/></robot>'):
            with self.assertRaises(ValueError):
                configure_hardware(xml, True, '', .01, .01, 1)


if __name__ == '__main__':
    unittest.main()
