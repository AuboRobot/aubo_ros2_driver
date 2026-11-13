"""Apply driver-owned hardware settings to the public description model."""
import xml.etree.ElementTree as ET


def hardware_name(xml):
    systems = ET.fromstring(xml).findall("ros2_control")
    if len(systems) != 1 or not systems[0].get("name"):
        raise ValueError("Expected exactly one named ros2_control system")
    return systems[0].get("name")


def configure_hardware(xml, use_fake_hardware, robot_ip, control_period,
                       servo_period, servo_mode, fake_sensor_commands="false"):
    root = ET.fromstring(xml)
    systems = root.findall("ros2_control")
    if len(systems) != 1:
        raise ValueError("Expected exactly one ros2_control system")
    hardware = systems[0].find("hardware")
    if hardware is None:
        raise ValueError("Missing ros2_control hardware")
    hardware.clear()
    ET.SubElement(hardware, "plugin").text = (
        "mock_components/GenericSystem" if use_fake_hardware
        else "aubo_driver/AuboHardwareInterface")
    parameters = ({"mock_sensor_commands": fake_sensor_commands,
                   "mock_gpio_commands": fake_sensor_commands, "state_following_offset": "0.0"}
                  if use_fake_hardware else {
                      "robot_ip": robot_ip, "control_period": control_period,
                      "servo_period": servo_period, "servo_mode": servo_mode})
    for name, value in parameters.items():
        ET.SubElement(hardware, "param", name=name).text = str(value)
    status = systems[0].find('gpio[@name="motion_status"]')
    if status is None:
        status = ET.SubElement(systems[0], "gpio", name="motion_status")
    interface = status.find('state_interface[@name="execution_state"]')
    if interface is None:
        interface = ET.SubElement(status, "state_interface", name="execution_state")
    initial = interface.find('param[@name="initial_value"]')
    if initial is None:
        initial = ET.SubElement(interface, "param", name="initial_value")
    initial.text = "0.0" if use_fake_hardware else "1.0"
    return ET.tostring(root, encoding="unicode")
