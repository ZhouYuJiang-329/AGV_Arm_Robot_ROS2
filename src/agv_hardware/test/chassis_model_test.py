"""Check the launch arguments, expanded production model and ROS hardware parser."""
import pathlib
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from launch.frontend import Parser


def main():
    source = pathlib.Path(sys.argv[2])
    root, parser = Parser.load(str(source / "agv_bringup/launch/agv.launch.xml"))
    parser.parse_description(root)  # Validate ROS launch syntax without starting any node.
    launch = ET.parse(source / "agv_bringup/launch/agv.launch.xml").getroot()
    parameters = {"chassis_ip": "127.0.0.1", "agv_max_linear_speed": "0.45",
                  "agv_max_angular_speed": "0.6", "agv_init_set_do": "false",
                  "agv_init_do_id": "3", "agv_init_do_status": "false",
                  "agv_init_relocate": "false"}
    declarations = {arg.attrib["name"] for arg in launch.findall("arg")}
    command = launch.find("./node[@pkg='robot_state_publisher']/param").attrib["value"]
    assert set(parameters) <= declarations
    assert all(f"{key}:=$(var {key})" in command for key in parameters)
    assert not any(node.attrib.get("exec") == "robot_move_node" for node in launch.findall("node"))
    model = str(source / "agv_description/urdf/agv_robot.urdf.xacro")
    default = ET.fromstring(subprocess.check_output([sys.argv[3], model]))
    defaults = {p.attrib["name"]: p.text for p in default.findall(".//ros2_control/hardware/param")}
    assert defaults["chassis_ip"] == "192.168.1.109"
    assert defaults["agv_init_set_do"] == defaults["agv_init_relocate"] == "true"
    xml = subprocess.check_output([sys.argv[3], model] + [f"{k}:={v}" for k, v in parameters.items()])
    with tempfile.TemporaryDirectory(prefix="agv-model-") as directory:
        path = pathlib.Path(directory) / "agv.urdf"
        path.write_bytes(xml)
        subprocess.run([sys.argv[1], "--model", str(path)], check=True, timeout=15)
    print("PASS: launch -> xacro -> HardwareInfo -> chassis configuration")


if __name__ == "__main__":
    main()
