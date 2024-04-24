import tempfile

import airo_models
from pydrake.planning import RobotDiagramBuilder

from airo_drake import SingleArmScene, add_floor, add_manipulator, finish_build
from airo_drake.building.manipulator import add_manipulator_from_urdf


def test_single_arm_scene():
    robot_diagram_builder = RobotDiagramBuilder()

    arm_index, gripper_index = add_manipulator(robot_diagram_builder, "ur5e", "robotiq_2f_85")
    add_floor(robot_diagram_builder)

    robot_diagram, context = finish_build(robot_diagram_builder)

    scene = SingleArmScene(robot_diagram, arm_index, gripper_index)

    del robot_diagram_builder

    assert robot_diagram is not None
    assert context is not None
    assert scene is not None
    assert arm_index is not None
    assert gripper_index is not None


def test_add_manipulator_from_urdf():
    robot_diagram_builder = RobotDiagramBuilder()

    arm_urdf_path = airo_models.get_urdf_path("ur5e")

    # define a simple parallel gripper in code for testing
    _gripper_urdf = """
    <?xml version="1.0"?>

<robot name="parallel_gripper">

  <!-- Define materials -->
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1"/>
  </material>

  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

  <link name="left_finger">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

  <link name="right_finger">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

  <!-- Define joints -->
  <joint name="left_joint" type="prismatic">
    <origin xyz="0.05 0 0.075" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="left_finger"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.1"/>
  </joint>

  <joint name="right_joint" type="prismatic">
    <origin xyz="-0.05 0 0.075" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="right_finger"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.1"/>
  </joint>

</robot>
    """
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as urdf_file:
        urdf_file.write(_gripper_urdf.encode())

    gripper_urdf_path = urdf_file.name

    arm_index, gripper_index = add_manipulator_from_urdf(robot_diagram_builder, arm_urdf_path, gripper_urdf_path)
    robot_diagram, context = finish_build(robot_diagram_builder)

    assert arm_index is not None
    assert gripper_index is not None
    assert robot_diagram is not None
    assert context is not None
