import airo_models
import numpy as np
from airo_typing import HomogeneousMatrixType
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagramBuilder

# X_URBASE_ROSBASE is the 180 rotation between ROS URDF base and the UR control box base
X_URBASE_ROSBASE = RigidTransform(rpy=RollPitchYaw([0, 0, np.pi]), p=[0, 0, 0])  # type: ignore

# 180 degree rotation and 1 cm (estimate) offset for the coupling / flange
X_URTOOL0_ROBOTIQ = RigidTransform(rpy=RollPitchYaw([0, 0, np.pi / 2]), p=[0, 0, 0.01])  # type: ignore


def add_manipulator_from_urdf(
    robot_diagram_builder: RobotDiagramBuilder,
    arm_urdf_path: str,
    gripper_urdf_path: str,
    arm_parent_name="world",
    X_ArmParent_ArmBase: HomogeneousMatrixType | None = None,
    gripper_parent_name: str = "tool0",
    X_GripperParent_GripperBase: HomogeneousMatrixType | None = None,
    static_gripper: bool = False,
) -> tuple[ModelInstanceIndex, ModelInstanceIndex]:
    """Add a manipulator (a robot arm with a gripper) to the robot diagram builder by specifying URDF paths.
    If the models are known in airo-models, you can use add_manipulator instead.

        Args:
            robot_diagram_builder (RobotDiagramBuilder): The robot diagram builder to which the manipulator will be added.
            arm_urdf_path (str): The file path to the URDF of the robot arm.
            gripper_urdf_path (str): The file path to the URDF of the gripper.
            arm_transform: The transform of the robot arm, if None, we use supply a robot-specific default.
            gripper_transform: The transform of the gripper, if None, we supply a default for the robot-gripper pair.
            static_gripper: If True, will fix all gripper joints to their default. Useful when you don't want the gripper DoFs in the plant.

        Returns:
            tuple[ModelInstanceIndex, ModelInstanceIndex]: The indices of the robot arm and gripper in the plant.
    """
    plant = robot_diagram_builder.plant()
    parser = robot_diagram_builder.parser()
    parser.SetAutoRenaming(True)

    arm_index = parser.AddModels(arm_urdf_path)[0]

    arm_name = arm_urdf_path.split("/")[-1].split(".")[0]
    gripper_name = gripper_urdf_path.split("/")[-1].split(".")[0]

    if static_gripper:
        gripper_urdf = airo_models.urdf.read_urdf(gripper_urdf_path)
        airo_models.urdf.make_static(gripper_urdf)
        gripper_urdf_path = airo_models.urdf.write_urdf_to_tempfile(
            gripper_urdf, gripper_urdf_path, prefix=f"{gripper_name}_static_"
        )

    gripper_index = parser.AddModels(gripper_urdf_path)[0]

    # Weld some frames together
    world_frame = plant.world_frame()
    arm_frame = plant.GetFrameByName("base_link", arm_index)
    arm_tool_frame = plant.GetFrameByName("tool0", arm_index)
    gripper_frame = plant.GetFrameByName("base_link", gripper_index)

    if X_ArmParent_ArmBase is None:
        if arm_name.startswith("ur"):
            arm_rigid_transform = X_URBASE_ROSBASE
        else:
            arm_rigid_transform = RigidTransform()
    else:
        arm_rigid_transform = RigidTransform(X_ArmParent_ArmBase)

    if X_GripperParent_GripperBase is None:
        if arm_name.startswith("ur") and gripper_name.startswith("robotiq"):
            gripper_rigid_transform = X_URTOOL0_ROBOTIQ
        else:
            gripper_rigid_transform = RigidTransform()
    else:
        gripper_rigid_transform = RigidTransform(X_GripperParent_GripperBase)

    plant.WeldFrames(world_frame, arm_frame, arm_rigid_transform)
    plant.WeldFrames(arm_tool_frame, gripper_frame, gripper_rigid_transform)

    return arm_index, gripper_index


def add_manipulator(
    robot_diagram_builder: RobotDiagramBuilder,
    arm_name: str,
    gripper_name: str,
    arm_parent_name="world",
    X_ArmParent_ArmBase: HomogeneousMatrixType | None = None,
    gripper_parent_name: str = "tool0",
    X_GripperParent_GripperBase: HomogeneousMatrixType | None = None,
    static_gripper: bool = False,
) -> tuple[ModelInstanceIndex, ModelInstanceIndex]:
    """Add a manipulator (a robot arm with a gripper) to the robot diagram builder.
    Looks up the URDF files for the robot and gripper and welds them together.
    Also provides slightly opionatated default transforms for the welds.
    For example, we rotate the ROS UR URDFs 180 degrees. This enables us to send
    TCP poses in the Drake world frame to the UR control box.

    Args:
        robot_diagram_builder: The robot diagram builder to which the manipulator will be added.
        arm_name: The name of the robot arm, must be known by airo-models
        gripper_name: The name of the gripper, must be known by airo-models
        arm_transform: The transform of the robot arm, if None, we use supply a robot-specific default.
        gripper_transform: The transform of the gripper, if None, we supply a default for the robot-gripper pair.
        static_gripper: If True, will fix all gripper joints to their default. Useful when you don't want the gripper DoFs in the plant.

    Returns:
        The robot and gripper index.
    """
    arm_urdf_path = airo_models.get_urdf_path(arm_name)
    gripper_urdf_path = airo_models.get_urdf_path(gripper_name)

    return add_manipulator_from_urdf(
        robot_diagram_builder,
        arm_urdf_path,
        gripper_urdf_path,
        arm_parent_name,
        X_ArmParent_ArmBase,
        gripper_parent_name,
        X_GripperParent_GripperBase,
        static_gripper,
    )
