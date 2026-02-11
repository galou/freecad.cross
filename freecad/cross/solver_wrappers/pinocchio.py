import FreeCAD as fc

from ..ik_utils import get_kinematic_urdf
from ..ik_utils import robot_axis_pose_si_units
from ..ik_utils import axis_pose_freecad_from_si_units
from ..wb_utils import warn

# Stubs and type hints.
from ..robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501


def ik(
        robot: CrossRobot,
        from_link: str,
        to_link: str,
        target: fc.Placement,
        seed: list[float] | None = None,
        fixed_joints: dict[str, float] | None = None,
        max_iter=1000,
        transl_tol=1e-6,
        rot_tol=1e-6,
) -> list[list[float]]:
    """Return some solutions to the inverse kinematics of a Robot.

    Compute the joint positions to reach the given target.
    Some algorithms may return multiple solutions.

    For search-based algorithms, a seed configuration must be provided.
    The optional arguments `max_iter`, `transl_tol` and `rot_tol` are used to
    tune the search in this case.

    Args:
        - robot: The Robot to compute the inverse kinematics for.
        - from_link: The name of the link to start from.
        - to_link: The name of the link to reach.
        - target: The target placement of `to_link` relative to `from_link`.
        - seed: An optional seed configuration to start the search from.
                The number of values must match the number of joints from
                `to_link` to `from_link`, including fixed ones.
        - fixed_joints: An optional dictionary of joint names to fix with their
                value.
        - max_iter: The maximum number of iterations to perform.
        - transl_tol: The translational tolerance to consider the target reached.
        - rot_tol: The rotational tolerance to consider the target reached.
    """

    # Late import to avoid hard dependency on pinocchio and to avoid slowing
    # down FreeCAD's launch.
    import tempfile
    import os
    import numpy as np

    # Add /opt/openrobots/lib/python?.?/site-packages before importing
    # pinocchio.
    # Otherswise, pinocchio, pinocchio tries to load hppfcl (provided by ROS) instead of coal (provided by robotpkg) and fails.
    from ..solver_wrappers.robotpkg import add_robotpkg_library_path
    add_robotpkg_library_path()

    try:
        import pinocchio
    except ImportError as e:
        warn(
            "The pinocchio module is required to use the ik() function."
            " Please install it e.g. with 'pip install pinocchio'."
            f" The thrown error was: {e}",
            gui=True,
        )
        raise

    if (robot, from_link, to_link) in ik._model_cache:
        model = ik._model_cache[(robot, from_link, to_link)]
    else:
        urdf_str = get_kinematic_urdf(robot, from_link, to_link)
        # Write URDF to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix='.urdf') as tmp:
            tmp.write(urdf_str.encode('utf-8'))
            urdf_path = tmp.name

        # Load model from URDF
        model = pinocchio.buildModelFromUrdf(urdf_path)
        ik._model_cache[(robot, from_link, to_link)] = model

        # Remove the temporary file
        os.unlink(urdf_path)

    data = model.createData()

    # The first joint is the universe joint, we skip it.
    joint_names = [name for name in model.names][1:]

    # Initial configuration
    if seed is not None:
        q = np.array(seed)
    else:
        q = np.array(robot_axis_pose_si_units(robot, joint_names))

    # Get target placement as SE3
    target_se3 = pinocchio.SE3(np.array(target.Matrix.A).reshape((4, 4)))
    fc_to_pin = 0.001  # FreeCAD to Pinocchio units (mm to m)
    target_se3.translation *= fc_to_pin

    # Find joint id for to_link
    to_link_id = model.getFrameId(to_link)

    # Simple iterative IK (Gauss-Newton)
    # Cf. https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b_examples_d_inverse_kinematics.html
    # for an alternative implementation.
    for i in range(max_iter):
        pinocchio.forwardKinematics(model, data, q)
        pinocchio.updateFramePlacements(model, data)
        current_se3 = data.oMf[to_link_id]
        err = pinocchio.log6(current_se3.inverse() * target_se3).vector
        if np.linalg.norm(err[:3]) < transl_tol and np.linalg.norm(err[3:]) < rot_tol:
            return [axis_pose_freecad_from_si_units(robot, q, joint_names)]
        J = pinocchio.computeFrameJacobian(model, data, q, to_link_id)
        v = np.linalg.pinv(J) @ err
        q = q + v
    warn('IK did not converge', gui=True)
    return []

ik._model_cache: dict[tuple[CrossRobot, str, str], 'pinocchio.pinocchio_pywrap_default.Model'] = {}
