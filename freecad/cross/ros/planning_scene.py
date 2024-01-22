from __future__ import annotations

from typing import ForwardRef, Optional

try:
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import GetPlanningScene
    imports_ok = True
except ImportError:
    PlanningScene = ForwardRef('PlanningScene')
    GetPlanningScene = ForwardRef('GetPlanningScene')
    imports_ok = False

from .. import wb_globals
from ..freecad_utils import message
from ..freecad_utils import tr
from ..freecad_utils import warn


def get_planning_scene(timeout_sec=0.0) -> Optional[PlanningScene]:
    """Get the current planning scene by calling the ROS server.

    Parameters:
        - timeout_sec: Timeout to reach the server and then timeout
                       to get the response.

    """
    if not imports_ok:
        warn(tr('ROS modules cannot be imported'), gui=True)
        return None

    node = wb_globals.g_ros_node
    if node is None:
        return None

    executor = wb_globals.g_ros_executor

    # TODO: configure the service name.
    servce_name = 'get_planning_scene'
    client = node.create_client(GetPlanningScene, servce_name)
    while not client.wait_for_service(timeout_sec=timeout_sec):
        msg = f'service {servce_name} not available, waiting again...'
        message(msg)
        node.get_logger().info(msg)
    request = GetPlanningScene.Request()
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
    if not future.done():
        warn(tr('The server was reached but the request timed out'), gui=True)
        return None
    return future.result().scene
