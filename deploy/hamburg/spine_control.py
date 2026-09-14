"""Native Hamburg spine adapter for an existing, long-lived Humble ROS node.

Construct this before the mission starts moving. No node or DDS participant is
created here; a caller owns the node and its executor.
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable


class SpineControl:
    def __init__(
        self,
        node: Any,
        profile: dict[str, Any],
        heartbeat: Callable[[], None] | None = None,
    ) -> None:
        if profile["interface"] != "action":
            raise ValueError("native Hamburg spine control requires the action profile")
        from rclpy.action import ActionClient
        from rosidl_runtime_py.utilities import get_action, get_service

        self.node = node
        self.profile = profile
        self.heartbeat = heartbeat
        self.heartbeat_count = 0
        self.errors: list[str] = []
        self.action_type = get_action(profile["action_type"])
        self.service_type = get_service(profile["position_service_type"])
        self.position_client = node.create_client(
            self.service_type, profile["position_service"]
        )
        self.action_client = ActionClient(
            node, self.action_type, profile["action_name"]
        )

    def close(self) -> None:
        self.node.destroy_client(self.position_client)
        self.action_client.destroy()

    def diagnostic_snapshot(self) -> dict[str, Any]:
        return {
            "interface": self.profile["interface"],
            "action_name": self.profile["action_name"],
            "position_service": self.profile["position_service"],
            "arm_keepalive_count": self.heartbeat_count,
            "errors": list(self.errors),
        }

    def _keep_arm_stream_alive(self) -> None:
        if self.heartbeat is not None:
            self.heartbeat()
            self.heartbeat_count += 1

    def _wait_for_endpoint(self, waiter: Callable[..., bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._keep_arm_stream_alive()
            if waiter(timeout_sec=min(0.02, max(0.0, deadline - time.monotonic()))):
                return True
        return False

    def _wait(self, future: Any, timeout_s: float, *, service_heartbeat: bool = True) -> Any:
        import rclpy

        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            if service_heartbeat:
                self._keep_arm_stream_alive()
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())),
            )
        if service_heartbeat:
            self._keep_arm_stream_alive()
        if not future.done() or future.result() is None:
            raise TimeoutError("native spine response timed out")
        return future.result()

    def position(self, timeout_s: float = 5.0) -> float:
        if not self._wait_for_endpoint(self.position_client.wait_for_service, timeout_s):
            raise RuntimeError("native spine position service unavailable")
        response = self._wait(
            self.position_client.call_async(self.service_type.Request()), timeout_s
        )
        if not bool(response.success):
            raise RuntimeError("native spine position query failed")
        measured = float(response.position)
        if not math.isfinite(measured):
            raise RuntimeError("native spine position is not finite")
        return measured

    def move_absolute(
        self,
        target_m: float,
        *,
        velocity_mps: float = 0.05,
        timeout_s: float = 20.0,
        tolerance_m: float = 0.003,
    ) -> dict[str, float | bool]:
        target_m = float(target_m)
        if not math.isfinite(target_m) or not (
            float(self.profile["minimum_m"]) <= target_m <= float(self.profile["maximum_m"])
        ):
            raise ValueError("spine target is outside the configured range")
        if not math.isfinite(velocity_mps) or not 0.0 < velocity_mps <= 0.05:
            raise ValueError("spine velocity must be positive and at most 0.05 m/s")
        start_m = self.position()
        if abs(start_m - target_m) <= tolerance_m:
            return {"moved": False, "start_position_m": start_m,
                    "target_position_m": target_m, "measured_position_m": start_m}
        if not self._wait_for_endpoint(self.action_client.wait_for_server, 5.0):
            raise RuntimeError("native spine action unavailable")
        goal = self.action_type.Goal()
        goal.position = target_m
        goal.velocity = float(velocity_mps)
        goal.acceleration = 0.1
        goal.deceleration = 0.1
        send_future = self.action_client.send_goal_async(goal)
        handle = None
        try:
            handle = self._wait(send_future, 5.0)
            if not handle.accepted:
                raise RuntimeError("native spine goal rejected")
            wrapped = self._wait(handle.get_result_async(), timeout_s)
        except BaseException:
            try:
                # The server may already have accepted the goal even if a
                # heartbeat or interrupt prevented receiving its handle.
                # Resolve only this request, with a bounded wait, so cleanup
                # cannot cancel another client's spine goal.
                if handle is None:
                    handle = self._wait(send_future, 5.0, service_heartbeat=False)
                # Cancellation must still reach the action server if the
                # heartbeat itself caused the original failure. The arm
                # publisher runs independently of this executor heartbeat.
                if handle.accepted:
                    response = self._wait(
                        handle.cancel_goal_async(), 5.0, service_heartbeat=False
                    )
                    if not response.goals_canceling:
                        raise RuntimeError("native spine cancellation was not accepted")
            except BaseException as cancel_exc:
                detail = f"spine cancellation failed: {type(cancel_exc).__name__}: {cancel_exc}"
                self.errors.append(detail)
                try:
                    self.node.get_logger().error(detail)
                except BaseException as log_exc:
                    self.errors.append(
                        f"spine cancellation logging failed: {type(log_exc).__name__}: {log_exc}"
                    )
            raise
        from action_msgs.msg import GoalStatus

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not bool(wrapped.result.success):
            raise RuntimeError(f"native spine action failed: {wrapped.result.error}")
        measured_m = self.position()
        if abs(measured_m - target_m) > tolerance_m:
            raise RuntimeError("native spine failed to reach the target height")
        return {"moved": True, "start_position_m": start_m,
                "target_position_m": target_m, "measured_position_m": measured_m}
