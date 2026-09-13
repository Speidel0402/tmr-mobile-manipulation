#!/usr/bin/env python3
"""Read-only Hamburg venue validation using one long-lived ROS participant.

This program intentionally does not start drivers, invoke ROS CLI tools, use
SSH, or publish motion.  It validates the organizer-provided environment and
the live ROS graph before any Task 3 motion program is enabled.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
from pathlib import Path
import re
import shutil
import sys
import time
from typing import Any

from interface_profile import apply_interface_profile, load_interface_profile


HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "config" / "venue.json"
DEFAULT_INTERFACE_CONFIG = HERE / "config" / "interfaces-shanghai.json"


def load_config(
    path: Path, interface_path: Path = DEFAULT_INTERFACE_CONFIG
) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        venue = json.load(stream)
    return apply_interface_profile(venue, load_interface_profile(interface_path))


def fastdds_profile_report(path: Path) -> tuple[dict[str, Any], list[str]]:
    errors: list[str] = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace").lower()
    except OSError as exc:
        return {"path": str(path), "readable": False}, [
            f"cannot read Fast DDS profile {path}: {exc}"
        ]
    contains_udp = "udpv4" in text or "udpv6" in text
    contains_shared_memory = any(
        token in text for token in ("shared_mem", "sharedmemory", ">shm<")
    )
    builtin_disabled = bool(
        re.search(r"<\s*usebuiltintransports\s*>\s*false\s*<", text)
    )
    if not contains_udp:
        errors.append("Fast DDS profile has no UDP transport descriptor")
    if contains_shared_memory:
        errors.append("Fast DDS profile includes a shared-memory transport")
    if not builtin_disabled:
        errors.append("Fast DDS profile does not disable built-in transports")
    return {
        "path": str(path),
        "readable": True,
        "contains_udp_transport": contains_udp,
        "contains_shared_memory_transport": contains_shared_memory,
        "builtin_transports_disabled": builtin_disabled,
        "udp_only": not errors,
    }, errors


def environment_report(config: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    expected = config["dds"]
    actual = {
        key: os.environ.get(key)
        for key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")
    }
    profile_key = expected["profile_environment_variable"]
    profile_value = os.environ.get(profile_key)
    actual[profile_key] = profile_value
    errors: list[str] = []
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"):
        if actual[key] != expected[key]:
            errors.append(f"{key} must be {expected[key]!r}, got {actual[key]!r}")
    if actual["ROS_LOCALHOST_ONLY"] not in expected["ROS_LOCALHOST_ONLY_allowed"]:
        errors.append(
            "ROS_LOCALHOST_ONLY must be unset or 0 so companion can receive "
            f"camera topics from ebim, got {actual['ROS_LOCALHOST_ONLY']!r}"
        )
    profile_report = None
    if not profile_value:
        errors.append(f"{profile_key} is not set")
    elif not Path(profile_value).is_file():
        errors.append(f"{profile_key} does not name a readable file: {profile_value}")
    else:
        profile_report, profile_errors = fastdds_profile_report(Path(profile_value))
        errors.extend(profile_errors)
    host = config["host"]
    machine = platform.machine().lower()
    if machine not in {host["architecture"], "arm64"}:
        errors.append(f"expected arm64/aarch64 host, got {machine}")
    os_release = None
    os_release_path = Path("/etc/os-release")
    if os_release_path.is_file():
        match = re.search(
            r'^VERSION_ID=["\']?([^"\'\n]+)',
            os_release_path.read_text(encoding="utf-8", errors="replace"),
            re.MULTILINE,
        )
        os_release = match.group(1) if match else None
        if os_release and not os_release.startswith(host["ubuntu_release"]):
            errors.append(
                f"Ubuntu must be {host['ubuntu_release']}, got {os_release}"
            )
    kernel = platform.release()
    if host["kernel_contains"] not in kernel:
        errors.append(
            f"kernel must contain {host['kernel_contains']!r}, got {kernel!r}"
        )
    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro != host["ros_distro"]:
        errors.append(f"ROS_DISTRO must be {host['ros_distro']!r}, got {ros_distro!r}")
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    if python_version != host["python_major_minor"]:
        errors.append(
            f"Python must be {host['python_major_minor']}, got {python_version}"
        )
    free_gb = shutil.disk_usage(HERE).free / (1024 ** 3)
    if free_gb < float(host["minimum_free_disk_gb"]):
        errors.append(
            f"free disk {free_gb:.2f} GiB is below "
            f"{host['minimum_free_disk_gb']:.2f} GiB"
        )
    return {
        "actual": actual,
        "machine": machine,
        "os_release": os_release,
        "kernel": kernel,
        "ros_distro": ros_distro,
        "python": python_version,
        "free_disk_gb": round(free_gb, 3),
        "fastdds_profile": profile_report,
    }, errors


def qos_profile(name: str):
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )

    if name == "sensor":
        return qos_profile_sensor_data
    if name == "transient_local":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    raise ValueError(f"unsupported QoS profile {name!r}")


def probe_native_spine(node: Any, config: dict[str, Any], timeout_s: float) -> tuple[dict[str, Any], list[str]]:
    """Check the native server and read position; never send an action goal."""
    service_spec = config["service_endpoints"][0]
    action_spec = config["action_endpoints"][0]
    service_name = service_spec["service"]
    action_name = action_spec["action"]
    service_type_name = service_spec["accepted_types"][0]
    action_type_name = action_spec["accepted_types"][0]
    report: dict[str, Any] = {
        "service": {"name": service_name, "expected_type": service_type_name, "ready": False},
        "action": {"name": action_name, "expected_type": action_type_name, "ready": False},
    }
    errors: list[str] = []
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rosidl_runtime_py.utilities import get_action, get_service

        service_type = get_service(service_type_name)
        action_type = get_action(action_type_name)
    except Exception as exc:
        errors.append(f"native spine interface package unavailable: {exc!r}")
        return report, errors
    service_client = node.create_client(service_type, service_name)
    action_client = ActionClient(node, action_type, action_name)
    try:
        budget = max(0.1, min(timeout_s, 2.0))
        report["service"]["ready"] = bool(service_client.wait_for_service(timeout_sec=budget))
        report["action"]["ready"] = bool(action_client.wait_for_server(timeout_sec=budget))
        discovered_services = dict(node.get_service_names_and_types())
        report["service"]["discovered_types"] = discovered_services.get(service_name, [])
        if service_type_name not in discovered_services.get(service_name, []):
            errors.append(f"spine service missing or type mismatch: {service_name} ({service_type_name})")
        report["action"]["goal_fields"] = dict(action_type.Goal.get_fields_and_field_types())
        required_goal_fields = {"position", "velocity", "acceleration", "deceleration"}
        if not required_goal_fields.issubset(report["action"]["goal_fields"]):
            errors.append("native spine action goal lacks the Shanghai-required fields")
        if not report["service"]["ready"]:
            errors.append(f"native spine position service unavailable: {service_name}")
        if not report["action"]["ready"]:
            errors.append(f"native spine action unavailable: {action_name} ({action_type_name})")
        if report["service"]["ready"]:
            future = service_client.call_async(service_type.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=budget)
            response = future.result() if future.done() else None
            if response is None or not bool(getattr(response, "success", False)):
                errors.append(f"native spine position query failed: {service_name}")
            else:
                position = float(response.position)
                report["service"]["position_m"] = position
                if not math.isfinite(position):
                    errors.append("native spine position is not finite")
    finally:
        node.destroy_client(service_client)
        action_client.destroy()
    return report, errors


def ros_graph_report(
    config: dict[str, Any], timeout_s: float
) -> tuple[dict[str, Any], list[str]]:
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(args=None)
    node = Node("tmr_task3_hamburg_preflight")
    receipt_times: dict[str, float] = {}
    samples: dict[str, dict[str, Any]] = {}
    subscriptions = []
    errors: list[str] = []
    try:
        discovery_deadline = time.monotonic() + min(3.0, timeout_s)
        expected_graph_topics = {
            spec["topic"]
            for spec in config["streams"] + config["command_endpoints"]
            if spec["required"]
        }
        graph: dict[str, list[str]] = {}
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            graph = dict(node.get_topic_names_and_types())
            if expected_graph_topics.issubset(graph):
                break

        def callback_for(spec: dict[str, Any]):
            def receive(message: Any) -> None:
                receipt_times[spec["topic"]] = time.monotonic()
                sample = {"received": True}
                for field in ("encoding", "width", "height"):
                    if hasattr(message, field):
                        sample[field] = getattr(message, field)
                samples[spec["topic"]] = sample

            return receive

        stream_results = []
        for spec in config["streams"]:
            topic = spec["topic"]
            found_types = graph.get(topic, [])
            item: dict[str, Any] = {
                "name": spec["name"],
                "topic": topic,
                "required": bool(spec["required"]),
                "types": found_types,
                "received": False,
            }
            if not found_types:
                if spec["required"]:
                    errors.append(f"required topic missing: {topic}")
                stream_results.append(item)
                continue
            if len(found_types) != 1:
                errors.append(f"topic has ambiguous types {found_types}: {topic}")
                stream_results.append(item)
                continue
            accepted = spec.get("accepted_types", [])
            if accepted and found_types[0] not in accepted:
                errors.append(
                    f"topic type mismatch for {topic}: {found_types[0]}, expected {accepted}"
                )
                stream_results.append(item)
                continue
            try:
                message_type = get_message(found_types[0])
                subscriptions.append(
                    node.create_subscription(
                        message_type,
                        topic,
                        callback_for(spec),
                        qos_profile(spec["qos"]),
                    )
                )
            except Exception as exc:
                errors.append(f"cannot subscribe to {topic}: {exc!r}")
            stream_results.append(item)

        deadline = time.monotonic() + timeout_s
        required_topics = {
            spec["topic"] for spec in config["streams"] if spec["required"]
        }
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if required_topics.issubset(receipt_times):
                break

        now = time.monotonic()
        by_topic = {item["topic"]: item for item in stream_results}
        for topic, stamp in receipt_times.items():
            item = by_topic[topic]
            item.update(samples.get(topic, {}))
            item["receipt_age_s"] = round(now - stamp, 3)
        for spec in config["streams"]:
            topic = spec["topic"]
            if spec["required"] and topic not in receipt_times:
                errors.append(f"no live sample received from required topic: {topic}")
            sample = samples.get(topic, {})
            for field in ("encoding", "width", "height"):
                expected_key = f"expected_{field}"
                if expected_key in spec and sample.get(field) != spec[expected_key]:
                    errors.append(
                        f"{topic} {field}={sample.get(field)!r}, "
                        f"expected {spec[expected_key]!r}"
                    )

        command_results = []
        for spec in config["command_endpoints"]:
            topic = spec["topic"]
            found_types = graph.get(topic, [])
            subscriber_count = len(node.get_subscriptions_info_by_topic(topic))
            message_schemas = []
            for type_name in found_types:
                schema: dict[str, Any] = {"type": type_name}
                try:
                    message_class = get_message(type_name)
                    schema["fields"] = dict(
                        message_class.get_fields_and_field_types()
                    )
                except Exception as exc:
                    schema["load_error"] = repr(exc)
                    if spec["required"]:
                        errors.append(
                            f"command message package unavailable for {topic}: "
                            f"{type_name}: {exc!r}"
                        )
                message_schemas.append(schema)
            item = {
                "name": spec["name"],
                "topic": topic,
                "required": bool(spec["required"]),
                "types": found_types,
                "message_schemas": message_schemas,
                "controller_subscriber_count": subscriber_count,
                "message_field": spec.get("message_field"),
                "command_values": spec.get("command_values"),
            }
            command_results.append(item)
            if spec["required"] and not found_types:
                errors.append(f"required command topic missing: {topic}")
            accepted = spec.get("accepted_types", [])
            if accepted and found_types and not any(t in accepted for t in found_types):
                errors.append(
                    f"command type mismatch for {topic}: {found_types}, expected {accepted}"
                )
            expected_field = spec.get("message_field")
            if expected_field and message_schemas:
                loaded_fields = [
                    schema.get("fields", {}) for schema in message_schemas
                ]
                if not any(expected_field in fields for fields in loaded_fields):
                    errors.append(
                        f"command field mismatch for {topic}: expected {expected_field!r}"
                    )
            if spec["required"] and subscriber_count < 1:
                errors.append(f"no controller subscribes to command topic: {topic}")

        spine_report = None
        if config.get("action_endpoints"):
            spine_report, spine_errors = probe_native_spine(node, config, timeout_s)
            errors.extend(spine_errors)

        return {
            "node_name": node.get_fully_qualified_name(),
            "single_ros_participant": True,
            "streams": stream_results,
            "command_endpoints": command_results,
            "service_count": len(node.get_service_names_and_types()),
            "native_spine": spine_report,
        }, errors
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--interface-config", type=Path, default=DEFAULT_INTERFACE_CONFIG
    )
    parser.add_argument(
        "--print-interface-only",
        action="store_true",
        help="print the resolved command profile without creating a ROS node",
    )
    parser.add_argument("--timeout-s", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    config = load_config(args.config, args.interface_config)
    if args.print_interface_only:
        print(json.dumps(config["interface_profile"], indent=2, sort_keys=True))
        return 0
    environment, environment_errors = environment_report(config)
    report: dict[str, Any] = {
        "schema_version": 1,
        "venue": config["venue"],
        "strategy": config["strategy"],
        "interface_profile": config["interface_profile"],
        "motion_commanded": False,
        "environment": environment,
    }
    errors = list(environment_errors)
    if not environment_errors:
        try:
            graph, graph_errors = ros_graph_report(config, args.timeout_s)
            report["ros_graph"] = graph
            errors.extend(graph_errors)
        except Exception as exc:
            errors.append(f"ROS graph check failed: {exc!r}")
    report["status"] = "ready" if not errors else "blocked"
    report["errors"] = errors
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(rendered + "\n", encoding="utf-8")
        temporary.replace(args.output)
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
