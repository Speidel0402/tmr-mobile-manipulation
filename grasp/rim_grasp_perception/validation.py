def is_stale(last_input_s: float, now_s: float, timeout_s: float) -> bool:
    return last_input_s > 0.0 and now_s - last_input_s > timeout_s
