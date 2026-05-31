#!/usr/bin/env python3

import time

import rclpy


def run_hold_loop(
    *,
    stop_event,
    drain_fn,
    readiness,
    logger,
    hold_sec: float,
    ntrip_connected_fn,
    rtcm_frames_fn,
    status_prefix: str = "LOITER hold",
    status_interval_sec: float = 2.0,
    sleep_sec: float = 0.05,
) -> bool:
    """Keep draining MAVLink during LOITER hold and report readiness status periodically."""

    start = time.time()
    last_print = 0.0

    while rclpy.ok() and not stop_event.is_set():
        drain_fn()

        now = time.time()
        if now - last_print > status_interval_sec:
            readiness.format_status(
                prefix=status_prefix,
                ntrip_connected=ntrip_connected_fn(),
                rtcm_frames=rtcm_frames_fn(),
                now=now,
            )
            last_print = now

        if hold_sec > 0.0 and now - start >= hold_sec:
            logger.warn("hold_sec completed.")
            return True

        time.sleep(sleep_sec)

    return False
