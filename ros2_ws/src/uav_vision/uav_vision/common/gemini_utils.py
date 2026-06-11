"""Gemini response parsing and candidate post-processing helpers."""

import json

from uav_interfaces.msg import TargetCandidate


def make_request_id(msg, call_index):
    """Build a stable request id from the image timestamp and Gemini call index."""

    return f"{msg.header.stamp.sec}-{msg.header.stamp.nanosec}-{call_index:04d}"


def append_error(current, new_error):
    """Append a new error string while preserving any earlier error text."""

    return f"{current}; {new_error}" if current else new_error


def parse_json_response(raw_text):
    """Parse Gemini JSON text, including responses wrapped in markdown fences."""

    text = raw_text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].startswith("```"):
            lines = lines[:-1]
        text = "\n".join(lines).strip()

    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


def as_target_candidates(value):
    """Convert Gemini JSON target candidates into validated TargetCandidate messages."""

    if not isinstance(value, list):
        return []

    candidates = []
    for fallback_index, item in enumerate(value):
        if not isinstance(item, dict):
            continue

        bbox = item.get("bbox_norm", {})
        if not isinstance(bbox, dict):
            bbox = {}

        candidate = TargetCandidate()
        candidate.candidate_index = fallback_index
        candidate.target_label = str(item.get("target_label", "")).strip()
        candidate.confidence = clamp_unit(item.get("confidence", 0.0))
        candidate.bbox_x_min_norm = clamp_unit(bbox.get("x_min", 0.0))
        candidate.bbox_y_min_norm = clamp_unit(bbox.get("y_min", 0.0))
        candidate.bbox_x_max_norm = clamp_unit(bbox.get("x_max", 0.0))
        candidate.bbox_y_max_norm = clamp_unit(bbox.get("y_max", 0.0))
        candidate.distance_bucket = choice_or_unknown(
            item.get("distance_bucket", "unknown"),
            {"far", "near", "unknown"},
        )

        if (
            candidate.bbox_x_min_norm >= candidate.bbox_x_max_norm
            or candidate.bbox_y_min_norm >= candidate.bbox_y_max_norm
        ):
            continue

        candidates.append(candidate)

    return candidates


def select_primary_candidate_index(requested_index, candidates):
    """Use Gemini's primary index when valid, otherwise choose the highest-confidence candidate."""

    requested_index = as_int(requested_index, default=-1)
    if find_candidate(candidates, requested_index) is not None:
        return requested_index
    if not candidates:
        return -1
    return max(candidates, key=lambda candidate: candidate.confidence).candidate_index


def find_candidate(candidates, candidate_index):
    """Return the candidate with the requested index, or None if it is missing."""

    return next(
        (
            candidate
            for candidate in candidates
            if candidate.candidate_index == candidate_index
        ),
        None,
    )


def gimbal_preset_for_candidate(candidate):
    """Map a candidate's visual distance bucket to a rule-based gimbal preset."""

    if candidate is None:
        return "HOLD"
    if candidate.distance_bucket == "far":
        return "PRESET_FAR"
    if candidate.distance_bucket == "near":
        return "PRESET_NEAR"
    return "HOLD"


def as_int(value, default):
    """Convert a value to int, or return default if conversion fails."""

    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def as_bool(value):
    """Convert common JSON/string boolean values into bool."""

    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes"}
    return bool(value)


def clamp_unit(value):
    """Clamp a numeric value into the [0.0, 1.0] range."""

    try:
        return min(max(float(value), 0.0), 1.0)
    except (TypeError, ValueError):
        return 0.0


def choice_or_unknown(value, choices):
    """Normalize an enum-like string and fall back to unknown when invalid."""

    normalized = str(value).strip().lower()
    return normalized if normalized in choices else "unknown"
