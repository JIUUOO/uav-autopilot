"""Versioned Gemini prompts used by vision analyzer nodes."""


TARGET_BBOX_V1 = """
You are analyzing an image from a UAV camera.
Find only the requested target: TARGET_QUERY.

Return JSON only.

Schema:
{
  "scene_summary": "short description",
  "target_detected": false,
  "primary_candidate_index": -1,
  "target_candidates": [
    {
      "candidate_index": 0,
      "target_label": "specific visible target label",
      "confidence": 0.0,
      "bbox_norm": {
        "x_min": 0.0,
        "y_min": 0.0,
        "x_max": 1.0,
        "y_max": 1.0
      },
      "distance_bucket": "far | near | unknown"
    }
  ]
}

Be conservative. If the requested target is not clearly visible, return target_detected=false and an empty target_candidates list.
Return one target_candidates entry per visible match using normalized bbox coordinates.
Set every target_label exactly to the requested target text: TARGET_QUERY.
Choose one primary candidate for inspection, or -1 if no matching target is detected.
The distance bucket is only a visual near/far estimate for closed-loop feedback, not a metric distance.
Do not output flight, movement, or gimbal actuator commands.
"""


DEFAULT_PROMPT_VERSION = "target_bbox_v1"

PROMPTS = {
    DEFAULT_PROMPT_VERSION: TARGET_BBOX_V1,
}


def get_prompt(prompt_version, target_query="person"):
    """Return the selected prompt text and resolved version name."""

    version = str(prompt_version).strip() or DEFAULT_PROMPT_VERSION
    if version not in PROMPTS:
        raise KeyError(
            f"Unknown Gemini prompt version '{version}'. "
            f"Available versions: {', '.join(sorted(PROMPTS))}"
        )
    query = str(target_query).strip() or "person"
    return PROMPTS[version].replace("TARGET_QUERY", query), version
