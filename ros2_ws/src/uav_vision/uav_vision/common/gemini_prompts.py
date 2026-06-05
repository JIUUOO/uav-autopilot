"""Versioned Gemini prompts used by vision analyzer nodes."""


PERSON_BBOX_V1 = """
You are analyzing an image from a UAV front camera.

Return JSON only.

Schema:
{
  "scene_summary": "short description",
  "person_detected": false,
  "primary_candidate_index": -1,
  "person_candidates": [
    {
      "candidate_index": 0,
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

Be conservative. If no person is clearly visible, return person_detected=false and an empty person_candidates list.
Return one person_candidates entry per visible person using normalized bbox coordinates.
Choose one primary candidate for inspection, or -1 if no person is detected.
The distance bucket is only a visual near/far estimate for closed-loop feedback, not a metric distance.
Do not output flight, movement, or gimbal actuator commands.
"""


DEFAULT_PROMPT_VERSION = "person_bbox_v1"

PROMPTS = {
    DEFAULT_PROMPT_VERSION: PERSON_BBOX_V1,
}


def get_prompt(prompt_version):
    """Return the selected prompt text and resolved version name."""

    version = str(prompt_version).strip() or DEFAULT_PROMPT_VERSION
    if version not in PROMPTS:
        raise KeyError(
            f"Unknown Gemini prompt version '{version}'. "
            f"Available versions: {', '.join(sorted(PROMPTS))}"
        )
    return PROMPTS[version], version
