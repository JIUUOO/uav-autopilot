"""Geometry helpers for normalized PersonCandidate bounding boxes."""


def bbox_center(candidate):
    """Return the normalized bbox center as (x, y)."""

    return (
        (candidate.bbox_x_min_norm + candidate.bbox_x_max_norm) / 2.0,
        (candidate.bbox_y_min_norm + candidate.bbox_y_max_norm) / 2.0,
    )


def bbox_width(candidate):
    """Return the normalized bbox width."""

    return max(candidate.bbox_x_max_norm - candidate.bbox_x_min_norm, 0.0)


def bbox_height(candidate):
    """Return the normalized bbox height."""

    return max(candidate.bbox_y_max_norm - candidate.bbox_y_min_norm, 0.0)


def bbox_area(candidate):
    """Return the normalized bbox area."""

    return bbox_width(candidate) * bbox_height(candidate)


def horizontal_error(candidate, target_x=0.5):
    """Return normalized horizontal center error; negative means left of target."""

    center_x, _center_y = bbox_center(candidate)
    return center_x - target_x


def vertical_error(candidate, target_y=0.5):
    """Return normalized vertical center error; negative means above target."""

    _center_x, center_y = bbox_center(candidate)
    return center_y - target_y


def horizontal_bucket(candidate, center_min=0.4, center_max=0.6):
    """Classify candidate bbox center as left, center, or right."""

    center_x, _center_y = bbox_center(candidate)
    if center_x < center_min:
        return "left"
    if center_x > center_max:
        return "right"
    return "center"


def vertical_bucket(candidate, center_min=0.4, center_max=0.6):
    """Classify candidate bbox center as upper, center, or lower."""

    _center_x, center_y = bbox_center(candidate)
    if center_y < center_min:
        return "upper"
    if center_y > center_max:
        return "lower"
    return "center"


def is_centered(candidate, x_min=0.4, x_max=0.6, y_min=0.35, y_max=0.7):
    """Return true when the bbox center is inside the target image region."""

    center_x, center_y = bbox_center(candidate)
    return x_min <= center_x <= x_max and y_min <= center_y <= y_max
