#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import CandidateTrack
from uav_interfaces.msg import CandidateTrackArray
from uav_interfaces.msg import GeminiReport
from uav_vision.common.candidate_geometry import bbox_area
from uav_vision.common.candidate_geometry import bbox_center


@dataclass
class TrackState:
    track_id: int
    observations: int
    last_call_index: int
    last_request_id: str
    target_label: str
    latest_confidence: float
    best_confidence: float
    center_x_norm: float
    center_y_norm: float
    bbox_area_norm: float
    distance_bucket: str
    priority_score: float
    updated_at: float


class CandidateManagerNode(Node):
    """
    Accumulate Gemini target candidates into image-space tracks.

    This node only groups repeated TargetCandidate detections by label and bbox center;
    it does not estimate RTK/world coordinates or command vehicle movement.
    """

    def __init__(self):
        super().__init__("candidate_manager")

        self.declare_parameter("gemini_report_topic", "/uav/vision/gemini_report")
        self.declare_parameter("tracks_topic", "/uav/vision/candidate_tracks")
        self.declare_parameter("min_confidence", 0.30)
        self.declare_parameter("match_center_distance", 0.15)
        self.declare_parameter("max_track_age_sec", 30.0)  # 0: keep tracks forever
        self.declare_parameter("publish_empty", True)

        self.gemini_report_topic = str(self.get_parameter("gemini_report_topic").value)
        self.tracks_topic = str(self.get_parameter("tracks_topic").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.match_center_distance = float(self.get_parameter("match_center_distance").value)
        self.max_track_age_sec = max(float(self.get_parameter("max_track_age_sec").value), 0.0)
        self.publish_empty = bool(self.get_parameter("publish_empty").value)

        self.tracks = {}
        self.next_track_id = 1
        self.last_primary_track_id = 0

        self.create_subscription(GeminiReport, self.gemini_report_topic, self.on_gemini_report, 10)
        self.tracks_pub = self.create_publisher(CandidateTrackArray, self.tracks_topic, 10)

        self.get_logger().warn(
            f"Candidate manager started: report={self.gemini_report_topic}, "
            f"tracks={self.tracks_topic}, min_conf={self.min_confidence:.2f}, "
            f"match_dist={self.match_center_distance:.2f}"
        )

    def on_gemini_report(self, report):
        now = time.monotonic()
        self.prune_stale_tracks(now)

        if not report.parsed_ok:
            if self.publish_empty:
                self.publish_tracks(report, primary_track_id=0)
            return

        primary_track_id = 0
        updated_track_ids = []

        for candidate in report.target_candidates:
            if candidate.confidence < self.min_confidence:
                continue

            center_x, center_y = bbox_center(candidate)
            track = self.find_matching_track(
                candidate.target_label,
                center_x,
                center_y,
                now,
            )
            if track is None:
                track = self.create_track()

            self.update_track(track, candidate, center_x, center_y, report, now)
            updated_track_ids.append(track.track_id)

            if candidate.candidate_index == report.primary_candidate_index:
                primary_track_id = track.track_id

        if primary_track_id == 0 and updated_track_ids:
            primary_track_id = self.best_track_id(updated_track_ids)
        if primary_track_id == 0:
            primary_track_id = self.best_track_id(self.tracks.keys())

        self.publish_tracks(report, primary_track_id=primary_track_id)

    def prune_stale_tracks(self, now):
        if self.max_track_age_sec <= 0.0:
            return

        stale_ids = [
            track_id
            for track_id, track in self.tracks.items()
            if now - track.updated_at > self.max_track_age_sec
        ]
        for track_id in stale_ids:
            del self.tracks[track_id]

    def find_matching_track(self, target_label, center_x, center_y, now):
        best_track = None
        best_distance = float("inf")

        for track in self.tracks.values():
            if self.max_track_age_sec > 0.0 and now - track.updated_at > self.max_track_age_sec:
                continue
            if track.target_label != target_label:
                continue

            distance = math.hypot(
                center_x - track.center_x_norm,
                center_y - track.center_y_norm,
            )
            if distance <= self.match_center_distance and distance < best_distance:
                best_track = track
                best_distance = distance

        return best_track

    def create_track(self):
        track = TrackState(
            track_id=self.next_track_id,
            observations=0,
            last_call_index=0,
            last_request_id="",
            target_label="",
            latest_confidence=0.0,
            best_confidence=0.0,
            center_x_norm=0.0,
            center_y_norm=0.0,
            bbox_area_norm=0.0,
            distance_bucket="unknown",
            priority_score=0.0,
            updated_at=0.0,
        )
        self.tracks[track.track_id] = track
        self.next_track_id += 1
        return track

    def update_track(self, track, candidate, center_x, center_y, report, now):
        track.observations += 1
        track.last_call_index = report.call_index
        track.last_request_id = report.request_id
        track.target_label = candidate.target_label
        track.latest_confidence = float(candidate.confidence)
        track.best_confidence = max(track.best_confidence, track.latest_confidence)
        track.center_x_norm = float(center_x)
        track.center_y_norm = float(center_y)
        track.bbox_area_norm = float(bbox_area(candidate))
        track.distance_bucket = candidate.distance_bucket or "unknown"
        track.priority_score = self.compute_priority(track)
        track.updated_at = now

    def compute_priority(self, track):
        observation_bonus = min(track.observations, 5) * 0.02
        return min(
            0.8 * track.best_confidence + 0.2 * track.latest_confidence + observation_bonus,
            1.0,
        )

    def best_track_id(self, track_ids):
        valid_tracks = [self.tracks[track_id] for track_id in track_ids if track_id in self.tracks]
        if not valid_tracks:
            return 0
        return max(valid_tracks, key=lambda track: track.priority_score).track_id

    def publish_tracks(self, report, primary_track_id):
        msg = CandidateTrackArray()
        msg.header = report.header
        msg.primary_track_id = int(primary_track_id)
        msg.tracks = [
            self.to_track_msg(track, primary=(track.track_id == primary_track_id))
            for track in sorted(
                self.tracks.values(),
                key=lambda item: item.priority_score,
                reverse=True,
            )
        ]
        msg.track_count = len(msg.tracks)
        self.last_primary_track_id = msg.primary_track_id
        self.tracks_pub.publish(msg)

    def to_track_msg(self, track, primary):
        msg = CandidateTrack()
        msg.track_id = int(track.track_id)
        msg.observations = int(track.observations)
        msg.last_call_index = int(track.last_call_index)
        msg.last_request_id = track.last_request_id
        msg.target_label = track.target_label
        msg.latest_confidence = float(track.latest_confidence)
        msg.best_confidence = float(track.best_confidence)
        msg.center_x_norm = float(track.center_x_norm)
        msg.center_y_norm = float(track.center_y_norm)
        msg.bbox_area_norm = float(track.bbox_area_norm)
        msg.distance_bucket = track.distance_bucket
        msg.priority_score = float(track.priority_score)
        msg.primary = bool(primary)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CandidateManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
