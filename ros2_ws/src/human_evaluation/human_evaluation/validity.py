"""Which topics a bag must contain to count as a valid run. No ROS imports."""

REQUIRED_TOPICS = (
    "/human_camera/image_raw",
    "/human_camera/depth/image_raw",
    "/human_camera/camera_info",
    "/human_detections",
    "/tracked_humans",
    "/tracked_humans_3d",
    "/tracked_humans_map",
    "/tf",
    "/odom",
    "/cmd_vel",
    "/plan",
    "/local_plan",
)

HUMAN_TOPICS = ("/human_detections", "/tracked_humans", "/tracked_humans_3d", "/tracked_humans_map")


def missing_topics(topic_counts: dict[str, int]) -> list[str]:
    """Required topics that are absent from the bag or have no messages."""
    return [topic for topic in REQUIRED_TOPICS if topic_counts.get(topic, 0) == 0]


def is_valid(topic_counts: dict[str, int]) -> bool:
    return not missing_topics(topic_counts)
