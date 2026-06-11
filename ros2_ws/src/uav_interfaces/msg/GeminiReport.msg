std_msgs/Header header

string request_id
uint32 call_index
string prompt_version

string image_source_topic
string report_path
string model
float32 latency_sec

bool parsed_ok
string error_message

string target_query
bool target_detected
int32 primary_candidate_index
uav_interfaces/TargetCandidate[] target_candidates
string recommended_gimbal_preset

string scene_summary
string raw_json
