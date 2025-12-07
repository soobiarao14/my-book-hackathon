# VoiceCommand.msg
# Represents captured voice input and parsed intent
# Reference: specs/001-humanoid-robotics-capstone/data-model.md

std_msgs/Header header
string raw_audio_path           # Path to recorded WAV file
string transcript               # Whisper ASR output
string parsed_action            # LLM extracted action (navigate/grasp/deliver/multi_step)
string target_object            # Object name ("red cup", "blue bottle")
string source_location          # Starting location ("kitchen", "table")
string dest_location            # Destination location ("workspace", "user")
float32 confidence_score        # LLM parsing confidence (0.0-1.0)
string[] subtasks               # For multi-step commands (JSON serialized)
int32 retry_count               # Number of API retries attempted
bool success                    # Parse success flag
string error_message            # Error details if parsing failed
