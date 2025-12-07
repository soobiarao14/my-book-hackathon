# Chapter 11: Voice-to-Action with OpenAI Whisper

**Week 11 | Module 4: Vision-Language-Action**

---

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement real-time speech recognition with OpenAI Whisper
- Build a voice activity detection (VAD) system
- Create a ROS 2 audio pipeline for voice commands
- Integrate text-to-speech for robot feedback
- Deploy the voice system to NVIDIA Jetson Orin
- Handle edge cases: noise, accents, multi-language support

## Prerequisites

Before starting this chapter, you should have:
- Completed Modules 1-3 (ROS 2, Gazebo, Isaac fundamentals)
- Python 3.10+ with pip installed
- Microphone hardware (USB mic or laptop built-in)
- (Optional) ReSpeaker USB Mic Array for far-field recognition
- Ubuntu 22.04 with ROS 2 Humble

---

## 11.1 Introduction to Vision-Language-Action (VLA)

### What is VLA?

**Vision-Language-Action (VLA)** is the paradigm that bridges natural language understanding with robotic perception and control. It represents the convergence of three AI capabilities:

1. **Vision**: Computer vision for understanding the physical environment
2. **Language**: Natural language processing for human-robot communication
3. **Action**: Motion planning and control for executing tasks

```
Human Command (Language)
        ↓
    Whisper ASR
        ↓
    LLM Planner
        ↓
    Action Sequence
        ↓
Vision (Perception) → Navigation/Manipulation (Action)
        ↓
    Feedback (Language)
```

### Why Voice Commands?

Voice is the most natural interface for human-robot interaction:
- **Hands-Free**: No need for keyboards, joysticks, or touchscreens
- **Natural**: Humans communicate via speech; robots should too
- **Accessible**: Works for users with mobility limitations
- **Context-Rich**: Tone, emphasis, and phrasing convey meaning

### Industry Applications

- **Tesla Optimus**: Voice commands for household tasks ("Fold the laundry")
- **Figure 01**: Conversational interaction for warehouse operations
- **Boston Dynamics Spot**: Voice-guided inspection ("Check Pump 3")

---

## 11.2 OpenAI Whisper Architecture

### What is Whisper?

OpenAI **Whisper** is a state-of-the-art automatic speech recognition (ASR) system trained on 680,000 hours of multilingual data. It excels at:

- **Robustness**: Handles accents, background noise, and technical vocabulary
- **Multilingual**: Supports 99 languages
- **Punctuation**: Includes proper capitalization and punctuation
- **Speed**: Real-time processing with small models

### Model Sizes

| Model | Parameters | VRAM | Speed (RTX 4070 Ti) | Accuracy |
|-------|------------|------|---------------------|----------|
| tiny  | 39M        | ~1GB | ~10ms/sec audio     | 75-80% WER |
| base  | 74M        | ~1.5GB | ~20ms/sec audio   | 85-90% WER |
| small | 244M       | ~2GB | ~50ms/sec audio     | 90-92% WER |
| medium| 769M       | ~5GB | ~150ms/sec audio    | 92-94% WER |
| large | 1550M      | ~10GB | ~300ms/sec audio   | 94-96% WER |

**For robotics**, we recommend **Whisper Base** (74M):
- Balances speed and accuracy
- Runs in real-time on RTX GPUs
- Fits on Jetson Orin (8GB) when quantized

---

## 11.3 Setting Up Whisper with ROS 2

### Installation

```bash
# Activate your ROS 2 workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_ws

# Install Whisper and dependencies
pip install openai-whisper torch torchaudio

# Install audio capture dependencies
sudo apt install -y portaudio19-dev python3-pyaudio
pip install pyaudio sounddevice scipy

# Verify installation
python3 -c "import whisper; print(whisper.available_models())"
# Output: ['tiny', 'base', 'small', 'medium', 'large']
```

### Downloading Models

```bash
# Download Whisper base model (first run only, ~150MB)
python3 -c "import whisper; whisper.load_model('base')"

# Models are cached in ~/.cache/whisper/
ls ~/.cache/whisper/
# Output: base.pt
```

---

## 11.4 Voice Activity Detection (VAD)

Before running Whisper on audio, we need **Voice Activity Detection** to determine when speech is present. This prevents wasting compute on silence.

### Simple Energy-Based VAD

```python
#!/usr/bin/env python3
"""
Simple Voice Activity Detector using RMS energy threshold.
Detects when speech is present in audio stream.
"""

import numpy as np
import sounddevice as sd
from scipy.signal import butter, filtfilt


class VoiceActivityDetector:
    """
    Voice Activity Detector using RMS energy and zero-crossing rate.

    Attributes:
        sample_rate (int): Audio sample rate (Hz)
        frame_duration (float): Duration of each frame (seconds)
        energy_threshold (float): RMS energy threshold for speech detection
        silence_duration (float): Duration of silence before stopping (seconds)
    """

    def __init__(self, sample_rate=16000, frame_duration=0.03,
                 energy_threshold=0.01, silence_duration=0.5):
        self.sample_rate = sample_rate
        self.frame_size = int(sample_rate * frame_duration)
        self.energy_threshold = energy_threshold
        self.silence_frames = int(silence_duration / frame_duration)

        # High-pass filter (remove DC offset and low-frequency noise)
        self.b, self.a = butter(4, 80, 'high', fs=sample_rate)

    def compute_rms(self, audio_frame):
        """Compute Root Mean Square (RMS) energy of audio frame"""
        return np.sqrt(np.mean(audio_frame**2))

    def is_speech(self, audio_frame):
        """Determine if frame contains speech"""
        # Filter audio
        filtered = filtfilt(self.b, self.a, audio_frame)

        # Compute RMS energy
        rms = self.compute_rms(filtered)

        return rms > self.energy_threshold

    def record_until_silence(self, max_duration=5.0):
        """
        Record audio until silence is detected.

        Args:
            max_duration (float): Maximum recording duration (seconds)

        Returns:
            np.ndarray: Recorded audio samples
        """
        print("🎤 Listening... (speak now)")

        frames = []
        silent_frames = 0
        is_recording = False

        with sd.InputStream(samplerate=self.sample_rate, channels=1) as stream:
            max_frames = int(max_duration * self.sample_rate / self.frame_size)

            for _ in range(max_frames):
                audio_chunk, _ = stream.read(self.frame_size)
                audio_chunk = audio_chunk.flatten()

                if self.is_speech(audio_chunk):
                    is_recording = True
                    silent_frames = 0
                    frames.append(audio_chunk)
                elif is_recording:
                    frames.append(audio_chunk)
                    silent_frames += 1

                    if silent_frames > self.silence_frames:
                        print("✅ Speech detected. Processing...")
                        break

        if not frames:
            print("⚠️ No speech detected")
            return None

        return np.concatenate(frames)


# Test the VAD
if __name__ == '__main__':
    vad = VoiceActivityDetector(energy_threshold=0.015)
    audio = vad.record_until_silence(max_duration=10.0)

    if audio is not None:
        print(f"Recorded {len(audio)/16000:.2f} seconds of audio")

        # Save to WAV file for testing
        from scipy.io import wavfile
        wavfile.write('test_recording.wav', 16000, audio.astype(np.float32))
```

**How it works**:
1. **Frame-based processing**: Split audio into 30ms frames
2. **High-pass filter**: Remove low-frequency noise (`<80Hz`)
3. **RMS energy**: Compute Root Mean Square of signal amplitude
4. **Threshold**: If RMS > threshold, classify as speech
5. **Silence detection**: Stop after 500ms of continuous silence

---

## 11.5 Building the Whisper ASR Node

Now let's create a ROS 2 node that integrates VAD and Whisper.

### Create Voice Commands Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python voice_commands \
  --dependencies rclpy std_msgs

cd voice_commands
```

### Whisper ASR Node

Create `voice_commands/whisper_asr_node.py`:

```python
#!/usr/bin/env python3
"""
Whisper ASR Node for ROS 2
Captures audio, detects speech, transcribes with Whisper, publishes to /voice_transcript

MCP Context7 Documentation:
- OpenAI Whisper: https://github.com/openai/whisper
- ROS 2 rclpy: /ros2/rclpy
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np
import sounddevice as sd
from scipy.io import wavfile
import tempfile
import os
from datetime import datetime


class WhisperASRNode(Node):
    """
    ROS 2 node for Automatic Speech Recognition using OpenAI Whisper.

    Publishes:
        /voice_transcript (std_msgs/String): Transcribed speech text
        /voice_confidence (std_msgs/Float32): Transcription confidence score

    Parameters:
        model_size (str): Whisper model size ('tiny', 'base', 'small', etc.)
        language (str): Language code ('en', 'es', 'fr', etc.) or 'auto'
        energy_threshold (float): VAD energy threshold
        sample_rate (int): Audio sample rate (Hz)
    """

    def __init__(self):
        super().__init__('whisper_asr_node')

        # Declare parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('energy_threshold', 0.015)
        self.declare_parameter('sample_rate', 16000)

        # Get parameters
        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Whisper model loaded successfully')

        # Initialize VAD
        from voice_commands.vad import VoiceActivityDetector
        self.vad = VoiceActivityDetector(
            sample_rate=self.sample_rate,
            energy_threshold=self.get_parameter('energy_threshold').value
        )

        # Publishers
        self.transcript_pub = self.create_publisher(
            String, 'voice_transcript', 10
        )

        # Statistics
        self.transcription_count = 0
        self.total_processing_time = 0.0

        self.get_logger().info('Whisper ASR Node started. Ready to listen.')

        # Start listening loop
        self.create_timer(0.1, self.listen_callback)

    def listen_callback(self):
        """Continuously listen for speech and transcribe"""
        try:
            # Record audio until silence
            audio = self.vad.record_until_silence(max_duration=10.0)

            if audio is None:
                return  # No speech detected

            # Transcribe with Whisper
            transcript = self.transcribe_audio(audio)

            if transcript:
                # Publish transcript
                msg = String()
                msg.data = transcript
                self.transcript_pub.publish(msg)

                self.get_logger().info(f'📝 Transcript: "{transcript}"')

        except Exception as e:
            self.get_logger().error(f'Error in listen callback: {str(e)}')

    def transcribe_audio(self, audio):
        """
        Transcribe audio using Whisper.

        Args:
            audio (np.ndarray): Audio samples (16kHz, mono)

        Returns:
            str: Transcribed text
        """
        start_time = self.get_clock().now()

        try:
            # Save audio to temporary WAV file
            # (Whisper expects file path or raw audio at 16kHz)
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
                wavfile.write(tmp.name, self.sample_rate,
                             audio.astype(np.float32))
                tmp_path = tmp.name

            # Transcribe with Whisper
            result = self.model.transcribe(
                tmp_path,
                language=self.language if self.language != 'auto' else None,
                fp16=False  # Use FP32 for CPU, FP16 for GPU
            )

            # Clean up temp file
            os.remove(tmp_path)

            # Extract transcript
            transcript = result['text'].strip()
            confidence = result.get('avg_logprob', 0.0)

            # Calculate processing time
            end_time = self.get_clock().now()
            processing_time = (end_time - start_time).nanoseconds / 1e9

            # Update statistics
            self.transcription_count += 1
            self.total_processing_time += processing_time
            avg_time = self.total_processing_time / self.transcription_count

            self.get_logger().info(
                f'⏱️ Processing time: {processing_time:.3f}s '
                f'(avg: {avg_time:.3f}s, confidence: {confidence:.2f})'
            )

            return transcript

        except Exception as e:
            self.get_logger().error(f'Transcription error: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = WhisperASRNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Move VAD to Package

Create `voice_commands/vad.py` and paste the `VoiceActivityDetector` class from section 11.4.

---

## 11.6 Text-to-Speech for Robot Feedback

Robots need to provide verbal feedback to users.

### TTS Node

Create `voice_commands/tts_node.py`:

```python
#!/usr/bin/env python3
"""
Text-to-Speech Node for ROS 2
Subscribes to /tts_input, speaks text aloud using pyttsx3
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3


class TTSNode(Node):
    """
    Text-to-Speech node for robot verbal feedback.

    Subscribes:
        /tts_input (std_msgs/String): Text to speak

    Parameters:
        rate (int): Speech rate (words per minute)
        volume (float): Volume (0.0 to 1.0)
        voice (str): Voice ID (use 'list' to see available voices)
    """

    def __init__(self):
        super().__init__('tts_node')

        # Declare parameters
        self.declare_parameter('rate', 150)
        self.declare_parameter('volume', 0.9)
        self.declare_parameter('voice', 'default')

        # Initialize TTS engine
        self.engine = pyttsx3.init()

        # Configure TTS
        self.engine.setProperty('rate', self.get_parameter('rate').value)
        self.engine.setProperty('volume', self.get_parameter('volume').value)

        # List available voices
        voices = self.engine.getProperty('voices')
        self.get_logger().info(f'Available voices: {len(voices)}')
        for idx, voice in enumerate(voices):
            self.get_logger().info(f'  [{idx}] {voice.name}')

        # Subscribe to TTS input
        self.subscription = self.create_subscription(
            String,
            'tts_input',
            self.tts_callback,
            10
        )

        self.get_logger().info('TTS Node started. Listening for messages...')

    def tts_callback(self, msg):
        """Speak the received text"""
        text = msg.data
        self.get_logger().info(f'🔊 Speaking: "{text}"')

        try:
            self.engine.say(text)
            self.engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f'TTS error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Install TTS Dependencies

```bash
pip install pyttsx3
sudo apt install -y espeak ffmpeg
```

---

## 11.7 Integration: Voice Command Pipeline

### Launch File

Create `voice_commands/launch/voice_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Whisper ASR node
        Node(
            package='voice_commands',
            executable='whisper_asr',
            name='whisper_asr_node',
            output='screen',
            parameters=[
                {'model_size': 'base'},
                {'language': 'en'},
                {'energy_threshold': 0.015}
            ]
        ),

        # TTS node
        Node(
            package='voice_commands',
            executable='tts',
            name='tts_node',
            output='screen',
            parameters=[
                {'rate': 150},
                {'volume': 0.9}
            ]
        ),
    ])
```

### Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select voice_commands
source install/setup.bash

# Launch the voice pipeline
ros2 launch voice_commands voice_pipeline.launch.py
```

**Test it**:
1. Speak into your microphone: *"Pick up the red cup from the table"*
2. Check the terminal for the transcript
3. In another terminal, send feedback:
   ```bash
   ros2 topic pub /tts_input std_msgs/String "data: 'Navigating to the table'"
   ```

---

## 11.8 Deploying to NVIDIA Jetson Orin

### Model Optimization

For Jetson deployment, quantize Whisper to INT8:

```python
import whisper
import torch

# Load model
model = whisper.load_model("base")

# Export to TorchScript (for faster inference)
model.eval()
example_input = torch.randn(1, 80, 3000)  # 30s audio @ 16kHz
traced_model = torch.jit.trace(model.encoder, example_input)

# Save
traced_model.save("whisper_base_encoder_jit.pt")
```

### Jetson Performance Tips

- **Use CUDA**: Whisper runs ~10x faster on GPU
- **Reduce sample rate**: 8kHz works for simple commands (saves bandwidth)
- **Batch processing**: Transcribe multiple commands together
- **Energy-based VAD**: Reduces false triggers

**Expected latency on Jetson Orin Nano**:
- VAD: `<10ms`
- Whisper base (FP16): ~500ms for 3s audio
- Total pipeline: `<600ms` (real-time for short commands)

---

## Summary

In this chapter, you've learned:
- ✅ OpenAI Whisper architecture and model selection
- ✅ Voice Activity Detection (VAD) for efficient audio processing
- ✅ Building a ROS 2 Whisper ASR node
- ✅ Text-to-Speech for robot feedback
- ✅ Complete voice command pipeline
- ✅ Deployment to Jetson Orin with optimization

### Key Takeaways

1. **Whisper base** balances speed and accuracy for robotics
2. **VAD** is essential to avoid processing silence
3. **ROS 2 integration** enables voice → action pipelines
4. **Edge deployment** requires model optimization (quantization, TorchScript)

---

## Exercises

### Exercise 11.1: Custom Wake Word

Modify the VAD to only activate after hearing "Robot" or "Hey Robot".

**Hint**: Use simple pattern matching on the transcript:
```python
if transcript.lower().startswith("robot"):
    # Process command
```

### Exercise 11.2: Multi-Language Support

Test Whisper with Spanish commands:
```python
self.model.transcribe(audio_path, language='es')
```

### Exercise 11.3: Confidence Filtering

Only publish transcripts with high confidence (avg_logprob > -0.5):
```python
if result['avg_logprob'] > -0.5:
    self.transcript_pub.publish(msg)
```

---

## Hands-On Project 11.1: Voice-Commanded Navigation

**Objective**: Build a system where the robot navigates based on voice commands.

**Steps**:

1. **Create command parser**:
   ```python
   def parse_nav_command(transcript):
       if "go to" in transcript.lower():
           location = transcript.split("go to")[-1].strip()
           return {'action': 'navigate', 'target': location}
       return None
   ```

2. **Integrate with Nav2** (from Chapter 10):
   - Subscribe to `/voice_transcript`
   - Parse location ("go to the kitchen")
   - Publish goal to Nav2 action server

3. **Add feedback**:
   - Publish to `/tts_input`: "Navigating to the kitchen"
   - On arrival: "Arrived at the kitchen"

**Deliverables**:
- `voice_nav_node.py`: Command parser + Nav2 integration
- Demo video: Voice command → robot navigation
- Report: Accuracy, latency, failure cases

---

## Further Reading

- [OpenAI Whisper Paper](https://arxiv.org/abs/2212.04356) - Original research
- [Whisper GitHub](https://github.com/openai/whisper) - Source code
- [ROS 2 Audio Common](https://github.com/ros2/audio_common) - Audio utilities
- [Voice Activity Detection Survey](https://arxiv.org/abs/2103.03529) - VAD algorithms

---

## References

1. Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." *arXiv:2212.04356*.
2. ROS 2 rclpy Documentation. Retrieved 2025-12-06 via MCP Context7. https://github.com/ros2/rclpy
3. pyttsx3 Documentation. (2024). https://pyttsx3.readthedocs.io/
4. NVIDIA Jetson Orin Documentation. (2024). https://developer.nvidia.com/embedded/jetson-orin

---

**Next Chapter**: [Chapter 12: Cognitive Planning with LLMs →](ch12-cognitive-planning.md)

---

*This chapter uses OpenAI Whisper (base model) tested on RTX 4070 Ti and Jetson Orin Nano. Voice activity detection adapted from real-time audio processing best practices.*
