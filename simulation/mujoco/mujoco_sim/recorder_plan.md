# Interactive Scene Video Recording for `mujoco_sim`

## Goal and design

Add Linux/ROS 2 video recording to the interactive viewer using the current camera and visualization state. Recording will:

- Capture only the 3D scene, excluding sidebars, help, profiler, and recording indicator.
- Use wall-clock timing, including pauses and camera movement while paused.
- Render at a fixed 1920×1080, 30 FPS by default.
- Stream frames directly to FFmpeg and produce H.264 MP4 files.
- Keep the GUI responsive through a bounded encoder queue.
- Require no MuJoCo upgrade; the bundled 3.3.5 API already supports the required framebuffer operations.

Data flow:

```text
ROS parameters → Simulate recording configuration
F8/UI button → recording request
Render thread → fixed offscreen render → mjr_readPixels
Encoder queue → FFmpeg stdin → timestamped MP4
```

## Implementation

### Recording configuration and ROS integration

Add these startup-only parameters to `MujocoSimNodeBase`:

```yaml
simulation:
  ros__parameters:
    recording.output_directory: "./recordings"
    recording.filename_prefix: "mujoco"
    recording.width: 1920
    recording.height: 1080
    recording.fps: 30
    recording.queue_capacity: 8
    recording.auto_start: false
```

- Declare and read them in the common base node so every robot plugin inherits recording support.
- In `set_sim_ptr()`, validate and pass a `RecordingConfig` into `Simulate`.
- Require positive, even dimensions, FPS in `1..240`, and queue capacity in `2..120`.
- Create the output directory when recording starts.
- Generate collision-free names such as `mujoco_20260912_143015.mp4`, adding a numeric suffix when necessary.
- Keep the existing launch file unchanged because it already passes the ROS parameter file to the simulation node.

### Public renderer interface and state

Extend `simulate.h` with:

```cpp
struct RecordingConfig {
  std::filesystem::path output_directory;
  std::string filename_prefix;
  int width;
  int height;
  int fps;
  int queue_capacity;
  bool auto_start;
};

void ConfigureRecording(RecordingConfig config);
void RequestStartRecording();
void RequestStopRecording();
bool IsRecording() const;
```

Use render-thread-owned states: `Idle`, `Recording`, `Finalizing`, and `Error`. Public/UI calls only set atomic requests; they must never call OpenGL or FFmpeg directly.

If `auto_start` is enabled, defer starting until a model has loaded successfully.

### FFmpeg writer

Add a `VideoRecorder` class in new `video_recorder.h/.cc` files.

- Launch FFmpeg with `posix_spawnp`; do not construct a shell command.
- Pipe raw RGB24 frames into standard input using:

```text
ffmpeg -loglevel error -y
       -f rawvideo -pixel_format rgb24
       -video_size WIDTHxHEIGHT -framerate FPS
       -i pipe:0
       -vf vflip,format=yuv420p
       -an -c:v libx264 -preset veryfast -crf 18
       -movflags +faststart OUTPUT.mp4
```

- Run pipe writes on a worker thread.
- Use a bounded queue of reusable RGB buffers.
- Attach a repeat count to queued frames. When rendering misses wall-clock intervals, repeat the latest frame so output duration remains synchronized with elapsed time.
- When the queue is full, replace the newest pending image with the latest capture while preserving its accumulated repeat count. This favors current footage and bounded memory without blocking interaction.
- On stop, drain the queue, close FFmpeg stdin, call `waitpid`, and report a nonzero exit status.
- Add `ffmpeg` as a runtime dependency in `package.xml`; no codec library linking is required.

### Render-thread capture

Modify `Simulate::Render()`:

1. Render the interactive window normally.
2. When a wall-clock frame is due:
   - Save the currently active framebuffer.
   - Resize the offscreen buffer once at recording start with `mjr_resizeOffscreen`.
   - Select `mjFB_OFFSCREEN`.
   - Render the existing `mjvScene` into a fixed recording viewport. This reuses the live interactive camera, geoms, labels, rendering flags, and perturbation visuals.
   - Read RGB pixels with `mjr_readPixels`.
   - Restore the previous framebuffer before any further window drawing or buffer swap.
   - Submit the frame to `VideoRecorder`.
3. Draw a red `REC mm:ss` overlay in the interactive window after restoring the window framebuffer. Do not render it into the offscreen recording.
4. Swap the interactive window buffers as before.

Exclude all 2D viewer UI, help, profiler, sensor charts, custom text, figures, and images from the recorded file. MuJoCo scene-level labels and visualization geoms remain included.

### Controls and lifecycle

- Add a `Record video` button to the File panel.
- Bind F8 to start/stop and add it to the help overlay.
- Starting before model load shows `Load a model before recording`.
- Starting while `Finalizing` is ignored with a visible status message.
- Model reload stops and finalizes the current recording before replacing the scene.
- Viewer shutdown synchronously drains and finalizes the recording so the MP4 remains playable.
- FFmpeg startup, pipe, directory, or encoding errors transition to `Error`, print the reason, and show a short viewer overlay.
- Multiple recordings in one session create separate timestamped files.

The approach follows MuJoCo’s documented framebuffer-access model and official [`record.cc`](https://github.com/google-deepmind/mujoco/blob/main/sample/record.cc) pipeline.

## Verification

- Add unit tests for parameter validation, unique filename generation, frame scheduling, queue replacement/repeat behavior, and clean subprocess shutdown.
- Add an FFmpeg integration test using small synthetic RGB frames; verify codec, resolution, FPS, pixel format, and duration with `ffprobe`.
- Manually record while orbiting, panning, zooming, pausing, perturbing objects, toggling panels, and resizing the window.
- Confirm the MP4 remains fixed resolution, follows the current camera, excludes UI panels, and has wall-clock duration within one frame interval.
- Confirm missing FFmpeg, invalid output directories, model reload, repeated recordings, and closing while recording fail or finalize cleanly without deadlocks.
- Build with the package’s normal ROS 2/ament workflow and verify existing simulation behavior is unchanged when recording is idle.

## Scope assumptions

- Target platform is Linux with ROS 2.
- FFmpeg with `libx264` is installed at runtime and discoverable through `PATH`.
- Recordings contain video only; audio is out of scope.
- Parameters are read at startup and are not dynamically reconfigurable.
- GPU pixel readback remains synchronous; asynchronous OpenGL PBO optimization is deferred unless performance testing shows 1080p/30 capture is unusable.
