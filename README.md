# nvgt_steamaudio

Geometry based sound simulation for [NVGT](https://nvgt.dev) with the [Steam Audio](https://valvesoftware.github.io/steam-audio/) SDK: occlusion and transmission through walls (per frequency band), reverb times of the surrounding geometry, and a convolution (hybrid) reverb node for a sound's effects chain.

NVGT already uses Steam Audio for HRTF panning and distance attenuation; this plugin adds the scene simulation part.

## Building (Windows)

Requirements: Visual Studio 2022, CMake 3.20+, the Steam Audio SDK 4.x (`include/phonon.h`, `lib/phonon.lib`, `bin/phonon.dll`).

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DSTEAMAUDIO_DIR=<path to the SDK>
cmake --build build --config Release
```

Copy `build/Release/nvgt_steamaudio.dll` into NVGT's `lib` folder (`phonon.dll` is already there). The plugin uses a static C runtime like NVGT, so it needs no Visual C++ Redistributable.

**Note:** the reverb node requires an NVGT build that contains the plugin node fix from [samtupy/nvgt#456](https://github.com/samtupy/nvgt/pull/456); without it, NVGT never calls the `process()` of plugin audio nodes. Occlusion, transmission and reverb times work with any build.

## Usage

```
#pragma plugin nvgt_steamaudio

steam_audio_scene scene;
// absorption low/mid/high, scattering, transmission low/mid/high
int wood = scene.add_material(0.11, 0.07, 0.06, 0.05, 0.07, 0.014, 0.005);
scene.add_box(4.9, 0, 0, 5.1, 6, 4, wood); // a wall
scene.commit();

steam_audio_simulator sim(scene);
steam_audio_source@ src = sim.add_source();
sim.set_listener(2, 2, 1.5);
src.set_position(8, 2, 1.5);
sim.run_direct(); // call regularly, e.g. every 100-200 ms
// occlusion: 1 = clear line of sight, 0 = blocked; transmission: how much passes through the blocking material
float muffle = (1 - src.occlusion) * (1 - src.get_transmission(STEAM_AUDIO_BAND_HIGH));
```

See `examples/selftest.nvgt` for every feature.

### API

- `steam_audio_scene()`: `add_material(...)` returns a material index; `add_box(min_x, min_y, min_z, max_x, max_y, max_z, material)`; `add_triangle(9 coordinates, material)`; `clear()` (keeps materials), `clear_materials()`; `commit()` builds the geometry. Simulators using the scene switch to a new commit automatically before their next simulation. Properties: `triangle_count`, `material_count`, `committed`.
- `steam_audio_simulator(scene, reflections = STEAM_AUDIO_REFLECTIONS_NONE, max_sources = 32)`: `reflections` is `_NONE` (direct only), `_PARAMETRIC` (reverb times) or `_HYBRID` (convolution reverb for `steam_audio_reverb_node`). Methods: `set_listener(x, y, z)`, `add_source()`, `run_direct()`, `run_reflections()` (blocking, CPU heavy), `start_reflection_thread(interval_ms = 1000)` / `stop_reflection_thread()`, `set_reflection_quality(rays, bounces, duration)`. Properties: `valid`, `reflection_thread_running`, `reflection_runs`.
- `steam_audio_source`: `set_position(x, y, z)`, `occlusion`, `get_transmission(band)`, `get_reverb_time(band)` (RT60 in seconds), `has_reflections`, `attached`, `remove()`.
- `steam_audio_reverb_node_create(engine = sound_default_engine, channels = 2)`: an audio node for `sound.effects_chain`. `set_source(source)` (a source of a `_HYBRID` simulator, or `null` for dry passthrough), `wet` (0-1, default 0.35). Add the node once and switch its source instead of adding / removing nodes on a playing sound.
- `steam_audio_get_available()`, `steam_audio_get_last_error()` (the last Steam Audio error code). Bands: `STEAM_AUDIO_BAND_LOW`, `_MID`, `_HIGH`.

Coordinates are plain numbers (x, y, z, z is up) rather than NVGT's `vector`, so the API does not depend on the physics library behind `vector`.

### Notes

- Reflection simulation is stochastic (ray tracing): reverb times vary from run to run. Smooth them in the script before using them, e.g. an exponential moving average.
- The reverb node reads the simulation results on the audio thread without waiting for a running simulation (the same model as Steam Audio's FMOD integration). Until the first reflection simulation of its source finished, it passes the sound through unchanged.
- The reverb node expects stereo input at 44100 Hz with blocks of up to 4096 frames (NVGT's default engine).
- NVGT does not notify plugins when a node is destroyed: call `set_source(null)` before dropping a node, otherwise it keeps its source alive.

## License

zlib (see `LICENSE`), the same as NVGT.
