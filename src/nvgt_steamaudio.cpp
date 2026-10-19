// nvgt_steamaudio - geometry based sound simulation for NVGT using the Steam Audio SDK.
//
// NVGT itself uses Steam Audio only for HRTF panning and distance attenuation. This plugin exposes the
// geometry based part of the SDK:
//   - steam_audio_scene: static geometry (boxes and triangles with acoustic materials)
//   - steam_audio_simulator: runs direct (occlusion, transmission) and reflection simulations on a scene
//   - steam_audio_source: a simulated sound source, results are read back per frequency band
//   - steam_audio_reverb_node: an audio node that applies the simulated (convolution / hybrid) reverb of a
//     source to a sound's effects chain
//
// Threading model (from phonon.h): iplSimulatorSetScene / iplSimulatorCommit must not run while any simulation
// is running, iplSourceSetInputs / iplSourceGetOutputs may be called for different simulation types from
// different threads without synchronization, and reflection simulation is meant to run on its own thread. The
// reverb node therefore reads the reflection outputs of its source directly on the audio thread (the same model
// as the official FMOD integration) and never waits for a running simulation.
//
// No std::string or other allocated object crosses the DLL boundary: the plugin and NVGT link their own static
// C runtime, so memory allocated on one side must not be freed on the other.

#include "../third_party/nvgt/nvgt_plugin.h"
#include <phonon.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

static const int sa_sample_rate = 44100;
static const int sa_frame_size = 256;
static const unsigned int sa_frame_capacity = 4096;
static const int sa_ambisonics_order = 1;
static const int sa_ambisonics_channels = (sa_ambisonics_order + 1) * (sa_ambisonics_order + 1);
static const float sa_max_reflection_duration = 2.0f;

enum steam_audio_reflection_type { SA_REFLECTIONS_NONE = 0, SA_REFLECTIONS_PARAMETRIC = 1, SA_REFLECTIONS_HYBRID = 2 };

static std::atomic<int> g_last_error{0};
static std::mutex g_context_mutex;
static IPLContext g_context = nullptr;
static IPLHRTF g_hrtf = nullptr;

static void set_error(IPLerror err) { g_last_error = int(err); }

static void script_exception(const char* message) {
	asIScriptContext* ctx = asGetActiveContext();
	if (ctx) ctx->SetException(message);
}

// One context for the whole process, created on first use and never released (objects can still be released by
// the script engine during shutdown). The settings are left at their defaults like NVGT's own phonon context:
// creating a second context with a different SIMD level / validation flag corrupted the first one in testing.
static IPLContext get_context() {
	std::lock_guard<std::mutex> lock(g_context_mutex);
	if (g_context) return g_context;
	IPLContextSettings settings{};
	settings.version = STEAMAUDIO_VERSION;
	IPLerror err = iplContextCreate(&settings, &g_context);
	if (err != IPL_STATUS_SUCCESS) {
		set_error(err);
		g_context = nullptr;
	}
	return g_context;
}

static IPLHRTF get_hrtf() {
	IPLContext ctx = get_context();
	if (!ctx) return nullptr;
	std::lock_guard<std::mutex> lock(g_context_mutex);
	if (g_hrtf) return g_hrtf;
	IPLAudioSettings audio{sa_sample_rate, sa_frame_size};
	IPLHRTFSettings settings{};
	settings.type = IPL_HRTFTYPE_DEFAULT;
	settings.volume = 1.0f;
	settings.normType = IPL_HRTFNORMTYPE_NONE;
	IPLerror err = iplHRTFCreate(ctx, &audio, &settings, &g_hrtf);
	if (err != IPL_STATUS_SUCCESS) {
		set_error(err);
		g_hrtf = nullptr;
	}
	return g_hrtf;
}

static IPLCoordinateSpace3 make_space(const IPLVector3& origin) {
	IPLCoordinateSpace3 cs{};
	cs.right = {1.0f, 0.0f, 0.0f};
	cs.up = {0.0f, 0.0f, 1.0f};
	cs.ahead = {0.0f, 1.0f, 0.0f};
	cs.origin = origin;
	return cs;
}

static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

class ref_counted {
	std::atomic<int> refs{1};
public:
	virtual ~ref_counted() {}
	void add_ref() { refs++; }
	void release() { if (--refs == 0) delete this; }
};

// ---------------------------------------------------------------------------------------------------------------
// steam_audio_scene

class steam_audio_scene : public ref_counted {
	std::vector<IPLMaterial> materials;
	std::vector<IPLVector3> vertices;
	std::vector<IPLTriangle> triangles;
	std::vector<IPLint32> material_indices;
	std::mutex mutex;
	IPLScene committed = nullptr; // retained, the simulators retain their own reference
	std::atomic<unsigned int> generation{0};
public:
	~steam_audio_scene() {
		if (committed) iplSceneRelease(&committed);
	}
	int add_material(float absorption_low, float absorption_mid, float absorption_high, float scattering, float transmission_low, float transmission_mid, float transmission_high) {
		IPLMaterial m{};
		m.absorption[0] = clamp01(absorption_low);
		m.absorption[1] = clamp01(absorption_mid);
		m.absorption[2] = clamp01(absorption_high);
		m.scattering = clamp01(scattering);
		m.transmission[0] = clamp01(transmission_low);
		m.transmission[1] = clamp01(transmission_mid);
		m.transmission[2] = clamp01(transmission_high);
		materials.push_back(m);
		return int(materials.size()) - 1;
	}
	bool add_triangle(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3, int material) {
		if (material < 0 || size_t(material) >= materials.size()) return false;
		IPLint32 base = IPLint32(vertices.size());
		vertices.push_back({x1, y1, z1});
		vertices.push_back({x2, y2, z2});
		vertices.push_back({x3, y3, z3});
		IPLTriangle t{};
		t.indices[0] = base; t.indices[1] = base + 1; t.indices[2] = base + 2;
		triangles.push_back(t);
		material_indices.push_back(material);
		return true;
	}
	// An axis aligned box as 8 vertices and 12 triangles, counter clockwise seen from outside.
	bool add_box(float minx, float miny, float minz, float maxx, float maxy, float maxz, int material) {
		if (material < 0 || size_t(material) >= materials.size()) return false;
		if (minx > maxx) std::swap(minx, maxx);
		if (miny > maxy) std::swap(miny, maxy);
		if (minz > maxz) std::swap(minz, maxz);
		IPLint32 b = IPLint32(vertices.size());
		vertices.push_back({minx, miny, minz});
		vertices.push_back({maxx, miny, minz});
		vertices.push_back({maxx, maxy, minz});
		vertices.push_back({minx, maxy, minz});
		vertices.push_back({minx, miny, maxz});
		vertices.push_back({maxx, miny, maxz});
		vertices.push_back({maxx, maxy, maxz});
		vertices.push_back({minx, maxy, maxz});
		static const IPLint32 quads[6][4] = {{0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}, {0, 3, 2, 1}, {4, 5, 6, 7}};
		for (int f = 0; f < 6; f++) {
			IPLTriangle t1{}, t2{};
			t1.indices[0] = b + quads[f][0]; t1.indices[1] = b + quads[f][1]; t1.indices[2] = b + quads[f][2];
			t2.indices[0] = b + quads[f][0]; t2.indices[1] = b + quads[f][2]; t2.indices[2] = b + quads[f][3];
			triangles.push_back(t1);
			triangles.push_back(t2);
			material_indices.push_back(material);
			material_indices.push_back(material);
		}
		return true;
	}
	// Clears the geometry being built. Materials are kept, the committed scene stays in use until the next commit().
	void clear() {
		vertices.clear();
		triangles.clear();
		material_indices.clear();
	}
	void clear_materials() {
		clear();
		materials.clear();
	}
	// Builds a new Steam Audio scene from the current geometry. Simulators using this object switch to it before
	// their next simulation.
	bool commit() {
		IPLContext ctx = get_context();
		if (!ctx) return false;
		IPLSceneSettings settings{};
		settings.type = IPL_SCENETYPE_DEFAULT;
		IPLScene scene = nullptr;
		IPLerror err = iplSceneCreate(ctx, &settings, &scene);
		if (err != IPL_STATUS_SUCCESS) {
			set_error(err);
			return false;
		}
		if (!triangles.empty()) {
			IPLStaticMeshSettings mesh_settings{};
			mesh_settings.numVertices = IPLint32(vertices.size());
			mesh_settings.numTriangles = IPLint32(triangles.size());
			mesh_settings.numMaterials = IPLint32(materials.size());
			mesh_settings.vertices = vertices.data();
			mesh_settings.triangles = triangles.data();
			mesh_settings.materialIndices = material_indices.data();
			mesh_settings.materials = materials.data();
			IPLStaticMesh mesh = nullptr;
			err = iplStaticMeshCreate(scene, &mesh_settings, &mesh);
			if (err != IPL_STATUS_SUCCESS) {
				set_error(err);
				iplSceneRelease(&scene);
				return false;
			}
			iplStaticMeshAdd(mesh, scene);
			iplStaticMeshRelease(&mesh); // the scene keeps its own reference
		}
		iplSceneCommit(scene);
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (committed) iplSceneRelease(&committed);
			committed = scene;
		}
		generation++;
		return true;
	}
	// Returns a retained reference to the committed scene (or null) and its generation.
	IPLScene acquire(unsigned int& out_generation) {
		std::lock_guard<std::mutex> lock(mutex);
		out_generation = generation;
		return committed ? iplSceneRetain(committed) : nullptr;
	}
	unsigned int get_generation() const { return generation; }
	unsigned int get_triangle_count() const { return (unsigned int)triangles.size(); }
	unsigned int get_material_count() const { return (unsigned int)materials.size(); }
	bool get_committed() { std::lock_guard<std::mutex> lock(mutex); return committed != nullptr; }
};

static steam_audio_scene* scene_factory() { return new steam_audio_scene(); }

// ---------------------------------------------------------------------------------------------------------------
// steam_audio_simulator / steam_audio_source

class steam_audio_source;

class steam_audio_simulator : public ref_counted {
	friend class steam_audio_source;
	steam_audio_scene* scene;
	IPLSimulator simulator = nullptr;
	IPLScene bound_scene = nullptr;
	std::atomic<bool> bound{false};
	std::atomic<unsigned int> bound_generation{0};
	std::atomic<unsigned int> reflection_runs{0};
	int reflection_type;
	// run_mutex: held while a reflection simulation runs. direct_mutex: held while a direct simulation runs. phonon
	// allows the two to run on different threads at the same time, but reconfiguring the simulator (scene binding,
	// adding / removing sources) must not overlap either, so that takes both, always in the order run -> direct.
	std::mutex run_mutex;
	std::mutex direct_mutex;
	std::mutex sources_mutex;
	std::vector<steam_audio_source*> sources;
	IPLVector3 listener{};
	std::mutex listener_mutex;
	int num_rays = 4096, num_bounces = 16;
	float duration = 1.5f;
	std::thread thread;
	std::atomic<bool> thread_stop{false};
	std::mutex thread_wait_mutex;
	std::condition_variable thread_wait;
	unsigned int thread_interval_ms = 1000;
	bool rebind_locked(); // caller holds run_mutex and direct_mutex
	bool needs_rebind() const { return !bound || bound_generation != scene->get_generation(); }
	bool rebind() { std::lock_guard<std::mutex> l2(direct_mutex); return rebind_locked(); } // caller holds run_mutex
	void reflections_locked();
	void thread_func();
public:
	bool valid() const { return simulator != nullptr; }
	steam_audio_simulator(steam_audio_scene* s, int reflections, int max_sources) : scene(s), reflection_type(reflections) {
		scene->add_ref();
		IPLContext ctx = get_context();
		if (!ctx) return;
		IPLSimulationSettings settings{};
		settings.flags = IPL_SIMULATIONFLAGS_DIRECT;
		if (reflection_type != SA_REFLECTIONS_NONE) settings.flags = IPLSimulationFlags(settings.flags | IPL_SIMULATIONFLAGS_REFLECTIONS);
		settings.sceneType = IPL_SCENETYPE_DEFAULT;
		settings.reflectionType = reflection_type == SA_REFLECTIONS_HYBRID ? IPL_REFLECTIONEFFECTTYPE_HYBRID : IPL_REFLECTIONEFFECTTYPE_PARAMETRIC;
		settings.maxNumOcclusionSamples = 32;
		settings.maxNumRays = 16384;
		settings.numDiffuseSamples = 32;
		settings.maxDuration = sa_max_reflection_duration;
		settings.maxOrder = sa_ambisonics_order;
		settings.maxNumSources = max_sources < 1 ? 1 : max_sources;
		settings.numThreads = 1;
		settings.samplingRate = sa_sample_rate;
		settings.frameSize = sa_frame_size;
		IPLerror err = iplSimulatorCreate(ctx, &settings, &simulator);
		if (err != IPL_STATUS_SUCCESS) {
			set_error(err);
			simulator = nullptr;
		}
	}
	~steam_audio_simulator() {
		stop_reflection_thread();
		if (bound_scene) iplSceneRelease(&bound_scene);
		if (simulator) iplSimulatorRelease(&simulator);
		scene->release();
	}
	int get_reflection_type() const { return reflection_type; }
	void set_listener(float x, float y, float z) {
		std::lock_guard<std::mutex> lock(listener_mutex);
		listener = {x, y, z};
	}
	IPLVector3 get_listener() {
		std::lock_guard<std::mutex> lock(listener_mutex);
		return listener;
	}
	void set_reflection_quality(int rays, int bounces, float seconds) {
		std::lock_guard<std::mutex> lock(run_mutex);
		num_rays = std::clamp(rays, 1, 16384);
		num_bounces = std::clamp(bounces, 1, 64);
		duration = std::clamp(seconds, 0.1f, sa_max_reflection_duration);
	}
	steam_audio_source* add_source();
	bool run_direct();
	bool run_reflections() {
		if (!simulator || reflection_type == SA_REFLECTIONS_NONE) return false;
		std::lock_guard<std::mutex> lock(run_mutex);
		if (needs_rebind() && !rebind()) return false;
		reflections_locked();
		return true;
	}
	bool start_reflection_thread(unsigned int interval_ms) {
		if (!simulator || reflection_type == SA_REFLECTIONS_NONE) return false;
		thread_interval_ms = interval_ms < 50 ? 50 : interval_ms;
		if (thread.joinable()) return true;
		thread_stop = false;
		thread = std::thread(&steam_audio_simulator::thread_func, this);
		return true;
	}
	void stop_reflection_thread() {
		if (!thread.joinable()) return;
		{
			std::lock_guard<std::mutex> lock(thread_wait_mutex);
			thread_stop = true;
		}
		thread_wait.notify_all();
		thread.join();
	}
	bool get_reflection_thread_running() const { return thread.joinable(); }
	unsigned int get_reflection_runs() const { return reflection_runs; }
};

class steam_audio_source : public ref_counted {
	friend class steam_audio_simulator;
	steam_audio_simulator* owner; // referenced while the source lives
	IPLSource source = nullptr;
	std::atomic<bool> attached{false};
	std::mutex position_mutex;
	IPLVector3 position{};
	IPLDirectEffectParams direct{};
	bool has_direct = false;
	std::atomic<bool> has_reflections{false};
public:
	steam_audio_source(steam_audio_simulator* sim, IPLSource src) : owner(sim), source(src), attached(true) { owner->add_ref(); }
	~steam_audio_source() {
		remove();
		if (source) iplSourceRelease(&source);
		owner->release();
	}
	void remove() {
		if (!attached) return;
		attached = false;
		{
			std::lock_guard<std::mutex> lock(owner->sources_mutex);
			auto& v = owner->sources;
			v.erase(std::remove(v.begin(), v.end(), this), v.end());
		}
		std::lock_guard<std::mutex> lock(owner->run_mutex);
		std::lock_guard<std::mutex> lock2(owner->direct_mutex);
		iplSourceRemove(source, owner->simulator);
		iplSimulatorCommit(owner->simulator);
		has_reflections = false;
	}
	bool get_attached() const { return attached; }
	steam_audio_simulator* get_owner() const { return owner; }
	IPLSource get_ipl_source() const { return source; }
	void set_position(float x, float y, float z) {
		std::lock_guard<std::mutex> lock(position_mutex);
		position = {x, y, z};
	}
	IPLVector3 get_position() {
		std::lock_guard<std::mutex> lock(position_mutex);
		return position;
	}
	// Steam Audio convention: 1 = nothing in the way, 0 = fully blocked. Use get_transmission() for how much of the
	// sound passes through the blocking geometry.
	float get_occlusion() const { return has_direct ? direct.occlusion : 1.0f; }
	float get_transmission(int band) const {
		if (band < 0 || band >= IPL_NUM_BANDS) return 1.0f;
		return has_direct ? direct.transmission[band] : 1.0f;
	}
	bool get_has_reflections() const { return has_reflections; }
	// Reverb time (RT60, seconds) of the last reflection simulation for a band, 0 if none ran yet.
	float get_reverb_time(int band) {
		if (band < 0 || band >= IPL_NUM_BANDS || !has_reflections || !attached) return 0.0f;
		IPLSimulationOutputs out{};
		iplSourceGetOutputs(source, IPL_SIMULATIONFLAGS_REFLECTIONS, &out);
		return out.reflections.reverbTimes[band];
	}
	void set_direct(const IPLDirectEffectParams& p) { direct = p; has_direct = true; }
	void set_has_reflections() { has_reflections = true; }
};

bool steam_audio_simulator::rebind_locked() {
	if (!simulator) return false;
	if (!needs_rebind()) return true;
	unsigned int gen = 0;
	IPLScene s = scene->acquire(gen);
	if (!s) return false;
	iplSimulatorSetScene(simulator, s);
	iplSimulatorCommit(simulator);
	if (bound_scene) iplSceneRelease(&bound_scene);
	bound_scene = s;
	bound_generation = gen;
	bound = true;
	return true;
}

steam_audio_source* steam_audio_simulator::add_source() {
	if (!simulator) {
		script_exception("steam_audio_simulator could not be created");
		return nullptr;
	}
	IPLSourceSettings settings{};
	settings.flags = IPL_SIMULATIONFLAGS_DIRECT;
	if (reflection_type != SA_REFLECTIONS_NONE) settings.flags = IPLSimulationFlags(settings.flags | IPL_SIMULATIONFLAGS_REFLECTIONS);
	IPLSource src = nullptr;
	IPLerror err = iplSourceCreate(simulator, &settings, &src);
	if (err != IPL_STATUS_SUCCESS) {
		set_error(err);
		return nullptr;
	}
	steam_audio_source* s = new steam_audio_source(this, src);
	{
		std::lock_guard<std::mutex> lock(run_mutex);
		std::lock_guard<std::mutex> lock2(direct_mutex);
		iplSourceAdd(src, simulator);
		iplSimulatorCommit(simulator);
	}
	std::lock_guard<std::mutex> lock(sources_mutex);
	sources.push_back(s);
	return s;
}

bool steam_audio_simulator::run_direct() {
	if (!simulator) return false;
	if (needs_rebind()) {
		std::lock_guard<std::mutex> lock(run_mutex);
		if (needs_rebind() && !rebind()) return false;
	}
	std::lock_guard<std::mutex> direct_lock(direct_mutex);
	IPLSimulationSharedInputs shared{};
	shared.listener = make_space(get_listener());
	iplSimulatorSetSharedInputs(simulator, IPL_SIMULATIONFLAGS_DIRECT, &shared);
	std::lock_guard<std::mutex> lock(sources_mutex);
	for (steam_audio_source* s : sources) {
		IPLSimulationInputs in{};
		in.flags = IPL_SIMULATIONFLAGS_DIRECT;
		in.directFlags = IPLDirectSimulationFlags(IPL_DIRECTSIMULATIONFLAGS_OCCLUSION | IPL_DIRECTSIMULATIONFLAGS_TRANSMISSION);
		in.source = make_space(s->get_position());
		in.occlusionType = IPL_OCCLUSIONTYPE_RAYCAST;
		in.numTransmissionRays = 1;
		iplSourceSetInputs(s->source, IPL_SIMULATIONFLAGS_DIRECT, &in);
	}
	iplSimulatorRunDirect(simulator);
	for (steam_audio_source* s : sources) {
		IPLSimulationOutputs out{};
		iplSourceGetOutputs(s->source, IPL_SIMULATIONFLAGS_DIRECT, &out);
		s->set_direct(out.direct);
	}
	return true;
}

void steam_audio_simulator::reflections_locked() {
	IPLSimulationSharedInputs shared{};
	shared.listener = make_space(get_listener());
	shared.numRays = num_rays;
	shared.numBounces = num_bounces;
	shared.duration = duration;
	shared.order = sa_ambisonics_order;
	shared.irradianceMinDistance = 1.0f;
	iplSimulatorSetSharedInputs(simulator, IPL_SIMULATIONFLAGS_REFLECTIONS, &shared);
	std::vector<steam_audio_source*> run;
	{
		std::lock_guard<std::mutex> lock(sources_mutex);
		run = sources;
	}
	if (run.empty()) return;
	for (steam_audio_source* s : run) {
		IPLSimulationInputs in{};
		in.flags = IPL_SIMULATIONFLAGS_REFLECTIONS;
		in.source = make_space(s->get_position());
		in.reverbScale[0] = in.reverbScale[1] = in.reverbScale[2] = 1.0f;
		in.hybridReverbTransitionTime = 0.1f; // convolution only for the early part, parametric tail afterwards
		in.hybridReverbOverlapPercent = 0.25f;
		in.baked = IPL_FALSE;
		iplSourceSetInputs(s->source, IPL_SIMULATIONFLAGS_REFLECTIONS, &in);
	}
	iplSimulatorRunReflections(simulator);
	// Sources removed meanwhile wait for run_mutex in remove(), so every pointer in run is still alive here.
	for (steam_audio_source* s : run) s->set_has_reflections();
	reflection_runs++;
}

void steam_audio_simulator::thread_func() {
	while (!thread_stop) {
		{
			std::lock_guard<std::mutex> lock(run_mutex);
			if (!needs_rebind() || rebind()) reflections_locked();
		}
		std::unique_lock<std::mutex> wait_lock(thread_wait_mutex);
		thread_wait.wait_for(wait_lock, std::chrono::milliseconds(thread_interval_ms), [this] { return thread_stop.load(); });
	}
}

static steam_audio_simulator* simulator_factory(steam_audio_scene* scene, int reflection_type, int max_sources) {
	if (!scene) {
		script_exception("steam_audio_simulator needs a scene");
		return nullptr;
	}
	if (reflection_type < SA_REFLECTIONS_NONE || reflection_type > SA_REFLECTIONS_HYBRID) reflection_type = SA_REFLECTIONS_NONE;
	return new steam_audio_simulator(scene, reflection_type, max_sources);
}

// ---------------------------------------------------------------------------------------------------------------
// steam_audio_reverb_node

class steam_audio_reverb_node_impl : public audio_plugin_node_interface {
	std::mutex mutex; // guards source and the effect state between the script thread and the audio thread
	steam_audio_source* source = nullptr;
	bool needs_reset = false;
	IPLReflectionEffect effect = nullptr;
	IPLAmbisonicsDecodeEffect decode = nullptr;
	IPLAudioBuffer mono{}, ambisonics{}, decoded{};
	bool ready = false;
public:
	unsigned int channels;
	std::atomic<float> wet{0.35f};
	explicit steam_audio_reverb_node_impl(unsigned int ch) : channels(ch) {
		IPLContext ctx = get_context();
		IPLHRTF hrtf = get_hrtf();
		if (!ctx || !hrtf) return;
		IPLAudioSettings audio{sa_sample_rate, sa_frame_size};
		IPLReflectionEffectSettings rs{};
		rs.type = IPL_REFLECTIONEFFECTTYPE_HYBRID;
		rs.irSize = int(sa_sample_rate * sa_max_reflection_duration);
		rs.numChannels = sa_ambisonics_channels;
		IPLerror err = iplReflectionEffectCreate(ctx, &audio, &rs, &effect);
		if (err != IPL_STATUS_SUCCESS) { set_error(err); return; }
		IPLAmbisonicsDecodeEffectSettings ds{};
		ds.speakerLayout.type = IPL_SPEAKERLAYOUTTYPE_STEREO;
		ds.hrtf = hrtf;
		ds.maxOrder = sa_ambisonics_order;
		err = iplAmbisonicsDecodeEffectCreate(ctx, &audio, &ds, &decode);
		if (err != IPL_STATUS_SUCCESS) { set_error(err); return; }
		iplAudioBufferAllocate(ctx, 1, sa_frame_capacity, &mono);
		iplAudioBufferAllocate(ctx, sa_ambisonics_channels, sa_frame_capacity, &ambisonics);
		iplAudioBufferAllocate(ctx, 2, sa_frame_capacity, &decoded);
		ready = true;
	}
	bool set_source(steam_audio_source* s) {
		if (s && (s->get_owner()->get_reflection_type() != SA_REFLECTIONS_HYBRID || !s->get_attached())) return false;
		if (s) s->add_ref();
		steam_audio_source* old;
		{
			std::lock_guard<std::mutex> lock(mutex);
			old = source;
			source = s;
			needs_reset = true;
		}
		if (old) old->release(); // outside the lock: this may destroy the source
		return true;
	}
	void process(audio_plugin_node_interface* node, const float** in, unsigned int* in_count, float** out, unsigned int* out_count) override {
		unsigned int n = std::min(in_count[0], out_count[0]);
		out_count[0] = n;
		if (!in[0] || !out[0] || n == 0) return;
		const unsigned int total = n * channels;
		std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
		// Dry passthrough whenever the reverb can not be applied right now: no source, no finished simulation yet
		// (an empty impulse response permanently corrupts the effect's internal state), or an unexpected format.
		if (!lock.owns_lock() || !ready || !source || !source->get_attached() || !source->get_has_reflections() || n > sa_frame_capacity || channels != 2) {
			std::copy(in[0], in[0] + total, out[0]);
			return;
		}
		if (needs_reset) {
			iplReflectionEffectReset(effect);
			iplAmbisonicsDecodeEffectReset(decode);
			needs_reset = false;
		}
		IPLSimulationOutputs outputs{};
		iplSourceGetOutputs(source->get_ipl_source(), IPL_SIMULATIONFLAGS_REFLECTIONS, &outputs);
		for (unsigned int i = 0; i < n; i++) mono.data[0][i] = 0.5f * (in[0][i * 2] + in[0][i * 2 + 1]);
		mono.numSamples = ambisonics.numSamples = decoded.numSamples = int(n);
		iplReflectionEffectApply(effect, &outputs.reflections, &mono, &ambisonics, nullptr);
		IPLAmbisonicsDecodeEffectParams dp{};
		dp.order = sa_ambisonics_order;
		dp.hrtf = get_hrtf();
		dp.orientation = make_space({0.0f, 0.0f, 0.0f});
		dp.binaural = IPL_FALSE; // the input already went through the HRTF spatializer
		iplAmbisonicsDecodeEffectApply(decode, &dp, &ambisonics, &decoded);
		const float w = clamp01(wet.load());
		for (unsigned int i = 0; i < n; i++) {
			float wl = decoded.data[0][i], wr = decoded.data[1][i];
			if (!std::isfinite(wl) || !std::isfinite(wr)) wl = wr = 0.0f;
			out[0][i * 2] = in[0][i * 2] * (1.0f - w) + wl * w;
			out[0][i * 2 + 1] = in[0][i * 2 + 1] * (1.0f - w) + wr * w;
		}
	}
};

// NVGT does not tell a plugin when one of its nodes is destroyed, so the implementations live as long as the
// process. The pool is never freed on purpose: freeing it at exit could release sources after the script engine
// is already gone.
static std::mutex g_node_pool_mutex;
static std::vector<steam_audio_reverb_node_impl*>* g_node_pool = new std::vector<steam_audio_reverb_node_impl*>();

static plugin_node* reverb_node_create(audio_engine* engine, unsigned int channels) {
	steam_audio_reverb_node_impl* impl = new steam_audio_reverb_node_impl(channels);
	{
		std::lock_guard<std::mutex> lock(g_node_pool_mutex);
		g_node_pool->push_back(impl);
	}
	return nvgt_audio_plugin_node_create(impl, 1, 1, 0, engine);
}
static steam_audio_reverb_node_impl* reverb_impl(plugin_node* node) {
	return node ? static_cast<steam_audio_reverb_node_impl*>(nvgt_audio_plugin_node_get(node)) : nullptr;
}
static bool reverb_node_set_source(plugin_node* node, steam_audio_source* source) {
	steam_audio_reverb_node_impl* impl = reverb_impl(node);
	return impl ? impl->set_source(source) : false;
}
static void reverb_node_set_wet(plugin_node* node, float wet) {
	if (steam_audio_reverb_node_impl* impl = reverb_impl(node)) impl->wet = clamp01(wet);
}
static float reverb_node_get_wet(plugin_node* node) {
	steam_audio_reverb_node_impl* impl = reverb_impl(node);
	return impl ? impl->wet.load() : 0.0f;
}

// ---------------------------------------------------------------------------------------------------------------
// registration

static int get_last_error() { return g_last_error; }
static bool get_available() { return get_context() != nullptr; }

#define CHECK(x) if ((x) < 0) return false

static bool register_ref_type(asIScriptEngine* e, const char* name) {
	CHECK(e->RegisterObjectType(name, 0, asOBJ_REF));
	CHECK(e->RegisterObjectBehaviour(name, asBEHAVE_ADDREF, "void f()", asMETHOD(ref_counted, add_ref), asCALL_THISCALL));
	CHECK(e->RegisterObjectBehaviour(name, asBEHAVE_RELEASE, "void f()", asMETHOD(ref_counted, release), asCALL_THISCALL));
	return true;
}

plugin_main(nvgt_plugin_shared* shared) {
	if (!prepare_plugin(shared)) return false;
	asIScriptEngine* e = shared->script_engine;
	CHECK(e->RegisterEnum("steam_audio_reflection_type"));
	CHECK(e->RegisterEnumValue("steam_audio_reflection_type", "STEAM_AUDIO_REFLECTIONS_NONE", SA_REFLECTIONS_NONE));
	CHECK(e->RegisterEnumValue("steam_audio_reflection_type", "STEAM_AUDIO_REFLECTIONS_PARAMETRIC", SA_REFLECTIONS_PARAMETRIC));
	CHECK(e->RegisterEnumValue("steam_audio_reflection_type", "STEAM_AUDIO_REFLECTIONS_HYBRID", SA_REFLECTIONS_HYBRID));
	CHECK(e->RegisterGlobalProperty("const int STEAM_AUDIO_BAND_LOW", (void*)new int(0)));
	CHECK(e->RegisterGlobalProperty("const int STEAM_AUDIO_BAND_MID", (void*)new int(1)));
	CHECK(e->RegisterGlobalProperty("const int STEAM_AUDIO_BAND_HIGH", (void*)new int(2)));
	CHECK(e->RegisterGlobalFunction("int steam_audio_get_last_error()", asFUNCTION(get_last_error), asCALL_CDECL));
	CHECK(e->RegisterGlobalFunction("bool steam_audio_get_available()", asFUNCTION(get_available), asCALL_CDECL));

	if (!register_ref_type(e, "steam_audio_scene")) return false;
	CHECK(e->RegisterObjectBehaviour("steam_audio_scene", asBEHAVE_FACTORY, "steam_audio_scene@ f()", asFUNCTION(scene_factory), asCALL_CDECL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "int add_material(float absorption_low, float absorption_mid, float absorption_high, float scattering, float transmission_low, float transmission_mid, float transmission_high)", asMETHOD(steam_audio_scene, add_material), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "bool add_box(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, int material)", asMETHOD(steam_audio_scene, add_box), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "bool add_triangle(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3, int material)", asMETHOD(steam_audio_scene, add_triangle), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "void clear()", asMETHOD(steam_audio_scene, clear), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "void clear_materials()", asMETHOD(steam_audio_scene, clear_materials), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "bool commit()", asMETHOD(steam_audio_scene, commit), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "uint get_triangle_count() const property", asMETHOD(steam_audio_scene, get_triangle_count), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "uint get_material_count() const property", asMETHOD(steam_audio_scene, get_material_count), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_scene", "bool get_committed() property", asMETHOD(steam_audio_scene, get_committed), asCALL_THISCALL));

	if (!register_ref_type(e, "steam_audio_source")) return false;
	if (!register_ref_type(e, "steam_audio_simulator")) return false;
	CHECK(e->RegisterObjectBehaviour("steam_audio_simulator", asBEHAVE_FACTORY, "steam_audio_simulator@ f(steam_audio_scene@+ scene, steam_audio_reflection_type reflections = STEAM_AUDIO_REFLECTIONS_NONE, int max_sources = 32)", asFUNCTION(simulator_factory), asCALL_CDECL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "bool get_valid() const property", asMETHOD(steam_audio_simulator, valid), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "void set_listener(float x, float y, float z)", asMETHOD(steam_audio_simulator, set_listener), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "void set_reflection_quality(int rays, int bounces, float duration)", asMETHOD(steam_audio_simulator, set_reflection_quality), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "steam_audio_source@ add_source()", asMETHOD(steam_audio_simulator, add_source), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "bool run_direct()", asMETHOD(steam_audio_simulator, run_direct), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "bool run_reflections()", asMETHOD(steam_audio_simulator, run_reflections), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "bool start_reflection_thread(uint interval_ms = 1000)", asMETHOD(steam_audio_simulator, start_reflection_thread), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "void stop_reflection_thread()", asMETHOD(steam_audio_simulator, stop_reflection_thread), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "uint get_reflection_runs() const property", asMETHOD(steam_audio_simulator, get_reflection_runs), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_simulator", "bool get_reflection_thread_running() const property", asMETHOD(steam_audio_simulator, get_reflection_thread_running), asCALL_THISCALL));

	CHECK(e->RegisterObjectMethod("steam_audio_source", "void set_position(float x, float y, float z)", asMETHOD(steam_audio_source, set_position), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_source", "float get_occlusion() const property", asMETHOD(steam_audio_source, get_occlusion), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_source", "float get_transmission(int band) const", asMETHOD(steam_audio_source, get_transmission), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_source", "float get_reverb_time(int band)", asMETHOD(steam_audio_source, get_reverb_time), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_source", "bool get_has_reflections() const property", asMETHOD(steam_audio_source, get_has_reflections), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_source", "bool get_attached() const property", asMETHOD(steam_audio_source, get_attached), asCALL_THISCALL));
	CHECK(e->RegisterObjectMethod("steam_audio_source", "void remove()", asMETHOD(steam_audio_source, remove), asCALL_THISCALL));

	nvgt_audio_plugin_node_register("steam_audio_reverb_node");
	CHECK(e->RegisterGlobalFunction("steam_audio_reverb_node@ steam_audio_reverb_node_create(audio_engine@ engine = sound_default_engine, uint channels = 2)", asFUNCTION(reverb_node_create), asCALL_CDECL));
	CHECK(e->RegisterObjectMethod("steam_audio_reverb_node", "bool set_source(steam_audio_source@+ source)", asFUNCTION(reverb_node_set_source), asCALL_CDECL_OBJFIRST));
	CHECK(e->RegisterObjectMethod("steam_audio_reverb_node", "void set_wet(float wet) property", asFUNCTION(reverb_node_set_wet), asCALL_CDECL_OBJFIRST));
	CHECK(e->RegisterObjectMethod("steam_audio_reverb_node", "float get_wet() property", asFUNCTION(reverb_node_get_wet), asCALL_CDECL_OBJFIRST));
	return true;
}
