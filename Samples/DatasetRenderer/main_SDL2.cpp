// DatasetRenderer
//	Command line tool that renders paired images of a 3D asset with:
//		- the WickedEngine rasterizer (RenderPath3D: shadow maps, PBR, IBL from sky, SSAO, TAA...)
//		- the WickedEngine path tracer (RenderPath3D_PathTracing), denoised with Open Image Denoise
//	Camera and lights are randomized (seeded) by the tool, every view is written with a JSON metadata file.
//	With --animate, camera animations (orbits, translations, mixes) are rendered instead of independent views,
//	the camera paths are kept away from the scene geometry.
//	Run with --help for usage.

#include "WickedEngine.h"
#include "ModelImporter.h"
#include "json.hpp"

#include <SDL2/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace wi::ecs;
using namespace wi::scene;
using namespace wi::graphics;

#ifndef DATASET_RENDERER_SHADER_DIR
#define DATASET_RENDERER_SHADER_DIR ""
#endif

namespace
{
	struct Options
	{
		std::string input;
		std::string outDir = "dataset_out";
		int width = 512;
		int height = 512;
		int views = 8;
		int firstView = 0;
		int seed = 0;

		// camera:
		float fov = 45; // vertical, degrees
		float distMin = 1.0f; // multiple of the distance that fits the bounding sphere in view
		float distMax = 1.4f;
		float elevMin = -5; // degrees
		float elevMax = 60;
		std::string focus = "object"; // object: random object of the scene, scene: whole model bounds
		float minTargetSize = 0.1f; // relative to the model radius
		float clearance = 0.02f; // relative to the target radius
		int maxAttempts = 200;
		float minCoverage = 0.05f; // fraction of the image covered by the visible target
		float minBrightness = 0.05f; // mean display value of the visible target pixels

		// animation:
		std::string animate; // empty: independent views
		int frames = 24;
		float orbitMin = 20; // degrees of azimuth
		float orbitMax = 90;
		float moveMin = 0.15f; // relative to the camera-target distance
		float moveMax = 0.5f;
		float minSplit = 0.5f;

		// path tracer:
		int spp = 256;
		int bounces = -1; // -1: engine default
		bool denoise = true;

		// rasterizer:
		int rasterFrames = 32;
		std::string aa = "taa";
		std::string ao = "msao";
		std::string gi = "none";
		bool ssr = false;
		int shadowRes = 2048;

		// shared post process:
		float exposure = 1;
		std::string tonemap = "aces";
		bool bloom = false;

		// lighting:
		bool keepSceneLighting = false;
		float sunIntensity = 8;
		float sunElevMin = 15;
		float sunElevMax = 75;
		int pointLights = 0;
		float pointIntensity = 4;
		std::string sky = "realistic";
		std::string hdri;
		bool rotateHdri = false;
		float skyIntensity = 1;
		bool ground = false;

		// outputs:
		bool saveNoisy = false;
		bool saveHDR = false;
		bool saveAux = false;

		// misc:
		std::string shaderDir = DATASET_RENDERER_SHADER_DIR;
		bool hidden = false;
		std::string gpu = "discrete";
	};

	void PrintUsage()
	{
		std::printf(R"(Usage: DatasetRenderer <model.(gltf|glb|vrm|fbx|obj|wiscene)> [options]

Renders paired images (rasterized, path traced) of a model from randomized views.

Output:
  -o, --out DIR            output directory (default: dataset_out)
  --width N, --height N    image resolution (default: 512x512)
  --views N                number of views (default: 8)
  --first-view N           index of the first view, to resume or split jobs (default: 0)
  --seed N                 random seed (default: 0). View i only depends on (seed, i)
  --save-noisy             also save the non-denoised path traced image
  --save-hdr               also save linear HDR radiance (before exposure/tonemap) as .pfm
  --save-aux               also save path tracer albedo and normal buffers as .pfm

Camera (placed around the bounding sphere of the focus, looking at its center or at a random point of its surface):
  --focus object|scene     object: a random object of the scene per view/animation, scene: the whole model
                           (default: object)
  --min-target-size X      min radius of focus objects, relative to the model radius (default: 0.1)
  --fov DEG                vertical field of view (default: 45)
  --dist-min X, --dist-max X
                           distance range, relative to the framing distance (default: 1.0 1.4)
  --elev-min DEG, --elev-max DEG
                           camera elevation range (default: -5 60)
  --clearance X            min distance from the camera to the geometry, relative to the focus radius (default: 0.02)
  --min-coverage X         min fraction of the image covered by the visible focus at the start (default: 0.05)
  --min-brightness X       min mean display value [0, 1] of the focus pixels in a low sample count path traced
                           preview of the start (default: 0.05, 0 disables the preview)
  --max-attempts N         starting points (or paths) drawn per view/animation before giving up, a new focus
                           object is drawn every N/10 attempts (default: 200)

Animation (camera paths instead of independent views):
  --animate orbit|translate|mix|random
                           render camera animations. --views/--first-view then count animations, written to
                           DIR/anim_XXXX/{raster,pathtraced,...}/FFFF.png with DIR/anim_XXXX/animation.json
                           and motion vectors in DIR/anim_XXXX/motion/FFFF.npy (float32, shape (height, width, 2):
                           pixel offset (x right, y down) from frame F to where the surface was in frame F-1)
                             orbit:     orbit around the focus
                             translate: forward, backward or lateral camera translation
                             mix:       2 or 3 chained orbit/translation segments (orbits also dolly)
                             random:    one of the above per animation
  --frames N               frames per animation (default: 24)
  --orbit-min DEG, --orbit-max DEG
                           orbit azimuth sweep (default: 20 90)
  --move-min X, --move-max X
                           translation length, relative to the camera-target distance (default: 0.15 0.5)
  --min-split X            colliding paths are cut to their longest valid part if it keeps at least
                           this fraction of the path, otherwise a new path is drawn (default: 0.5)

Lighting:
  --sun-intensity X        directional sun intensity, 0 disables it (default: 8)
  --sun-elev-min DEG, --sun-elev-max DEG
                           sun elevation range, azimuth is uniform (default: 15 75)
  --point-lights N         number of random point lights around the model (default: 0)
  --point-intensity X      point light irradiance at the model center (default: 4)
  --sky gradient|realistic|none
                           sky model (default: realistic)
  --hdri FILE              use an environment map (.hdr, .dds, ...; equirect or cubemap) as sky
  --rotate-hdri            randomize the environment map rotation per view
  --sky-intensity X        sky exposure multiplier (default: 1)
  --ground                 add a large grey ground plane under the model
  --keep-scene-lighting    keep lights, weather and probes from the file instead of the tool lights

Path tracer:
  --spp N                  samples per pixel (default: 256)
  --bounces N              max bounces (default: engine default)
  --no-denoise             disable Open Image Denoise

Rasterizer:
  --raster-frames N        frames rendered before capture, for TAA/GI convergence (default: 32)
  --aa taa|msaa|fxaa|none  anti-aliasing (default: taa)
  --ao none|ssao|hbao|msao screen space ambient occlusion (default: msao)
  --gi none|ddgi|vxgi|surfel
                           real-time global illumination (default: none). Needs more --raster-frames
  --ssr                    enable screen space reflections
  --shadow-res N           shadow map resolution (default: 2048)

Shared post process (applied identically to both renderers):
  --exposure X             (default: 1)
  --tonemap aces|reinhard|uchimura  (default: aces)
  --bloom                  enable bloom

Misc:
  --shader-dir DIR         shader cache directory (default: %s)
  --hidden                 hide the preview window
  --gpu discrete|integrated|nvidia|amd|intel
                           graphics adapter to use: the first one of this type or vendor (default: discrete)
  -h, --help               show this help
)", DATASET_RENDERER_SHADER_DIR);
	}

	bool ToInt(const std::string& s, int& out)
	{
		char* end = nullptr;
		long v = std::strtol(s.c_str(), &end, 10);
		if (s.empty() || end == nullptr || *end != 0)
			return false;
		out = (int)v;
		return true;
	}
	bool ToFloat(const std::string& s, float& out)
	{
		char* end = nullptr;
		float v = std::strtof(s.c_str(), &end);
		if (s.empty() || end == nullptr || *end != 0 || !std::isfinite(v))
			return false;
		out = v;
		return true;
	}

	// Returns 0 on success, 1 on error, -1 when the program should exit without error (help)
	int ParseArgs(int argc, char* argv[], Options& o)
	{
		for (int i = 1; i < argc; ++i)
		{
			const std::string a = argv[i];

			auto nextStr = [&](std::string& out) {
				if (i + 1 >= argc)
				{
					std::fprintf(stderr, "Missing value for %s\n", a.c_str());
					return false;
				}
				out = argv[++i];
				return true;
			};
			auto nextInt = [&](int& out) {
				std::string s;
				if (!nextStr(s))
					return false;
				if (!ToInt(s, out))
				{
					std::fprintf(stderr, "Invalid integer for %s: %s\n", a.c_str(), s.c_str());
					return false;
				}
				return true;
			};
			auto nextFloat = [&](float& out) {
				std::string s;
				if (!nextStr(s))
					return false;
				if (!ToFloat(s, out))
				{
					std::fprintf(stderr, "Invalid number for %s: %s\n", a.c_str(), s.c_str());
					return false;
				}
				return true;
			};

			bool ok = true;
			if (a == "-h" || a == "--help") { PrintUsage(); return -1; }
			else if (a == "-o" || a == "--out") ok = nextStr(o.outDir);
			else if (a == "--width") ok = nextInt(o.width);
			else if (a == "--height") ok = nextInt(o.height);
			else if (a == "--views") ok = nextInt(o.views);
			else if (a == "--first-view") ok = nextInt(o.firstView);
			else if (a == "--seed") ok = nextInt(o.seed);
			else if (a == "--save-noisy") o.saveNoisy = true;
			else if (a == "--save-hdr") o.saveHDR = true;
			else if (a == "--save-aux") o.saveAux = true;
			else if (a == "--fov") ok = nextFloat(o.fov);
			else if (a == "--dist-min") ok = nextFloat(o.distMin);
			else if (a == "--dist-max") ok = nextFloat(o.distMax);
			else if (a == "--elev-min") ok = nextFloat(o.elevMin);
			else if (a == "--elev-max") ok = nextFloat(o.elevMax);
			else if (a == "--focus") ok = nextStr(o.focus);
			else if (a == "--min-coverage") ok = nextFloat(o.minCoverage);
			else if (a == "--min-brightness") ok = nextFloat(o.minBrightness);
			else if (a == "--animate") ok = nextStr(o.animate);
			else if (a == "--frames") ok = nextInt(o.frames);
			else if (a == "--orbit-min") ok = nextFloat(o.orbitMin);
			else if (a == "--orbit-max") ok = nextFloat(o.orbitMax);
			else if (a == "--move-min") ok = nextFloat(o.moveMin);
			else if (a == "--move-max") ok = nextFloat(o.moveMax);
			else if (a == "--clearance") ok = nextFloat(o.clearance);
			else if (a == "--min-split") ok = nextFloat(o.minSplit);
			else if (a == "--max-attempts") ok = nextInt(o.maxAttempts);
			else if (a == "--min-target-size") ok = nextFloat(o.minTargetSize);
			else if (a == "--sun-intensity") ok = nextFloat(o.sunIntensity);
			else if (a == "--sun-elev-min") ok = nextFloat(o.sunElevMin);
			else if (a == "--sun-elev-max") ok = nextFloat(o.sunElevMax);
			else if (a == "--point-lights") ok = nextInt(o.pointLights);
			else if (a == "--point-intensity") ok = nextFloat(o.pointIntensity);
			else if (a == "--sky") ok = nextStr(o.sky);
			else if (a == "--hdri") ok = nextStr(o.hdri);
			else if (a == "--rotate-hdri") o.rotateHdri = true;
			else if (a == "--sky-intensity") ok = nextFloat(o.skyIntensity);
			else if (a == "--ground") o.ground = true;
			else if (a == "--keep-scene-lighting") o.keepSceneLighting = true;
			else if (a == "--spp") ok = nextInt(o.spp);
			else if (a == "--bounces") ok = nextInt(o.bounces);
			else if (a == "--no-denoise") o.denoise = false;
			else if (a == "--raster-frames") ok = nextInt(o.rasterFrames);
			else if (a == "--aa") ok = nextStr(o.aa);
			else if (a == "--ao") ok = nextStr(o.ao);
			else if (a == "--gi") ok = nextStr(o.gi);
			else if (a == "--ssr") o.ssr = true;
			else if (a == "--shadow-res") ok = nextInt(o.shadowRes);
			else if (a == "--exposure") ok = nextFloat(o.exposure);
			else if (a == "--tonemap") ok = nextStr(o.tonemap);
			else if (a == "--bloom") o.bloom = true;
			else if (a == "--shader-dir") ok = nextStr(o.shaderDir);
			else if (a == "--hidden") o.hidden = true;
			else if (a == "--gpu") ok = nextStr(o.gpu);
			else if (a.size() > 1 && a[0] == '-')
			{
				std::fprintf(stderr, "Unknown option: %s (see --help)\n", a.c_str());
				return 1;
			}
			else if (o.input.empty()) o.input = a;
			else
			{
				std::fprintf(stderr, "Only one input model is supported (got '%s' and '%s')\n", o.input.c_str(), a.c_str());
				return 1;
			}
			if (!ok)
				return 1;
		}

		auto oneOf = [](const std::string& v, std::initializer_list<const char*> list) {
			for (auto& x : list)
				if (v == x)
					return true;
			return false;
		};

		if (o.input.empty()) { std::fprintf(stderr, "No input model given (see --help)\n"); return 1; }
		if (o.width <= 0 || o.height <= 0 || o.width > 16384 || o.height > 16384) { std::fprintf(stderr, "Invalid resolution\n"); return 1; }
		if (o.views <= 0 || o.firstView < 0) { std::fprintf(stderr, "Invalid view count\n"); return 1; }
		if (o.spp <= 0) { std::fprintf(stderr, "--spp must be > 0\n"); return 1; }
		if (o.rasterFrames <= 0) { std::fprintf(stderr, "--raster-frames must be > 0\n"); return 1; }
		if (o.fov <= 1 || o.fov >= 170) { std::fprintf(stderr, "--fov must be in ]1, 170[\n"); return 1; }
		if (o.distMin <= 0 || o.distMax < o.distMin) { std::fprintf(stderr, "Invalid --dist-min/--dist-max\n"); return 1; }
		if (o.elevMax < o.elevMin || o.sunElevMax < o.sunElevMin) { std::fprintf(stderr, "Invalid elevation range\n"); return 1; }
		if (o.pointLights < 0 || o.shadowRes <= 0) { std::fprintf(stderr, "Invalid light settings\n"); return 1; }
		if (!oneOf(o.aa, { "taa", "msaa", "fxaa", "none" })) { std::fprintf(stderr, "Invalid --aa\n"); return 1; }
		if (!oneOf(o.ao, { "none", "ssao", "hbao", "msao" })) { std::fprintf(stderr, "Invalid --ao\n"); return 1; }
		if (!oneOf(o.gi, { "none", "ddgi", "vxgi", "surfel" })) { std::fprintf(stderr, "Invalid --gi\n"); return 1; }
		if (!oneOf(o.sky, { "gradient", "realistic", "none" })) { std::fprintf(stderr, "Invalid --sky\n"); return 1; }
		if (!oneOf(o.tonemap, { "aces", "reinhard", "uchimura" })) { std::fprintf(stderr, "Invalid --tonemap\n"); return 1; }
		if (!oneOf(o.focus, { "object", "scene" })) { std::fprintf(stderr, "Invalid --focus\n"); return 1; }
		if (!oneOf(o.gpu, { "discrete", "integrated", "nvidia", "amd", "intel" })) { std::fprintf(stderr, "Invalid --gpu\n"); return 1; }
		if (o.clearance <= 0) { std::fprintf(stderr, "--clearance must be > 0\n"); return 1; }
		if (o.maxAttempts <= 0) { std::fprintf(stderr, "--max-attempts must be > 0\n"); return 1; }
		if (o.minTargetSize < 0) { std::fprintf(stderr, "--min-target-size must be >= 0\n"); return 1; }
		if (o.minCoverage < 0 || o.minCoverage > 1) { std::fprintf(stderr, "--min-coverage must be in [0, 1]\n"); return 1; }
		if (o.minBrightness < 0 || o.minBrightness > 1) { std::fprintf(stderr, "--min-brightness must be in [0, 1]\n"); return 1; }
		if (!o.animate.empty())
		{
			if (!oneOf(o.animate, { "orbit", "translate", "mix", "random" })) { std::fprintf(stderr, "Invalid --animate\n"); return 1; }
			if (o.frames <= 0) { std::fprintf(stderr, "--frames must be > 0\n"); return 1; }
			if (o.orbitMin < 0 || o.orbitMax < o.orbitMin) { std::fprintf(stderr, "Invalid --orbit-min/--orbit-max\n"); return 1; }
			if (o.moveMin < 0 || o.moveMax < o.moveMin) { std::fprintf(stderr, "Invalid --move-min/--move-max\n"); return 1; }
			if (o.minSplit <= 0 || o.minSplit > 1) { std::fprintf(stderr, "--min-split must be in ]0, 1]\n"); return 1; }
		}
		if (o.saveHDR && o.aa == "fxaa") { std::fprintf(stderr, "--save-hdr is not supported with --aa fxaa (FXAA overwrites the raster HDR buffer)\n"); return 1; }

		std::error_code ec;
		if (!std::filesystem::is_regular_file(o.input, ec)) { std::fprintf(stderr, "Input file not found: %s\n", o.input.c_str()); return 1; }
		o.input = std::filesystem::absolute(o.input, ec).generic_string();
		if (!o.hdri.empty())
		{
			if (!std::filesystem::is_regular_file(o.hdri, ec)) { std::fprintf(stderr, "HDRI file not found: %s\n", o.hdri.c_str()); return 1; }
			o.hdri = std::filesystem::absolute(o.hdri, ec).generic_string();
		}
		if (!o.shaderDir.empty() && o.shaderDir.back() != '/')
		{
			o.shaderDir += '/';
		}
		return 0;
	}

	// Rasterizer render path, exposes the HDR (pre-tonemap) result
	class DatasetRasterizer : public wi::RenderPath3D
	{
	public:
		const Texture& GetHDRResult() const
		{
			if (wi::renderer::GetTemporalAAEnabled() && temporalAAResources.IsValid())
				return *temporalAAResources.GetCurrent();
			return rtMain;
		}
	};

	// Path tracer render path, exposes the internal accumulation/denoiser buffers
	class DatasetPathTracer : public wi::RenderPath3D_PathTracing
	{
	public:
		bool IsDenoiseFinished() const { return denoiserResult.IsValid() && !wi::jobsystem::IsBusy(denoiserContext); }
		const Texture& GetNoisyHDR() const { return traceResult; }
		const Texture& GetDenoisedHDR() const { return denoiserResult; }
		const Texture& GetAlbedo() const { return denoiserAlbedo; }
		const Texture& GetNormal() const { return denoiserNormal; }
	};

	// Reads back a texture as float RGB values, without color space conversion
	bool ReadTextureRGB(const Texture& texture, std::vector<XMFLOAT3>& rgb, uint32_t& width, uint32_t& height)
	{
		if (!texture.IsValid())
			return false;
		wi::vector<uint8_t> data;
		if (!wi::helper::saveTextureToMemory(texture, data))
			return false;

		const TextureDesc& desc = texture.GetDesc();
		width = desc.width;
		height = desc.height;
		const size_t count = size_t(width) * size_t(height);
		const uint32_t stride = GetFormatStride(desc.format);
		if (data.size() < count * stride)
			return false;

		rgb.resize(count);
		for (size_t i = 0; i < count; ++i)
		{
			XMFLOAT3 c = {};
			const uint8_t* src = data.data() + i * stride;
			switch (desc.format)
			{
			case Format::R32G32B32A32_FLOAT:
			case Format::R32G32B32_FLOAT:
				std::memcpy(&c, src, sizeof(XMFLOAT3));
				break;
			case Format::R16G16B16A16_FLOAT:
			{
				XMHALF4 h;
				std::memcpy(&h, src, sizeof(h));
				c = XMFLOAT3(XMConvertHalfToFloat(h.x), XMConvertHalfToFloat(h.y), XMConvertHalfToFloat(h.z));
				break;
			}
			case Format::R11G11B10_FLOAT:
			{
				XMFLOAT3PK pk;
				std::memcpy(&pk, src, sizeof(pk));
				XMStoreFloat3(&c, XMLoadFloat3PK(&pk));
				break;
			}
			case Format::R9G9B9E5_SHAREDEXP:
			{
				XMFLOAT3SE se;
				std::memcpy(&se, src, sizeof(se));
				XMStoreFloat3(&c, XMLoadFloat3SE(&se));
				break;
			}
			case Format::R8G8B8A8_UNORM:
			case Format::R8G8B8A8_UNORM_SRGB:
				c = XMFLOAT3(src[0] / 255.0f, src[1] / 255.0f, src[2] / 255.0f);
				break;
			case Format::B8G8R8A8_UNORM:
			case Format::B8G8R8A8_UNORM_SRGB:
				c = XMFLOAT3(src[2] / 255.0f, src[1] / 255.0f, src[0] / 255.0f);
				break;
			case Format::R10G10B10A2_UNORM:
			{
				uint32_t v;
				std::memcpy(&v, src, sizeof(v));
				c = XMFLOAT3((v & 1023) / 1023.0f, ((v >> 10) & 1023) / 1023.0f, ((v >> 20) & 1023) / 1023.0f);
				break;
			}
			default:
				return false;
			}
			rgb[i] = c;
		}
		return true;
	}

	// Write a texture as a portable float map (linear, RGB float32, bottom-to-top rows)
	bool SaveTexturePFM(const Texture& texture, const std::string& fileName)
	{
		std::vector<XMFLOAT3> rgb;
		uint32_t w = 0;
		uint32_t h = 0;
		if (!ReadTextureRGB(texture, rgb, w, h))
		{
			if (texture.IsValid())
				std::fprintf(stderr, "Unsupported texture format for PFM export: %s\n", fileName.c_str());
			return false;
		}
		const size_t width = w;
		const size_t height = h;
		const size_t count = width * height;

		const std::string header = "PF\n" + std::to_string(width) + " " + std::to_string(height) + "\n-1.0\n"; // negative scale: little endian
		std::vector<uint8_t> file(header.size() + count * 3 * sizeof(float));
		std::memcpy(file.data(), header.data(), header.size());
		uint8_t* dst = file.data() + header.size();
		const size_t rowSize = width * 3 * sizeof(float);
		for (size_t y = 0; y < height; ++y)
		{
			// PFM rows are stored bottom to top:
			std::memcpy(dst + y * rowSize, rgb.data() + (height - 1 - y) * width, rowSize);
		}
		return wi::helper::FileWrite(fileName, file.data(), file.size());
	}

	// Reads back the first mip of a R32_FLOAT texture
	bool ReadTextureR32(const Texture& texture, std::vector<float>& values, uint32_t& width, uint32_t& height)
	{
		if (!texture.IsValid() || texture.GetDesc().format != Format::R32_FLOAT)
			return false;
		wi::vector<uint8_t> data;
		if (!wi::helper::saveTextureToMemory(texture, data))
			return false;
		width = texture.GetDesc().width;
		height = texture.GetDesc().height;
		const size_t count = size_t(width) * size_t(height);
		if (data.size() < count * sizeof(float))
			return false;
		values.resize(count);
		std::memcpy(values.data(), data.data(), count * sizeof(float));
		return true;
	}

	// Writes a C-order little endian float32 array as a NumPy .npy file (format version 1.0)
	bool SaveNpy(const std::string& fileName, const float* data, const std::vector<size_t>& shape)
	{
		size_t count = 1;
		std::string dims;
		for (size_t i = 0; i < shape.size(); ++i)
		{
			dims += (i > 0 ? ", " : "") + std::to_string(shape[i]);
			count *= shape[i];
		}
		if (shape.size() == 1)
			dims += ",";
		std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (" + dims + "), }";
		const size_t preamble = 10; // magic, version, header length
		header.append((64 - (preamble + header.size() + 1) % 64) % 64, ' '); // the data starts 64 bytes aligned
		header += '\n';

		std::vector<uint8_t> file;
		file.reserve(preamble + header.size() + count * sizeof(float));
		const uint8_t magic[] = { 0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0 };
		file.insert(file.end(), magic, magic + sizeof(magic));
		file.push_back(uint8_t(header.size() & 0xFF));
		file.push_back(uint8_t(header.size() >> 8));
		file.insert(file.end(), header.begin(), header.end());
		const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
		file.insert(file.end(), bytes, bytes + count * sizeof(float));
		return wi::helper::FileWrite(fileName, file.data(), file.size());
	}

	XMFLOAT3 SphericalDirection(float azimuth, float elevation)
	{
		return XMFLOAT3(
			std::cos(elevation) * std::cos(azimuth),
			std::sin(elevation),
			std::cos(elevation) * std::sin(azimuth)
		);
	}

	// Rotation that maps +Y onto dir (Wicked lights emit along their local +Y axis, directional light direction points towards the light)
	XMVECTOR RotationFromUpTo(const XMFLOAT3& dir)
	{
		const XMVECTOR up = XMVectorSet(0, 1, 0, 0);
		const XMVECTOR d = XMVector3Normalize(XMLoadFloat3(&dir));
		const float cosAngle = XMVectorGetX(XMVector3Dot(up, d));
		if (cosAngle > 0.9999f)
			return XMQuaternionIdentity();
		if (cosAngle < -0.9999f)
			return XMQuaternionRotationAxis(XMVectorSet(1, 0, 0, 0), XM_PI);
		const XMVECTOR axis = XMVector3Normalize(XMVector3Cross(up, d));
		return XMQuaternionRotationAxis(axis, std::acos(cosAngle));
	}

	XMFLOAT3 HSVToRGB(float h, float s, float v)
	{
		auto f = [&](float n) {
			const float k = std::fmod(n + h * 6, 6.0f);
			return v - v * s * std::max(0.0f, std::min({ k, 4 - k, 1.0f }));
		};
		return XMFLOAT3(f(5), f(3), f(1));
	}

	nlohmann::json ToJson(const XMFLOAT3& v) { return { v.x, v.y, v.z }; }
	nlohmann::json ToJson(const XMFLOAT4X4& m)
	{
		nlohmann::json rows = nlohmann::json::array();
		for (int r = 0; r < 4; ++r)
			rows.push_back({ m.m[r][0], m.m[r][1], m.m[r][2], m.m[r][3] });
		return rows;
	}

	struct PointLightSetup
	{
		XMFLOAT3 position;
		XMFLOAT3 color;
		float intensity;
		float range;
	};

	struct LightSetup
	{
		XMFLOAT3 sunDirection;
		float sunAzimuth;
		float sunElevation;
		float skyRotation;
		std::vector<PointLightSetup> pointLights;
	};

	// Lights of view/animation `index`, around the model bounding sphere
	LightSetup SampleLights(const Options& o, int index, const XMFLOAT3& center, float radius)
	{
		std::seed_seq seq{ (uint32_t)o.seed, (uint32_t)index, 0x5eedu };
		std::mt19937 rng(seq);
		auto uniform = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); };

		LightSetup v = {};

		// Sun:
		v.sunAzimuth = uniform(0, XM_2PI);
		v.sunElevation = XMConvertToRadians(uniform(o.sunElevMin, o.sunElevMax));
		v.sunDirection = SphericalDirection(v.sunAzimuth, v.sunElevation);

		v.skyRotation = o.rotateHdri ? uniform(0, XM_2PI) : 0;

		// Point lights, in the upper hemisphere around the model:
		for (int i = 0; i < o.pointLights; ++i)
		{
			PointLightSetup& pl = v.pointLights.emplace_back();
			const float dist = radius * uniform(1.2f, 2.0f);
			const XMFLOAT3 d = SphericalDirection(uniform(0, XM_2PI), XMConvertToRadians(uniform(10, 70)));
			pl.position = XMFLOAT3(center.x + d.x * dist, center.y + d.y * dist, center.z + d.z * dist);
			pl.color = HSVToRGB(uniform(0, 1), uniform(0, 0.5f), 1);
			pl.intensity = o.pointIntensity * dist * dist; // inverse square falloff -> about pointIntensity at the center
			pl.range = dist * 4;
		}

		return v;
	}

	// Camera elevation range in radians
	void ElevationRange(const Options& o, float& elevMin, float& elevMax)
	{
		float elevMinDeg = o.elevMin;
		if (o.ground)
		{
			elevMinDeg = std::max(elevMinDeg, 5.0f); // don't go below the ground
		}
		elevMin = XMConvertToRadians(std::clamp(elevMinDeg, -85.0f, 85.0f));
		elevMax = XMConvertToRadians(std::clamp(std::max(o.elevMax, elevMinDeg), -85.0f, 85.0f));
	}

	void SetTransform(Scene& scene, Entity entity, const XMFLOAT3& position, const XMVECTOR& rotation)
	{
		TransformComponent* transform = scene.transforms.GetComponent(entity);
		if (transform == nullptr)
			return;
		transform->ClearTransform();
		transform->Rotate(rotation);
		transform->Translate(position);
		transform->UpdateTransform();
	}

	std::string ViewName(int index)
	{
		char buf[32];
		std::snprintf(buf, sizeof(buf), "%04d", index);
		return buf;
	}

	bool CreateOutputDirs(const std::filesystem::path& dir, const Options& o, bool withMeta, bool withMotion = false)
	{
		std::vector<std::string> subdirs = { "raster", "pathtraced" };
		if (withMeta) subdirs.push_back("meta");
		if (withMotion) subdirs.push_back("motion");
		if (o.saveNoisy) subdirs.push_back("pathtraced_noisy");
		if (o.saveHDR)
		{
			subdirs.push_back("raster_hdr");
			subdirs.push_back("pathtraced_hdr");
			if (o.saveNoisy) subdirs.push_back("pathtraced_noisy_hdr");
		}
		if (o.saveAux)
		{
			subdirs.push_back("albedo");
			subdirs.push_back("normal");
		}
		for (auto& x : subdirs)
		{
			std::error_code ec;
			std::filesystem::create_directories(dir / x, ec);
			if (ec)
			{
				std::fprintf(stderr, "Failed to create output directory %s: %s\n", (dir / x).string().c_str(), ec.message().c_str());
				return false;
			}
		}
		return true;
	}

	XMVECTOR L3(const XMFLOAT3& v) { return XMLoadFloat3(&v); }
	XMFLOAT3 S3(const XMVECTOR& v) { XMFLOAT3 r; XMStoreFloat3(&r, v); return r; }

	// Squared distance from point P to triangle ABC (Ericson, Real-Time Collision Detection 5.1.5)
	float DistanceSqPointTriangle(const XMVECTOR& P, const XMVECTOR& A, const XMVECTOR& B, const XMVECTOR& C)
	{
		auto dot = [](const XMVECTOR& x, const XMVECTOR& y) { return XMVectorGetX(XMVector3Dot(x, y)); };
		auto lenSq = [](const XMVECTOR& x) { return XMVectorGetX(XMVector3LengthSq(x)); };
		const XMVECTOR AB = B - A;
		const XMVECTOR AC = C - A;
		const XMVECTOR AP = P - A;
		const float d1 = dot(AB, AP);
		const float d2 = dot(AC, AP);
		if (d1 <= 0 && d2 <= 0)
			return lenSq(AP);
		const XMVECTOR BP = P - B;
		const float d3 = dot(AB, BP);
		const float d4 = dot(AC, BP);
		if (d3 >= 0 && d4 <= d3)
			return lenSq(BP);
		const float vc = d1 * d4 - d3 * d2;
		if (vc <= 0 && d1 >= 0 && d3 <= 0)
			return lenSq(P - (A + AB * (d1 / (d1 - d3))));
		const XMVECTOR CP = P - C;
		const float d5 = dot(AB, CP);
		const float d6 = dot(AC, CP);
		if (d6 >= 0 && d5 <= d6)
			return lenSq(CP);
		const float vb = d5 * d2 - d1 * d6;
		if (vb <= 0 && d2 >= 0 && d6 <= 0)
			return lenSq(P - (A + AC * (d2 / (d2 - d6))));
		const float va = d3 * d6 - d5 * d4;
		if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
			return lenSq(P - (B + (C - B) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))));
		const float denom = 1.0f / (va + vb + vc);
		return lenSq(P - (A + AB * (vb * denom) + AC * (vc * denom)));
	}

	// Two sided ray-triangle intersection (Moller-Trumbore)
	bool RayTriangle(const XMVECTOR& O, const XMVECTOR& D, const XMVECTOR& A, const XMVECTOR& B, const XMVECTOR& C, float& t)
	{
		const XMVECTOR E1 = B - A;
		const XMVECTOR E2 = C - A;
		const XMVECTOR Pv = XMVector3Cross(D, E2);
		const float det = XMVectorGetX(XMVector3Dot(E1, Pv));
		if (det == 0)
			return false;
		const float invDet = 1.0f / det;
		const XMVECTOR Tv = O - A;
		const float u = XMVectorGetX(XMVector3Dot(Tv, Pv)) * invDet;
		if (u < 0 || u > 1)
			return false;
		const XMVECTOR Qv = XMVector3Cross(Tv, E1);
		const float v = XMVectorGetX(XMVector3Dot(D, Qv)) * invDet;
		if (v < 0 || u + v > 1)
			return false;
		t = XMVectorGetX(XMVector3Dot(E2, Qv)) * invDet;
		return t > 0;
	}

	// World space triangles of all meshes in the scene with a BVH, used to keep the camera away from the geometry
	class CollisionWorld
	{
	public:
		struct Triangle
		{
			XMFLOAT3 p0, p1, p2;
			uint32_t objectIndex;
			XMFLOAT3 normal = {}; // unit normal on the front side (from the vertex normals), zero if unknown or double sided
		};
		struct Node
		{
			wi::primitive::AABB aabb;
			uint32_t left = 0; // children: left, left + 1
			uint32_t offset = 0;
			uint32_t count = 0; // > 0 for leaves
			bool isLeaf() const { return count > 0; }
		};
		wi::vector<Triangle> triangles;
		wi::vector<Node> nodes;
		wi::vector<uint32_t> leafIndices;
		wi::primitive::AABB bounds;
		struct Range { uint32_t first = 0, last = 0; };
		wi::vector<Range> objectTriangles; // triangles of each object (by object index)
		wi::vector<float> areaCdf; // cumulative triangle area within each object range

		// Uniform random point on the surface of an object, with the front normal of its triangle (zero if unknown)
		bool SampleSurface(int objectIndex, std::mt19937& rng, XMFLOAT3& point, XMFLOAT3& normal) const
		{
			if (objectIndex < 0 || objectIndex >= (int)objectTriangles.size())
				return false;
			const Range range = objectTriangles[objectIndex];
			if (range.first >= range.last)
				return false;
			std::uniform_real_distribution<float> uniform(0, 1);
			const float x = uniform(rng) * areaCdf[range.last - 1];
			const uint32_t index = std::min(range.last - 1,
				(uint32_t)(std::upper_bound(areaCdf.begin() + range.first, areaCdf.begin() + range.last, x) - areaCdf.begin()));
			const Triangle& t = triangles[index];
			const float su = std::sqrt(uniform(rng));
			const float b1 = su * (1 - uniform(rng));
			const float b2 = su - b1;
			point = S3(L3(t.p0) * (1 - b1 - b2) + L3(t.p1) * b1 + L3(t.p2) * b2);
			normal = t.normal;
			return true;
		}

		void Build(const Scene& scene)
		{
			triangles.clear();
			areaCdf.clear();
			bounds = {};
			const size_t objectCount = std::min(scene.objects.GetCount(), scene.matrix_objects.size());
			objectTriangles.assign(objectCount, {});
			for (size_t objectIndex = 0; objectIndex < objectCount; ++objectIndex)
			{
				const uint32_t firstTriangle = (uint32_t)triangles.size();
				const ObjectComponent& object = scene.objects[objectIndex];
				if (object.meshID == INVALID_ENTITY)
					continue;
				const MeshComponent* mesh = scene.meshes.GetComponent(object.meshID);
				if (mesh == nullptr || mesh->indices.empty() || mesh->vertex_positions.empty())
					continue;
				const XMMATRIX objectMat = XMLoadFloat4x4(&scene.matrix_objects[objectIndex]);
				const ArmatureComponent* armature = mesh->IsSkinned() ? scene.armatures.GetComponent(mesh->armatureID) : nullptr;
				if (armature != nullptr && armature->boneData.empty())
					armature = nullptr;

				wi::vector<XMFLOAT3> positions(mesh->vertex_positions.size());
				for (size_t i = 0; i < positions.size(); ++i)
				{
					const XMVECTOR p = armature != nullptr ? SkinVertex(*mesh, *armature, (uint32_t)i) : XMLoadFloat3(&mesh->vertex_positions[i]);
					XMStoreFloat3(&positions[i], XMVector3Transform(p, objectMat));
				}

				uint32_t firstSubset = 0;
				uint32_t lastSubset = 0;
				mesh->GetLODSubsetRange(0, firstSubset, lastSubset);
				const bool hasNormals = mesh->vertex_normals.size() == mesh->vertex_positions.size();
				const float mirror = XMVectorGetX(XMMatrixDeterminant(objectMat)) < 0 ? -1.0f : 1.0f;
				for (uint32_t subsetIndex = firstSubset; subsetIndex < lastSubset; ++subsetIndex)
				{
					const MeshComponent::MeshSubset& subset = mesh->subsets[subsetIndex];
					const MaterialComponent* material = scene.materials.GetComponent(subset.materialID);
					const bool doubleSided = mesh->IsDoubleSided() || subset.IsDoubleSided() || (material != nullptr && material->IsDoubleSided());
					for (uint32_t i = 0; i + 2 < subset.indexCount; i += 3)
					{
						const size_t base = (size_t)subset.indexOffset + i;
						if (base + 2 >= mesh->indices.size())
							break;
						const uint32_t i0 = mesh->indices[base + 0];
						const uint32_t i1 = mesh->indices[base + 1];
						const uint32_t i2 = mesh->indices[base + 2];
						if (i0 >= positions.size() || i1 >= positions.size() || i2 >= positions.size())
							continue;
						Triangle triangle = { positions[i0], positions[i1], positions[i2], (uint32_t)objectIndex };
						const XMVECTOR geometricNormal = XMVector3Cross(L3(triangle.p1) - L3(triangle.p0), L3(triangle.p2) - L3(triangle.p0));
						const float area = XMVectorGetX(XMVector3LengthSq(geometricNormal));
						if (!(area > 0) || !std::isfinite(area))
							continue; // degenerate
						if (hasNormals && !doubleSided)
						{
							// Front side from the rest pose vertex normals, the world winding flips with mirroring transforms:
							const XMVECTOR r0 = XMLoadFloat3(&mesh->vertex_positions[i0]);
							const XMVECTOR restNormal = XMVector3Cross(XMLoadFloat3(&mesh->vertex_positions[i1]) - r0, XMLoadFloat3(&mesh->vertex_positions[i2]) - r0);
							const XMVECTOR shadingNormal = XMLoadFloat3(&mesh->vertex_normals[i0]) + XMLoadFloat3(&mesh->vertex_normals[i1]) + XMLoadFloat3(&mesh->vertex_normals[i2]);
							const float side = XMVectorGetX(XMVector3Dot(restNormal, shadingNormal));
							if (side != 0 && std::isfinite(side))
							{
								triangle.normal = S3(XMVector3Normalize(geometricNormal) * ((side > 0 ? 1.0f : -1.0f) * mirror));
							}
						}
						areaCdf.push_back((triangles.size() > firstTriangle ? areaCdf.back() : 0.0f) + std::sqrt(area) * 0.5f);
						triangles.push_back(triangle);
					}
				}
				objectTriangles[objectIndex] = { firstTriangle, (uint32_t)triangles.size() };
			}

			wi::vector<wi::primitive::AABB> aabbs(triangles.size());
			for (size_t i = 0; i < triangles.size(); ++i)
			{
				const Triangle& t = triangles[i];
				aabbs[i] = wi::primitive::AABB(
					S3(XMVectorMin(L3(t.p0), XMVectorMin(L3(t.p1), L3(t.p2)))),
					S3(XMVectorMax(L3(t.p0), XMVectorMax(L3(t.p1), L3(t.p2))))
				);
				bounds = wi::primitive::AABB::Merge(bounds, aabbs[i]);
			}
			BuildBVH(aabbs);
		}

		// True if no geometry is closer than radius to p
		bool IsClear(const XMFLOAT3& p, float radius) const
		{
			const wi::primitive::Sphere sphere(p, radius);
			const XMVECTOR P = L3(p);
			const float radiusSq = radius * radius;
			bool clear = true;
			Traverse(sphere, [&](uint32_t index) {
				const Triangle& t = triangles[index];
				if (DistanceSqPointTriangle(P, L3(t.p0), L3(t.p1), L3(t.p2)) < radiusSq)
				{
					clear = false;
					return true;
				}
				return false;
			});
			return clear;
		}

		// Returns the object index of the closest hit before maxDistance (ignoring ignoreObject), or -1.
		//	backFace is set when the hit triangle is seen from behind (the ray starts inside a closed object).
		int Raycast(const XMFLOAT3& origin, const XMFLOAT3& direction, float maxDistance, int ignoreObject = -1, float* hitDistance = nullptr, bool* backFace = nullptr) const
		{
			if (hitDistance != nullptr)
				*hitDistance = maxDistance;
			if (backFace != nullptr)
				*backFace = false;
			if (nodes.empty())
				return -1;
			const XMVECTOR O = L3(origin);
			const XMVECTOR D = L3(direction);
			const XMVECTOR invD = XMVectorReciprocal(D);
			float tMax = maxDistance;
			constexpr float miss = std::numeric_limits<float>::infinity();

			// Slab test, returns the entry distance or miss:
			auto Entry = [&](const wi::primitive::AABB& box) {
				const XMVECTOR t0 = (L3(box._min) - O) * invD;
				const XMVECTOR t1 = (L3(box._max) - O) * invD;
				const XMFLOAT3 lo = S3(XMVectorMin(t0, t1));
				const XMFLOAT3 hi = S3(XMVectorMax(t0, t1));
				const float tEnter = std::max({ lo.x, lo.y, lo.z, 0.0f });
				const float tExit = std::min({ hi.x, hi.y, hi.z, tMax });
				return tEnter <= tExit ? tEnter : miss;
			};

			// Front to back traversal, so that the closest hit shrinks tMax early:
			int hit = -1;
			const Triangle* hitTriangle = nullptr;
			std::vector<std::pair<uint32_t, float>> stack;
			stack.reserve(64);
			const float rootEntry = Entry(nodes[0].aabb);
			if (rootEntry != miss)
				stack.emplace_back(0, rootEntry);
			while (!stack.empty())
			{
				const auto [nodeIndex, entry] = stack.back();
				stack.pop_back();
				if (entry > tMax)
					continue;
				const Node& node = nodes[nodeIndex];
				if (node.isLeaf())
				{
					for (uint32_t i = 0; i < node.count; ++i)
					{
						const Triangle& t = triangles[leafIndices[node.offset + i]];
						if ((int)t.objectIndex == ignoreObject)
							continue;
						float distance = 0;
						if (RayTriangle(O, D, L3(t.p0), L3(t.p1), L3(t.p2), distance) && distance < tMax)
						{
							tMax = distance;
							hit = (int)t.objectIndex;
							hitTriangle = &t;
						}
					}
				}
				else
				{
					const uint32_t a = node.left;
					const uint32_t b = node.left + 1;
					const float ta = Entry(nodes[a].aabb);
					const float tb = Entry(nodes[b].aabb);
					const bool aFirst = ta <= tb;
					const uint32_t nearNode = aFirst ? a : b;
					const uint32_t farNode = aFirst ? b : a;
					const float nearEntry = aFirst ? ta : tb;
					const float farEntry = aFirst ? tb : ta;
					if (farEntry != miss)
						stack.emplace_back(farNode, farEntry);
					if (nearEntry != miss)
						stack.emplace_back(nearNode, nearEntry);
				}
			}
			if (hitDistance != nullptr)
				*hitDistance = tMax;
			if (backFace != nullptr && hitTriangle != nullptr)
				*backFace = XMVectorGetX(XMVector3Dot(D, L3(hitTriangle->normal))) > 0;
			return hit;
		}

	private:
		// Splits at the middle of the centroid bounds (median split as a fallback), so leaves stay small even on clustered geometry
		void BuildBVH(const wi::vector<wi::primitive::AABB>& aabbs)
		{
			constexpr uint32_t maxLeafSize = 4;
			const uint32_t count = (uint32_t)aabbs.size();
			nodes.clear();
			leafIndices.resize(count);
			if (count == 0)
				return;
			std::vector<XMFLOAT3> centers(count);
			for (uint32_t i = 0; i < count; ++i)
			{
				leafIndices[i] = i;
				centers[i] = aabbs[i].getCenter();
			}
			nodes.reserve(size_t(count) * 2);
			nodes.emplace_back();
			nodes[0].count = count;
			std::vector<uint32_t> todo = { 0 };
			while (!todo.empty())
			{
				const uint32_t nodeIndex = todo.back();
				todo.pop_back();
				const uint32_t offset = nodes[nodeIndex].offset;
				const uint32_t nodeCount = nodes[nodeIndex].count;
				wi::primitive::AABB box;
				XMVECTOR cmin = XMVectorReplicate(std::numeric_limits<float>::max());
				XMVECTOR cmax = -cmin;
				for (uint32_t i = offset; i < offset + nodeCount; ++i)
				{
					box = wi::primitive::AABB::Merge(box, aabbs[leafIndices[i]]);
					cmin = XMVectorMin(cmin, L3(centers[leafIndices[i]]));
					cmax = XMVectorMax(cmax, L3(centers[leafIndices[i]]));
				}
				nodes[nodeIndex].aabb = box;
				if (nodeCount <= maxLeafSize)
					continue;
				const XMFLOAT3 extent = S3(cmax - cmin);
				int axis = 0;
				if (extent.y > extent.x) axis = 1;
				if (extent.z > (&extent.x)[axis]) axis = 2;
				if (!((&extent.x)[axis] > 0))
					continue; // all centers are identical, keep a leaf
				const XMFLOAT3 lower = S3(cmin);
				const float splitPos = (&lower.x)[axis] + (&extent.x)[axis] * 0.5f;
				auto first = leafIndices.begin() + offset;
				auto last = first + nodeCount;
				auto key = [&](uint32_t index) { return (&centers[index].x)[axis]; };
				uint32_t leftCount = (uint32_t)(std::partition(first, last, [&](uint32_t index) { return key(index) < splitPos; }) - first);
				if (leftCount == 0 || leftCount == nodeCount)
				{
					leftCount = nodeCount / 2;
					std::nth_element(first, first + leftCount, last, [&](uint32_t a, uint32_t b) { return key(a) < key(b); });
				}
				const uint32_t left = (uint32_t)nodes.size();
				nodes.emplace_back();
				nodes.emplace_back();
				nodes[left].offset = offset;
				nodes[left].count = leftCount;
				nodes[left + 1].offset = offset + leftCount;
				nodes[left + 1].count = nodeCount - leftCount;
				nodes[nodeIndex].left = left;
				nodes[nodeIndex].count = 0;
				todo.push_back(left);
				todo.push_back(left + 1);
			}
		}

		// Callback returns true to stop the traversal
		template <typename T, typename F>
		void Traverse(const T& primitive, F&& callback) const
		{
			if (nodes.empty())
				return;
			std::vector<uint32_t> stack;
			stack.reserve(64);
			stack.push_back(0);
			while (!stack.empty())
			{
				const Node& node = nodes[stack.back()];
				stack.pop_back();
				if (!node.aabb.intersects(primitive))
					continue;
				if (node.isLeaf())
				{
					for (uint32_t i = 0; i < node.count; ++i)
					{
						if (callback(leafIndices[node.offset + i]))
							return;
					}
				}
				else
				{
					stack.push_back(node.left);
					stack.push_back(node.left + 1);
				}
			}
		}
	};

	// Object the camera looks at (views) or the animation is built around
	struct Target
	{
		int objectIndex = -1; // -1: the whole model
		Entity entity = INVALID_ENTITY;
		wi::primitive::AABB aabb;
		XMFLOAT3 center = {};
		float radius = 0;
		std::string name;
	};

	struct CameraPose
	{
		XMFLOAT3 eye;
		XMFLOAT3 forward;
	};

	struct OrbitPose
	{
		float azimuth = 0;
		float elevation = 0;
		float distance = 0;
		CameraPose pose = {};
	};

	// Random camera looking at the target center, at a distance that fits its bounding sphere in view (times --dist-min/max)
	OrbitPose SampleOrbitPose(const Options& o, std::mt19937& rng, const Target& target)
	{
		auto uniform = [&](float a, float b) { return a < b ? std::uniform_real_distribution<float>(a, b)(rng) : a; };

		const float aspect = float(o.width) / float(o.height);
		const float halfFovY = XMConvertToRadians(o.fov) * 0.5f;
		const float halfFovX = std::atan(std::tan(halfFovY) * aspect);
		const float fitDistance = target.radius / std::sin(std::min(halfFovX, halfFovY));
		float elevMin = 0;
		float elevMax = 0;
		ElevationRange(o, elevMin, elevMax);

		OrbitPose p;
		p.azimuth = uniform(0, XM_2PI);
		p.elevation = uniform(elevMin, elevMax);
		p.distance = fitDistance * uniform(o.distMin, o.distMax);
		const XMVECTOR dir = L3(SphericalDirection(p.azimuth, p.elevation));
		p.pose.eye = S3(L3(target.center) + dir * p.distance);
		p.pose.forward = S3(-dir);
		return p;
	}

	// Moves the camera towards an object target along the view axis, so that no other object is in between
	//	(e.g. walls in front of an object inside a building). Returns false if there is no room for the camera.
	bool PlaceInFrontOfOccluders(const CollisionWorld& world, const Target& target, float clearance, OrbitPose& start)
	{
		if (target.objectIndex < 0)
			return true;
		const XMVECTOR C = L3(target.center);
		const XMFLOAT3 dir = S3(-L3(start.pose.forward)); // from the target to the camera
		float hitDistance = start.distance;
		const int hit = world.Raycast(target.center, dir, start.distance, target.objectIndex, &hitDistance);
		if (hit < 0)
			return true;
		const float distance = hitDistance - 2 * clearance;
		if (distance < 2 * clearance)
			return false;
		start.distance = distance;
		start.pose.eye = S3(C + L3(dir) * distance);
		return true;
	}

	struct PathSegment
	{
		enum class Type { Orbit, Translate } type = Type::Orbit;

		// Orbit: spherical coordinates around a pivot, linearly interpolated
		XMFLOAT3 pivot = {};
		float distance0 = 0, distance1 = 0;
		float azimuth0 = 0, azimuth1 = 0;
		float elevation0 = 0, elevation1 = 0;

		// Translate: fixed orientation
		XMFLOAT3 from = {}, to = {};
		XMFLOAT3 forward = {};
		std::string direction;

		float length = 0;

		CameraPose Eval(float t) const
		{
			CameraPose pose;
			if (type == Type::Orbit)
			{
				const float azimuth = azimuth0 + (azimuth1 - azimuth0) * t;
				const float elevation = elevation0 + (elevation1 - elevation0) * t;
				const float distance = distance0 + (distance1 - distance0) * t;
				const XMVECTOR dir = L3(SphericalDirection(azimuth, elevation));
				pose.eye = S3(L3(pivot) + dir * distance);
				pose.forward = S3(-dir);
			}
			else
			{
				pose.eye = S3(XMVectorLerp(L3(from), L3(to), t));
				pose.forward = forward;
			}
			return pose;
		}

		nlohmann::json ToJsonDesc() const
		{
			if (type == Type::Orbit)
			{
				return {
					{ "type", "orbit" },
					{ "pivot", ToJson(pivot) },
					{ "distance", { distance0, distance1 } },
					{ "azimuth", { azimuth0, azimuth1 } },
					{ "elevation", { elevation0, elevation1 } },
				};
			}
			return {
				{ "type", "translate" },
				{ "direction", direction },
				{ "from", ToJson(from) },
				{ "to", ToJson(to) },
				{ "forward", ToJson(forward) },
			};
		}
	};

	// Chain of segments parameterized by normalized arc length s in [0, 1]. Only the [s0, s1] part is rendered.
	struct CameraPath
	{
		std::vector<PathSegment> segments;
		float length = 0;
		float s0 = 0;
		float s1 = 1;

		void Finalize()
		{
			length = 0;
			for (PathSegment& segment : segments)
			{
				segment.length = 0;
				const int steps = segment.type == PathSegment::Type::Orbit ? 64 : 1;
				XMVECTOR prev = L3(segment.Eval(0).eye);
				for (int i = 1; i <= steps; ++i)
				{
					const XMVECTOR cur = L3(segment.Eval(float(i) / steps).eye);
					segment.length += XMVectorGetX(XMVector3Length(cur - prev));
					prev = cur;
				}
				length += segment.length;
			}
		}

		CameraPose Eval(float s) const
		{
			s = std::clamp(s, 0.0f, 1.0f);
			if (!(length > 0))
			{
				const float x = s * segments.size();
				const size_t index = std::min(segments.size() - 1, (size_t)x);
				return segments[index].Eval(std::clamp(x - index, 0.0f, 1.0f));
			}
			float distance = s * length;
			for (size_t i = 0; i < segments.size(); ++i)
			{
				const PathSegment& segment = segments[i];
				if (distance <= segment.length || i == segments.size() - 1)
				{
					return segment.Eval(segment.length > 0 ? std::clamp(distance / segment.length, 0.0f, 1.0f) : 1.0f);
				}
				distance -= segment.length;
			}
			return segments.back().Eval(1);
		}

		CameraPose EvalFrame(int frame, int frameCount) const
		{
			const float t = frameCount > 1 ? float(frame) / float(frameCount - 1) : 0.0f;
			return Eval(s0 + (s1 - s0) * t);
		}
	};

	CameraPath GeneratePath(const Options& o, std::mt19937& rng, const std::string& type, const Target& target, const OrbitPose& start)
	{
		auto uniform = [&](float a, float b) { return a < b ? std::uniform_real_distribution<float>(a, b)(rng) : a; };
		auto uniformInt = [&](int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); };

		float elevMin = 0;
		float elevMax = 0;
		ElevationRange(o, elevMin, elevMax);

		// Start pose, framing the target like in the independent view mode:
		const float startAzimuth = start.azimuth;
		const float startElevation = start.elevation;
		const float startDistance = start.distance;
		const XMVECTOR C = L3(target.center);
		const XMVECTOR up = XMVectorSet(0, 1, 0, 0);
		XMVECTOR eye = L3(start.pose.eye);
		XMVECTOR forward = L3(start.pose.forward);

		std::vector<PathSegment::Type> types;
		if (type == "orbit")
		{
			types.push_back(PathSegment::Type::Orbit);
		}
		else if (type == "translate")
		{
			types.push_back(PathSegment::Type::Translate);
		}
		else
		{
			const int count = uniformInt(2, 3);
			for (int i = 0; i < count; ++i)
			{
				types.push_back(uniformInt(0, 1) == 0 ? PathSegment::Type::Orbit : PathSegment::Type::Translate);
			}
			if (std::all_of(types.begin(), types.end(), [&](PathSegment::Type t) { return t == types.front(); }))
			{
				types.back() = types.front() == PathSegment::Type::Orbit ? PathSegment::Type::Translate : PathSegment::Type::Orbit;
			}
		}
		const bool mix = types.size() > 1;
		const float scale = mix ? 1.5f / float(types.size()) : 1.0f; // keep the total motion of mixed paths comparable

		CameraPath path;
		for (size_t i = 0; i < types.size(); ++i)
		{
			PathSegment segment;
			segment.type = types[i];
			if (segment.type == PathSegment::Type::Orbit)
			{
				if (i == 0)
				{
					segment.pivot = target.center;
					segment.azimuth0 = startAzimuth;
					segment.elevation0 = startElevation;
					segment.distance0 = startDistance;
				}
				else
				{
					// Continue from the current pose: orbit around the point at the target depth on the view axis
					const float depth = std::max(XMVectorGetX(XMVector3Dot(C - eye, forward)), target.radius);
					const XMFLOAT3 back = S3(-forward);
					segment.pivot = S3(eye + forward * depth);
					segment.azimuth0 = std::atan2(back.z, back.x);
					segment.elevation0 = std::asin(std::clamp(back.y, -1.0f, 1.0f));
					segment.distance0 = depth;
				}
				const float sweep = XMConvertToRadians(uniform(o.orbitMin, o.orbitMax)) * scale * (uniformInt(0, 1) == 0 ? 1.0f : -1.0f);
				segment.azimuth1 = segment.azimuth0 + sweep;
				segment.elevation1 = std::clamp(segment.elevation0 + uniform(-0.3f, 0.3f) * std::abs(sweep),
					std::min(segment.elevation0, elevMin), std::max(segment.elevation0, elevMax));
				segment.distance1 = mix ? segment.distance0 * uniform(0.8f, 1.25f) : segment.distance0;
			}
			else
			{
				static const char* names[] = { "forward", "backward", "left", "right" };
				const XMVECTOR right = XMVector3Normalize(XMVector3Cross(up, forward));
				const int direction = uniformInt(0, 3);
				const XMVECTOR motion = direction == 0 ? forward : direction == 1 ? -forward : direction == 2 ? -right : right;
				const float length = XMVectorGetX(XMVector3Length(C - eye)) * uniform(o.moveMin, o.moveMax) * scale;
				XMVECTOR from = eye;
				if (i == 0 && direction >= 2)
				{
					from = eye - motion * (length * 0.5f); // lateral motion centered on the framed target
				}
				segment.from = S3(from);
				segment.to = S3(from + motion * length);
				segment.forward = S3(forward);
				segment.direction = names[direction];
			}
			const CameraPose end = segment.Eval(1);
			eye = L3(end.eye);
			forward = L3(end.forward);
			path.segments.push_back(segment);
		}
		path.Finalize();
		return path;
	}

	// Casts a grid of primary rays through the image and returns the fraction of them whose first hit is the target
	//	(any object but excludeObject when the target is the whole model). Normalized image positions of these hits go to targetPixels.
	//	backFaceFraction is the fraction of all hits that see a surface from behind (high when the camera is inside an object).
	float TargetCoverage(const CollisionWorld& world, const Options& o, const CameraPose& pose, const Target& target, int excludeObject,
		std::vector<XMFLOAT2>& targetPixels, float& backFaceFraction)
	{
		constexpr int grid = 32;
		const float tanY = std::tan(XMConvertToRadians(o.fov) * 0.5f);
		const float tanX = tanY * float(o.width) / float(o.height);
		const XMVECTOR forward = XMVector3Normalize(L3(pose.forward));
		const XMVECTOR right = XMVector3Normalize(XMVector3Cross(XMVectorSet(0, 1, 0, 0), forward)); // left handed, like the view matrix
		const XMVECTOR up = XMVector3Cross(forward, right);
		const float maxDistance = XMVectorGetX(XMVector3Length(L3(pose.eye) - L3(world.bounds.getCenter()))) + world.bounds.getRadius() * 2 + 1;

		targetPixels.clear();
		int hits = 0;
		int backFaces = 0;
		for (int y = 0; y < grid; ++y)
		{
			for (int x = 0; x < grid; ++x)
			{
				const float u = (x + 0.5f) / grid;
				const float v = (y + 0.5f) / grid;
				const XMVECTOR dir = XMVector3Normalize(forward + right * ((2 * u - 1) * tanX) + up * ((1 - 2 * v) * tanY));
				bool backFace = false;
				const int hit = world.Raycast(pose.eye, S3(dir), maxDistance, -1, nullptr, &backFace);
				if (hit >= 0)
				{
					hits++;
					backFaces += backFace ? 1 : 0;
				}
				const bool onTarget = target.objectIndex < 0 ? (hit >= 0 && hit != excludeObject) : hit == target.objectIndex;
				if (onTarget)
				{
					targetPixels.push_back(XMFLOAT2(u, v));
				}
			}
		}
		backFaceFraction = hits > 0 ? float(backFaces) / float(hits) : 0.0f;
		return float(targetPixels.size()) / float(grid * grid);
	}

	// Checks that the camera stays at least `clearance` away from the geometry along the path.
	//	On collision, the path is cut to its longest valid part if it is long enough (sets s0/s1 and split=true).
	bool ValidatePath(CameraPath& path, const CollisionWorld& world, float clearance, float minSplit, bool& split)
	{
		// Sample spacing well below the clearance, so the camera can't cross geometry between two valid samples:
		const float step = clearance * 0.25f;
		const int samples = (int)std::clamp(std::ceil(path.length / step) + 1.0f, 2.0f, 200000.0f);
		std::vector<uint8_t> clear(samples);
		for (int i = 0; i < samples; ++i)
		{
			clear[i] = world.IsClear(path.Eval(float(i) / float(samples - 1)).eye, clearance) ? 1 : 0;
		}

		struct Run { int first; int last; };
		std::vector<Run> runs;
		for (int i = 0; i < samples; ++i)
		{
			if (!clear[i])
				continue;
			if (!runs.empty() && runs.back().last == i - 1)
				runs.back().last = i;
			else
				runs.push_back({ i, i });
		}
		std::sort(runs.begin(), runs.end(), [](const Run& a, const Run& b) { return (a.last - a.first) > (b.last - b.first); });

		if (!runs.empty() && float(runs[0].last - runs[0].first) / float(samples - 1) >= minSplit - 1e-6f)
		{
			path.s0 = float(runs[0].first) / float(samples - 1);
			path.s1 = float(runs[0].last) / float(samples - 1);
			split = runs[0].first != 0 || runs[0].last != samples - 1;
			return true;
		}
		return false;
	}
}

int main(int argc, char* argv[])
{
	Options opt;
	const int parse = ParseArgs(argc, argv, opt);
	if (parse != 0)
		return parse < 0 ? 0 : 1;

	const bool animationMode = !opt.animate.empty();
	const std::filesystem::path outDir = std::filesystem::absolute(opt.outDir);
	if (animationMode)
	{
		std::error_code ec;
		std::filesystem::create_directories(outDir, ec);
		if (ec)
		{
			std::fprintf(stderr, "Failed to create output directory %s: %s\n", outDir.string().c_str(), ec.message().c_str());
			return 1;
		}
	}
	else if (!CreateOutputDirs(outDir, opt, true))
	{
		return 1;
	}

	// Window (only used as a preview and to host the swapchain):
	const float previewScale = std::min(1.0f, 768.0f / float(std::max(opt.width, opt.height)));
	sdl2::sdlsystem_ptr_t system = sdl2::make_sdlsystem(SDL_INIT_EVERYTHING | SDL_INIT_EVENTS);
	sdl2::window_ptr_t window = sdl2::make_window(
		"DatasetRenderer",
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		std::max(1, int(opt.width * previewScale)), std::max(1, int(opt.height * previewScale)),
		(opt.hidden ? SDL_WINDOW_HIDDEN : SDL_WINDOW_SHOWN) | SDL_WINDOW_VULKAN | SDL_WINDOW_ALLOW_HIGHDPI);
	if (!window)
	{
		std::fprintf(stderr, "Failed to create window: %s\n", SDL_GetError());
		return 1;
	}

	if (!opt.shaderDir.empty())
	{
		std::error_code ec;
		std::filesystem::create_directories(opt.shaderDir, ec);
		wi::renderer::SetShaderPath(opt.shaderDir); // must be set before SetWindow() which appends the backend subfolder
	}

	struct DatasetApplication : wi::Application
	{
		DatasetApplication() { alwaysactive = true; } // keep rendering when the window is unfocused/minimized
	};
	static DatasetApplication application;
	application.allow_hdr = false; // SDR swapchain -> tonemapped sRGB output in both render paths
	application.swapChain.desc.vsync = false;
	application.SetRenderResolution((uint32_t)opt.width, (uint32_t)opt.height);
	{
		// The engine picks the adapter in SetWindow() from its command line flags (igpu, nvidiagpu, amdgpu, intelgpu):
		std::string flag;
		if (opt.gpu == "integrated") flag = "igpu";
		else if (opt.gpu != "discrete") flag = opt.gpu + "gpu";
		char* engineArgs[] = { argv[0], flag.data() };
		wi::arguments::Parse(flag.empty() ? 1 : 2, engineArgs);
	}
	application.SetWindow(window.get());
	{
		const std::string adapter = wi::graphics::GetDevice()->GetAdapterName();
		std::printf("GPU: %s\n", adapter.c_str());
		if (opt.gpu != "discrete" && opt.gpu != "integrated" && wi::helper::toUpper(adapter).find(wi::helper::toUpper(opt.gpu)) == std::string::npos)
		{
			std::fprintf(stderr, "Error: no %s GPU found (the engine fell back to %s)\n", opt.gpu.c_str(), adapter.c_str());
			return 1;
		}
	}
	application.Initialize();
	application.infoDisplay.active = false;

	auto Shutdown = [&](int code) {
		application.ActivatePath(nullptr);
		wi::jobsystem::ShutDown();
		return code;
	};

	bool quit = false;
	auto RunFrame = [&]() {
		SDL_PumpEvents();
		application.Run();
		SDL_Event event;
		while (SDL_PollEvent(&event))
		{
			if (event.type == SDL_QUIT || (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE))
			{
				quit = true;
			}
			wi::input::sdlinput::ProcessEvent(event);
		}
		return !quit;
	};

	std::printf("Initializing engine (the first run compiles shaders, this can take a while)...\n");
	std::fflush(stdout);
	while (!wi::initializer::IsInitializeFinished())
	{
		if (!RunFrame())
			return Shutdown(1);
	}

	wi::physics::SetSimulationEnabled(false);

	Scene& scene = wi::scene::GetScene();

	// Load model:
	{
		std::printf("Loading %s\n", opt.input.c_str());
		std::fflush(stdout);
		const std::string ext = wi::helper::toUpper(wi::helper::GetExtensionFromFileName(opt.input));
		auto loaded = std::make_unique<Scene>();
		if (ext == "GLTF" || ext == "GLB" || ext == "VRM")
		{
			ImportModel_GLTF(opt.input, *loaded);
		}
		else if (ext == "FBX")
		{
			ImportModel_FBX(opt.input, *loaded);
		}
		else if (ext == "OBJ")
		{
			ImportModel_OBJ(opt.input, *loaded);
		}
		else if (ext == "WISCENE")
		{
			wi::scene::LoadModel(*loaded, opt.input);
		}
		else
		{
			std::fprintf(stderr, "Unsupported model format: %s (export .blend files to glTF/GLB from Blender)\n", ext.c_str());
			return Shutdown(1);
		}
		scene.Merge(*loaded);
	}
	if (scene.objects.GetCount() == 0)
	{
		std::fprintf(stderr, "The model doesn't contain any renderable object\n");
		return Shutdown(1);
	}

	// Freeze animations so that the path tracer can accumulate:
	for (size_t i = 0; i < scene.animations.GetCount(); ++i)
	{
		scene.animations[i].Pause();
	}

	// Lighting from the file:
	if (!opt.keepSceneLighting)
	{
		while (scene.lights.GetCount() > 0)
			scene.lights.Remove(scene.lights.GetEntity(0));
		while (scene.weathers.GetCount() > 0)
			scene.weathers.Remove(scene.weathers.GetEntity(0));
		while (scene.probes.GetCount() > 0)
			scene.probes.Remove(scene.probes.GetEntity(0)); // the path tracer doesn't use probes, the rasterizer will use the sky instead
	}

	if (scene.weathers.GetCount() == 0)
	{
		Entity entity = CreateEntity();
		scene.names.Create(entity) = "dataset_weather";
		WeatherComponent& weather = scene.weathers.Create(entity);
		weather.stars = 0;
		weather.fogDensity = 0;
		weather.skyExposure = opt.skyIntensity;
		if (opt.sky == "gradient")
		{
			weather.horizon = XMFLOAT3(0.65f, 0.75f, 0.9f);
			weather.zenith = XMFLOAT3(0.25f, 0.45f, 0.85f);
		}
		else if (opt.sky == "realistic")
		{
			weather.SetRealisticSky(true);
		}
		else
		{
			weather.horizon = XMFLOAT3(0, 0, 0);
			weather.zenith = XMFLOAT3(0, 0, 0);
		}
		if (!opt.hdri.empty())
		{
			weather.skyMapName = opt.hdri;
			weather.skyMap = wi::resourcemanager::Load(opt.hdri);
			if (!weather.skyMap.IsValid())
			{
				std::fprintf(stderr, "Failed to load HDRI: %s\n", opt.hdri.c_str());
				return Shutdown(1);
			}
		}
	}
	// The path tracer ignores the constant ambient term, remove it so that both renderers get the same lighting:
	for (size_t i = 0; i < scene.weathers.GetCount(); ++i)
	{
		scene.weathers[i].ambient = XMFLOAT3(0, 0, 0);
	}

	// Render paths with matching post process settings:
	DatasetRasterizer raster;
	DatasetPathTracer pathtracer;
	{
		wi::renderer::Tonemap tonemap = wi::renderer::Tonemap::ACES;
		if (opt.tonemap == "reinhard") tonemap = wi::renderer::Tonemap::Reinhard;
		if (opt.tonemap == "uchimura") tonemap = wi::renderer::Tonemap::Uchimura;

		wi::RenderPath3D* paths[] = { &raster, &pathtracer };
		for (wi::RenderPath3D* path : paths)
		{
			path->setExposure(opt.exposure);
			path->setTonemap(tonemap);
			path->setBloomEnabled(opt.bloom);
			path->setEyeAdaptionEnabled(false);
			path->setLensFlareEnabled(false);
			path->setLightShaftsEnabled(false);
			path->setVolumeLightsEnabled(false);
			path->setDepthOfFieldEnabled(false);
			path->setMotionBlurEnabled(false);
			path->setDitherEnabled(false);
			path->setColorGradingEnabled(false);
			path->setSharpenFilterEnabled(false);
			path->setChromaticAberrationEnabled(false);
			path->setOutlineEnabled(false);
			path->setFXAAEnabled(false);
		}

		raster.setOcclusionCullingEnabled(false); // GPU occlusion queries lag behind camera cuts
		raster.setSSREnabled(opt.ssr);
		if (opt.ao == "ssao") raster.setAO(wi::RenderPath3D::AO_SSAO);
		else if (opt.ao == "hbao") raster.setAO(wi::RenderPath3D::AO_HBAO);
		else if (opt.ao == "msao") raster.setAO(wi::RenderPath3D::AO_MSAO);
		else raster.setAO(wi::RenderPath3D::AO_DISABLED);

		wi::renderer::SetTemporalAAEnabled(opt.aa == "taa");
		raster.setMSAASampleCount(opt.aa == "msaa" ? 4 : 1);
		raster.setFXAAEnabled(opt.aa == "fxaa");

		wi::renderer::SetDDGIEnabled(opt.gi == "ddgi");
		wi::renderer::SetVXGIEnabled(opt.gi == "vxgi");
		wi::renderer::SetSurfelGIEnabled(opt.gi == "surfel");

		wi::renderer::SetShadowProps2D(opt.shadowRes);
		wi::renderer::SetShadowPropsCube(std::max(128, opt.shadowRes / 2));

		pathtracer.setTargetSampleCount(opt.spp);
		if (opt.bounces >= 0)
		{
			wi::renderer::SetRaytraceBounceCount((uint32_t)opt.bounces);
		}
	}

	const bool denoise = opt.denoise && pathtracer.isDenoiserAvailable();
	if (opt.denoise && !denoise)
	{
		std::fprintf(stderr, "Warning: the engine was built without Open Image Denoise, path traced images will not be denoised\n");
	}

	CameraComponent& camera = wi::scene::GetCamera();
	camera.CreatePerspective((float)opt.width, (float)opt.height, 0.01f, 1000.0f, XMConvertToRadians(opt.fov));

	// Run a few frames so that the scene is updated (bounds, render data, delayed texture imports):
	application.ActivatePath(&raster);
	for (int i = 0; i < 3; ++i)
	{
		if (!RunFrame())
			return Shutdown(1);
	}

	const wi::primitive::AABB bounds = scene.bounds;
	if (!bounds.IsValid())
	{
		std::fprintf(stderr, "Invalid scene bounds\n");
		return Shutdown(1);
	}
	const XMFLOAT3 center = bounds.getCenter();
	const float radius = std::max(1e-4f, bounds.getRadius());
	std::printf("Model bounds: center (%.3f %.3f %.3f), radius %.3f\n", center.x, center.y, center.z, radius);

	Entity groundEntity = INVALID_ENTITY;
	if (opt.ground)
	{
		Entity entity = scene.Entity_CreatePlane("dataset_ground");
		groundEntity = entity;
		TransformComponent* transform = scene.transforms.GetComponent(entity);
		transform->Scale(XMFLOAT3(radius * 20, 1, radius * 20));
		transform->Translate(XMFLOAT3(center.x, bounds._min.y - radius * 0.001f, center.z));
		transform->UpdateTransform();
		MaterialComponent* material = scene.materials.GetComponent(entity);
		material->SetBaseColor(XMFLOAT4(0.5f, 0.5f, 0.5f, 1));
		material->SetRoughness(0.8f);
		material->SetMetalness(0);
	}

	// Tool lights:
	Entity sunEntity = INVALID_ENTITY;
	std::vector<Entity> pointLightEntities;
	if (!opt.keepSceneLighting)
	{
		if (opt.sunIntensity > 0)
		{
			sunEntity = scene.Entity_CreateLight("dataset_sun", XMFLOAT3(0, 0, 0), XMFLOAT3(1, 1, 1), opt.sunIntensity, 1000, LightComponent::DIRECTIONAL);
			scene.lights.GetComponent(sunEntity)->SetCastShadow(true);
		}
		for (int i = 0; i < opt.pointLights; ++i)
		{
			Entity entity = scene.Entity_CreateLight("dataset_point_" + std::to_string(i), XMFLOAT3(0, 0, 0), XMFLOAT3(1, 1, 1), 1, 1, LightComponent::POINT);
			scene.lights.GetComponent(entity)->SetCastShadow(true);
			pointLightEntities.push_back(entity);
		}
	}

	// Dataset level metadata:
	{
		nlohmann::json j;
		j["input"] = opt.input;
		j["resolution"] = { opt.width, opt.height };
		j["seed"] = opt.seed;
		j["views"] = { opt.firstView, opt.firstView + opt.views };
		j["mode"] = animationMode ? "animation" : "views";
		j["start"] = {
			{ "focus", opt.focus },
			{ "min_target_size", opt.minTargetSize },
			{ "clearance", opt.clearance },
			{ "min_coverage", opt.minCoverage },
			{ "min_brightness", opt.minBrightness },
			{ "max_attempts", opt.maxAttempts },
		};
		if (animationMode)
		{
			j["animation"] = {
				{ "type", opt.animate },
				{ "frames", opt.frames },
				{ "orbit_degrees", { opt.orbitMin, opt.orbitMax } },
				{ "move", { opt.moveMin, opt.moveMax } },
				{ "min_split", opt.minSplit },
			};
		}
		j["bounds"] = { { "min", ToJson(bounds._min) }, { "max", ToJson(bounds._max) }, { "center", ToJson(center) }, { "radius", radius } };
		j["pathtracer"] = { { "spp", opt.spp }, { "bounces", (int)wi::renderer::GetRaytraceBounceCount() }, { "denoised", denoise } };
		j["rasterizer"] = { { "frames", opt.rasterFrames }, { "aa", opt.aa }, { "ao", opt.ao }, { "gi", opt.gi }, { "ssr", opt.ssr }, { "shadow_resolution", opt.shadowRes } };
		j["postprocess"] = { { "exposure", opt.exposure }, { "tonemap", opt.tonemap }, { "bloom", opt.bloom } };
		j["lighting"] = {
			{ "keep_scene_lighting", opt.keepSceneLighting },
			{ "sun_intensity", opt.sunIntensity },
			{ "point_lights", opt.pointLights },
			{ "point_intensity", opt.pointIntensity },
			{ "sky", opt.sky },
			{ "hdri", opt.hdri },
			{ "sky_intensity", opt.skyIntensity },
			{ "ground", opt.ground },
		};
		j["conventions"] = "World space is left handed, Y up. Matrices are row-major for row vectors (v * M), projection uses reversed Z. "
			"PNG images are tonemapped sRGB. PFM images are linear radiance before exposure and tonemapping. "
			"sun.direction points from the scene towards the sun. "
			"In animation mode, animation i is in anim_<i>/ with frames <kind>/<frame>.png and cameras in anim_<i>/animation.json "
			"(\"views\" counts animations). motion/<frame>.npy holds float32 motion vectors of shape (height, width, 2): "
			"the offset in pixels (x right, y down) from the pixel center in this frame to the position of the same surface point "
			"in the previous frame (zero for frame 0, NaN where the point is behind the previous camera, the sky moves with the "
			"camera rotation only). They come from the rasterizer depth of opaque surfaces, the scene is static.";
		std::ofstream file(outDir / "dataset.json");
		file << j.dump(2) << std::endl;
	}

	const int frameGuard = opt.spp * 4 + 10000;

	auto SetCamera = [&](const XMFLOAT3& eye, const XMFLOAT3& forward, float zNear, float zFar) {
		camera.jitter = XMFLOAT2(0, 0);
		camera.CreatePerspective((float)opt.width, (float)opt.height, zNear, zFar, XMConvertToRadians(opt.fov));
		camera.Eye = eye;
		camera.At = forward;
		camera.Up = XMFLOAT3(0, 1, 0);
		camera.SetDirty();
		camera.UpdateCamera();

		// Directional shadow cascades end at fixed distances (8, 80, 800 by default) and nothing further is shadowed.
		//	Scale them with the far plane, which encloses the whole scene, keeping the default 10x ratio between cascades:
		for (size_t i = 0; i < scene.lights.GetCount(); ++i)
		{
			LightComponent& light = scene.lights[i];
			if (light.GetType() == LightComponent::DIRECTIONAL)
			{
				light.cascade_distances = { zFar * 0.01f, zFar * 0.1f, zFar };
			}
		}
	};

	auto ApplyLights = [&](const LightSetup& view) {
		if (sunEntity != INVALID_ENTITY)
		{
			SetTransform(scene, sunEntity, XMFLOAT3(0, 0, 0), RotationFromUpTo(view.sunDirection));
		}
		for (size_t i = 0; i < pointLightEntities.size(); ++i)
		{
			const PointLightSetup& pl = view.pointLights[i];
			SetTransform(scene, pointLightEntities[i], pl.position, XMQuaternionIdentity());
			LightComponent* light = scene.lights.GetComponent(pointLightEntities[i]);
			light->color = pl.color;
			light->intensity = pl.intensity;
			light->range = pl.range;
		}
		if (opt.rotateHdri && scene.weathers.GetCount() > 0)
		{
			scene.weathers[0].sky_rotation = view.skyRotation;
		}
	};

	auto LightsToJson = [&](nlohmann::json& j, const LightSetup& view) {
		if (sunEntity != INVALID_ENTITY)
		{
			j["sun"] = {
				{ "direction", ToJson(view.sunDirection) },
				{ "azimuth", view.sunAzimuth },
				{ "elevation", view.sunElevation },
				{ "intensity", opt.sunIntensity },
				{ "color", { 1, 1, 1 } },
			};
		}
		nlohmann::json points = nlohmann::json::array();
		for (size_t i = 0; i < pointLightEntities.size(); ++i)
		{
			const PointLightSetup& pl = view.pointLights[i];
			points.push_back({ { "position", ToJson(pl.position) }, { "color", ToJson(pl.color) }, { "intensity", pl.intensity }, { "range", pl.range } });
		}
		j["point_lights"] = points;
		j["sky_rotation"] = view.skyRotation;
	};

	// Writes the backward motion vectors of the last rasterized frame towards the camera pose `previous` (same projection),
	//	from the rasterizer depth buffer. The depth was rendered with the TAA jitter of that frame, which is accounted for.
	auto SaveMotion = [&](const std::filesystem::path& fileName, const CameraPose& previous) {
		std::vector<float> depth;
		uint32_t w = 0;
		uint32_t h = 0;
		if (!ReadTextureR32(raster.depthBuffer_Copy, depth, w, h) || w != (uint32_t)opt.width || h != (uint32_t)opt.height)
		{
			std::fprintf(stderr, "\nError: could not read back the rasterizer depth buffer\n");
			return false;
		}

		// Unjittered current and previous cameras:
		CameraComponent current = camera;
		current.jitter = XMFLOAT2(0, 0);
		current.UpdateCamera();
		CameraComponent before = current;
		before.Eye = previous.eye;
		before.At = previous.forward;
		before.Up = XMFLOAT3(0, 1, 0);
		before.UpdateCamera();
		const XMMATRIX invVP = XMLoadFloat4x4(&camera.InvVP); // as rendered (jittered)
		const XMMATRIX VP = XMLoadFloat4x4(&current.VP);
		const XMMATRIX prevVP = XMLoadFloat4x4(&before.VP);

		const float nan = std::numeric_limits<float>::quiet_NaN();
		std::vector<float> motion(size_t(w) * h * 2);
		for (uint32_t y = 0; y < h; ++y)
		{
			for (uint32_t x = 0; x < w; ++x)
			{
				const size_t i = size_t(y) * w + x;
				const float ndcX = (x + 0.5f) / w * 2 - 1;
				const float ndcY = 1 - (y + 0.5f) / h * 2;
				XMVECTOR P; // homogeneous world position
				if (depth[i] > 0)
				{
					P = XMVector4Transform(XMVectorSet(ndcX, ndcY, depth[i], 1), invVP);
					P = P / XMVectorSplatW(P);
				}
				else
				{
					// Sky (cleared depth): direction at infinity
					const XMVECTOR nearPoint = XMVector3TransformCoord(XMVectorSet(ndcX, ndcY, 1, 1), invVP);
					const XMVECTOR farPoint = XMVector3TransformCoord(XMVectorSet(ndcX, ndcY, 0.5f, 1), invVP);
					P = XMVectorSetW(XMVector3Normalize(farPoint - nearPoint), 0);
				}
				const XMFLOAT4 c = [&] { XMFLOAT4 r; XMStoreFloat4(&r, XMVector4Transform(P, VP)); return r; }();
				const XMFLOAT4 p = [&] { XMFLOAT4 r; XMStoreFloat4(&r, XMVector4Transform(P, prevVP)); return r; }();
				if (!(p.w > 0) || !(c.w > 0))
				{
					motion[i * 2 + 0] = nan;
					motion[i * 2 + 1] = nan;
					continue;
				}
				motion[i * 2 + 0] = (p.x / p.w - c.x / c.w) * 0.5f * w;
				motion[i * 2 + 1] = (c.y / c.w - p.y / p.w) * 0.5f * h;
			}
		}
		if (!SaveNpy(fileName.string(), motion.data(), { h, w, 2 }))
		{
			std::fprintf(stderr, "\nError: failed to write %s\n", fileName.string().c_str());
			return false;
		}
		return true;
	};

	// Renders the current camera with both renderers and writes dir/<kind>/name.(png|pfm)
	//	With motionFrom, also writes the motion vectors towards that camera pose to dir/motion/name.npy
	//	Returns 0 on success, 1 on error, 2 when the window was closed
	auto RenderPair = [&](const std::filesystem::path& dir, const std::string& name, const std::string& label, const CameraPose* motionFrom = nullptr) -> int {
		std::printf("%s rasterizing...", label.c_str());
		std::fflush(stdout);

		// Rasterizer:
		wi::renderer::SetTemporalAAEnabled(opt.aa == "taa");
		application.ActivatePath(&raster);
		for (int i = 0; i < opt.rasterFrames && !quit; ++i)
		{
			RunFrame();
		}
		if (quit)
			return 2;
		bool ok = wi::helper::saveTextureToFile(*raster.GetLastPostprocessRT(), (dir / "raster" / (name + ".png")).string());
		if (opt.saveHDR)
		{
			ok &= SaveTexturePFM(raster.GetHDRResult(), (dir / "raster_hdr" / (name + ".pfm")).string());
		}
		if (motionFrom != nullptr)
		{
			ok &= SaveMotion(dir / "motion" / (name + ".npy"), *motionFrom);
		}

		std::printf(" path tracing...");
		std::fflush(stdout);

		// Path tracer:
		//	TAA is a global setting that would also jitter the path tracer camera, which blurs the result
		wi::renderer::SetTemporalAAEnabled(false);
		application.ActivatePath(&pathtracer);
		pathtracer.resetProgress();
		bool noisySaved = false;
		bool converged = false;
		int lastSample = -1;
		int lastPercent = -1;
		for (int frame = 0; frame < frameGuard && !quit; ++frame)
		{
			RunFrame();
			const int sample = pathtracer.getCurrentSampleCount();
			if (sample < lastSample)
			{
				noisySaved = false; // accumulation was reset
			}
			lastSample = sample;

			const int percent = std::min(100, sample * 100 / opt.spp);
			if (percent / 10 != lastPercent / 10)
			{
				lastPercent = percent;
				std::printf(" %d%%", percent);
				std::fflush(stdout);
			}

			if (sample == opt.spp - 1 && !noisySaved)
			{
				// This frame accumulated the last sample and composited the noisy result (the denoiser starts on the next frame):
				noisySaved = true;
				if (!denoise)
				{
					ok &= wi::helper::saveTextureToFile(*pathtracer.GetLastPostprocessRT(), (dir / "pathtraced" / (name + ".png")).string());
					converged = true;
					break;
				}
				if (opt.saveNoisy)
				{
					ok &= wi::helper::saveTextureToFile(*pathtracer.GetLastPostprocessRT(), (dir / "pathtraced_noisy" / (name + ".png")).string());
				}
			}
			if (sample >= opt.spp && pathtracer.IsDenoiseFinished())
			{
				// Render one more frame that composites and tonemaps the denoised result:
				RunFrame();
				if (pathtracer.getCurrentSampleCount() >= opt.spp && pathtracer.IsDenoiseFinished())
				{
					converged = true;
					ok &= wi::helper::saveTextureToFile(*pathtracer.GetLastPostprocessRT(), (dir / "pathtraced" / (name + ".png")).string());
					break;
				}
			}
		}
		if (quit)
			return 2;
		if (!converged)
		{
			std::fprintf(stderr, "\nError: path tracer did not converge for %s (is something in the scene animated?)\n", label.c_str());
			return 1;
		}

		if (opt.saveHDR)
		{
			if (denoise)
			{
				ok &= SaveTexturePFM(pathtracer.GetDenoisedHDR(), (dir / "pathtraced_hdr" / (name + ".pfm")).string());
				if (opt.saveNoisy)
				{
					ok &= SaveTexturePFM(pathtracer.GetNoisyHDR(), (dir / "pathtraced_noisy_hdr" / (name + ".pfm")).string());
				}
			}
			else
			{
				ok &= SaveTexturePFM(pathtracer.GetNoisyHDR(), (dir / "pathtraced_hdr" / (name + ".pfm")).string());
			}
		}
		if (opt.saveAux)
		{
			const bool auxOk =
				SaveTexturePFM(pathtracer.GetAlbedo(), (dir / "albedo" / (name + ".pfm")).string()) &&
				SaveTexturePFM(pathtracer.GetNormal(), (dir / "normal" / (name + ".pfm")).string());
			if (!auxOk)
			{
				std::fprintf(stderr, "\nWarning: auxiliary buffers are only available when the engine is built with Open Image Denoise\n");
			}
		}

		if (!ok)
		{
			std::fprintf(stderr, "\nError: failed to write some outputs of %s\n", label.c_str());
			return 1;
		}
		std::printf(" done\n");
		std::fflush(stdout);
		return 0;
	};

	// Collision geometry (world space triangles of all meshes, including the ground), used to validate the cameras:
	RunFrame(); // updates the transforms of the entities created above
	CollisionWorld world;
	world.Build(scene);
	std::printf("Collision geometry: %zu triangles\n", world.triangles.size());

	int groundObjectIndex = -1;
	for (size_t i = 0; i < scene.objects.GetCount() && groundEntity != INVALID_ENTITY; ++i)
	{
		if (scene.objects.GetEntity(i) == groundEntity)
			groundObjectIndex = (int)i;
	}

	// Focus targets, the camera looks at one of them (drawn per view/animation):
	std::vector<Target> targets;
	auto CollectTargets = [&](float minRadius) {
		targets.clear();
		const size_t objectCount = std::min(scene.objects.GetCount(), scene.aabb_objects.size());
		for (size_t i = 0; i < objectCount; ++i)
		{
			const Entity entity = scene.objects.GetEntity(i);
			if (entity == groundEntity || scene.objects[i].meshID == INVALID_ENTITY)
				continue;
			const wi::primitive::AABB& aabb = scene.aabb_objects[i];
			if (!aabb.IsValid())
				continue;
			const float r = aabb.getRadius();
			if (!(r > 0) || r < minRadius)
				continue;
			Target& target = targets.emplace_back();
			target.objectIndex = (int)i;
			target.entity = entity;
			target.aabb = aabb;
			target.center = aabb.getCenter();
			target.radius = r;
			const NameComponent* nameComponent = scene.names.GetComponent(entity);
			target.name = nameComponent != nullptr ? nameComponent->name : "";
		}
	};
	if (opt.focus == "object")
	{
		CollectTargets(opt.minTargetSize * radius);
		if (targets.empty())
		{
			CollectTargets(0);
		}
		if (targets.empty())
		{
			std::fprintf(stderr, "Warning: no object with a mesh to focus on, using the whole model\n");
		}
	}
	if (targets.empty())
	{
		Target& target = targets.emplace_back();
		target.aabb = bounds;
		target.center = center;
		target.radius = radius;
		target.name = "<scene>";
	}
	std::printf("%zu focus target(s)\n", targets.size());
	auto PickTarget = [&](std::mt19937& rng) {
		return targets[std::uniform_int_distribution<size_t>(0, targets.size() - 1)(rng)];
	};
	// Each drawn target gets several starting point attempts, otherwise the targets that are easy to frame would be favored:
	const int attemptsPerTarget = std::max(1, opt.maxAttempts / 10);

	const float tanY = std::tan(XMConvertToRadians(opt.fov) * 0.5f);
	const float tanX = tanY * float(opt.width) / float(opt.height);
	const float nearCornerFactor = std::sqrt(1 + tanX * tanX + tanY * tanY); // distance to the near plane corners / near
	const wi::primitive::AABB farBounds = wi::primitive::AABB::Merge(bounds, world.bounds);

	// Projection shared by a set of camera positions. The near plane stays closer than the clearance, so it never clips the geometry:
	auto ClipPlanes = [&](const std::vector<CameraPose>& poses, float clearance, float& zNear, float& zFar) {
		zNear = clearance * 0.5f / nearCornerFactor;
		zFar = zNear * 10;
		for (const CameraPose& pose : poses)
		{
			for (int c = 0; c < 8; ++c)
			{
				const XMFLOAT3 corner(
					(c & 1) ? farBounds._max.x : farBounds._min.x,
					(c & 2) ? farBounds._max.y : farBounds._min.y,
					(c & 4) ? farBounds._max.z : farBounds._min.z);
				zFar = std::max(zFar, XMVectorGetX(XMVector3Length(L3(corner) - L3(pose.eye))));
			}
		}
		zFar *= 1.01f;
	};

	// Starting point validation. Returns -1 when valid, otherwise the rejection reason.
	//	The lights must be set. Geometric checks first, then a path traced preview to reject black renders of the focus.
	enum Rejection { RejectCollision, RejectOccluded, RejectInside, RejectCoverage, RejectDark, RejectionCount };
	static const char* rejectionNames[RejectionCount] = { "collision", "occluded", "inside", "low coverage", "dark" };
	struct StartCheck
	{
		float coverage = 0;
		float brightness = -1; // -1: not measured
	};
	bool previewWarned = false;

	// Random start camera for a target, moved in front of occluders. The camera looks at the target center, or for objects half of the time
	//	at a random point of their surface seen from its front side (the center of large or hollow objects is often buried in other geometry).
	//	aimed is the target with its center replaced by the look at point. Returns false if there is no room for the camera.
	auto SampleStart = [&](std::mt19937& rng, const Target& target, float clearance, Target& aimed, OrbitPose& start) {
		aimed = target;
		XMFLOAT3 normal = {};
		if (target.objectIndex >= 0 && std::uniform_int_distribution<int>(0, 1)(rng) == 1)
		{
			world.SampleSurface(target.objectIndex, rng, aimed.center, normal);
		}
		const XMVECTOR N = L3(normal);
		const bool facing = XMVectorGetX(XMVector3LengthSq(N)) > 0;
		for (int i = 0; i < 16; ++i)
		{
			start = SampleOrbitPose(opt, rng, aimed);
			if (!facing || XMVectorGetX(XMVector3Dot(-L3(start.pose.forward), N)) > 0.1f)
				return PlaceInFrontOfOccluders(world, aimed, clearance, start);
		}
		return false;
	};
	auto CheckStart = [&](const CameraPose& start, const Target& target, float clearance, float zNear, float zFar, StartCheck& check) -> int {
		check = StartCheck();
		if (!world.IsClear(start.eye, clearance))
			return RejectCollision; // camera in or too close to the geometry
		std::vector<XMFLOAT2> pixels;
		float backFaceFraction = 0;
		check.coverage = TargetCoverage(world, opt, start, target, groundObjectIndex, pixels, backFaceFraction);
		if (backFaceFraction > 0.5f)
			return RejectInside; // mostly sees the back of surfaces: the camera is inside a closed object
		if (pixels.empty() || check.coverage < opt.minCoverage)
			return RejectCoverage; // focus (mostly) hidden or too small
		if (opt.minBrightness <= 0)
			return -1;

		// Low sample count path traced preview (the darker renderer, the rasterizer has no global illumination by default):
		SetCamera(start.eye, start.forward, zNear, zFar);
		wi::renderer::SetTemporalAAEnabled(false);
		application.ActivatePath(&pathtracer);
		pathtracer.resetProgress();
		const int previewSamples = std::min(opt.spp, 8);
		for (int frame = 0; frame < 1000 && !quit; ++frame)
		{
			RunFrame();
			if (pathtracer.getCurrentSampleCount() >= previewSamples - 1)
				break;
		}
		if (quit)
			return -1; // the caller stops
		std::vector<XMFLOAT3> rgb;
		uint32_t w = 0;
		uint32_t h = 0;
		if (!ReadTextureRGB(*pathtracer.GetLastPostprocessRT(), rgb, w, h) || w == 0 || h == 0)
		{
			if (!previewWarned)
			{
				std::fprintf(stderr, "Warning: could not read back the path traced preview, the brightness check is disabled\n");
				previewWarned = true;
			}
			return -1;
		}
		double sum = 0;
		for (const XMFLOAT2& p : pixels)
		{
			const uint32_t x = std::min(w - 1, uint32_t(p.x * w));
			const uint32_t y = std::min(h - 1, uint32_t(p.y * h));
			const XMFLOAT3& c = rgb[size_t(y) * w + x];
			sum += 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
		}
		check.brightness = float(sum / double(pixels.size()));
		return check.brightness < opt.minBrightness ? RejectDark : -1;
	};
	auto RejectionSummary = [&](const int* counts) {
		std::string s;
		for (int i = 0; i < RejectionCount; ++i)
		{
			if (counts[i] > 0)
				s += (s.empty() ? "" : ", ") + std::to_string(counts[i]) + " " + rejectionNames[i];
		}
		return s.empty() ? std::string("none") : s;
	};
	auto RejectionsToJson = [&](const int* counts) {
		nlohmann::json j;
		for (int i = 0; i < RejectionCount; ++i)
		{
			j[rejectionNames[i]] = counts[i];
		}
		return j;
	};

	int written = 0;

	if (!animationMode)
	{
		for (int viewIndex = opt.firstView; viewIndex < opt.firstView + opt.views && !quit; ++viewIndex)
		{
			const std::string name = ViewName(viewIndex);
			std::seed_seq seq{ (uint32_t)opt.seed, (uint32_t)viewIndex, 0xf0c5u };
			std::mt19937 rng(seq);
			const LightSetup lighting = SampleLights(opt, viewIndex, center, radius);
			ApplyLights(lighting);

			Target target;
			Target aimed;
			OrbitPose view;
			float clearance = 0;
			float zNear = 0;
			float zFar = 0;
			StartCheck check;
			int rejections[RejectionCount] = {};
			bool found = false;
			int attempts = 0;
			while (!found && attempts < opt.maxAttempts && !quit)
			{
				if (attempts % attemptsPerTarget == 0)
					target = PickTarget(rng);
				attempts++;
				clearance = opt.clearance * target.radius;
				if (!SampleStart(rng, target, clearance, aimed, view))
				{
					rejections[RejectOccluded]++;
					continue;
				}
				ClipPlanes({ view.pose }, clearance, zNear, zFar);
				const int rejection = CheckStart(view.pose, target, clearance, zNear, zFar, check);
				if (rejection >= 0)
					rejections[rejection]++;
				else
					found = true;
			}
			if (quit)
				break;
			if (!found)
			{
				std::fprintf(stderr, "[view %s] Warning: no valid camera found in %d attempts (rejected: %s), skipped "
					"(try a smaller --clearance, --min-coverage or --min-brightness)\n", name.c_str(), attempts, RejectionSummary(rejections).c_str());
				continue;
			}
			std::printf("[view %s] focus '%s' (%d attempt(s), coverage %.2f, brightness %.2f)\n", name.c_str(), target.name.c_str(),
				attempts, check.coverage, check.brightness);

			SetCamera(view.pose.eye, view.pose.forward, zNear, zFar);
			const XMFLOAT4X4 viewMatrix = camera.View;
			const XMFLOAT4X4 projectionMatrix = camera.Projection;

			const int result = RenderPair(outDir, name, "[view " + name + "]");
			if (result == 2)
				break;
			if (result != 0)
				return Shutdown(1);

			// Per view metadata:
			nlohmann::json j;
			j["view"] = viewIndex;
			j["target"] = { { "name", target.name }, { "object_index", target.objectIndex }, { "center", ToJson(target.center) }, { "radius", target.radius },
				{ "aim", ToJson(aimed.center) } };
			j["start"] = {
				{ "attempts", attempts },
				{ "rejections", RejectionsToJson(rejections) },
				{ "clearance", clearance },
				{ "coverage", check.coverage },
				{ "brightness", check.brightness },
			};
			j["camera"] = {
				{ "eye", ToJson(view.pose.eye) },
				{ "forward", ToJson(view.pose.forward) },
				{ "up", { 0, 1, 0 } },
				{ "target", ToJson(aimed.center) },
				{ "fov_y_degrees", opt.fov },
				{ "near", zNear },
				{ "far", zFar },
				{ "azimuth", view.azimuth },
				{ "elevation", view.elevation },
				{ "distance", view.distance },
				{ "view_matrix", ToJson(viewMatrix) },
				{ "projection_matrix", ToJson(projectionMatrix) },
			};
			LightsToJson(j, lighting);
			std::ofstream file(outDir / "meta" / (name + ".json"));
			file << j.dump(2) << std::endl;
			if (!file)
			{
				std::fprintf(stderr, "Error: failed to write the metadata of view %s\n", name.c_str());
				return Shutdown(1);
			}
			written++;
		}
	}
	else
	{
		static const char* pathTypes[] = { "orbit", "translate", "mix" };

		for (int animIndex = opt.firstView; animIndex < opt.firstView + opt.views && !quit; ++animIndex)
		{
			const std::string animName = "anim_" + ViewName(animIndex);
			std::seed_seq seq{ (uint32_t)opt.seed, (uint32_t)animIndex, 0xa417u };
			std::mt19937 rng(seq);
			const std::string type = opt.animate == "random" ? pathTypes[std::uniform_int_distribution<int>(0, 2)(rng)] : opt.animate;

			const LightSetup lighting = SampleLights(opt, animIndex, center, radius);
			ApplyLights(lighting);

			CameraPath path;
			Target target;
			Target aimed;
			float clearance = 0;
			bool split = false;
			bool found = false;
			int attempts = 0;
			std::vector<CameraPose> poses(opt.frames);
			float zNear = 0;
			float zFar = 0;
			StartCheck check;
			int rejections[RejectionCount] = {};
			while (!found && attempts < opt.maxAttempts && !quit)
			{
				if (attempts % attemptsPerTarget == 0)
					target = PickTarget(rng);
				attempts++;
				clearance = opt.clearance * target.radius;
				OrbitPose start;
				if (!SampleStart(rng, target, clearance, aimed, start))
				{
					rejections[RejectOccluded]++;
					continue;
				}
				path = GeneratePath(opt, rng, type, aimed, start);
				split = false;
				if (!ValidatePath(path, world, clearance, opt.minSplit, split))
				{
					rejections[RejectCollision]++;
					continue;
				}
				for (int f = 0; f < opt.frames; ++f)
				{
					poses[f] = path.EvalFrame(f, opt.frames);
				}
				ClipPlanes(poses, clearance, zNear, zFar);
				const int rejection = CheckStart(poses[0], target, clearance, zNear, zFar, check);
				if (rejection >= 0)
					rejections[rejection]++;
				else
					found = true;
			}
			if (quit)
				break;
			if (!found)
			{
				std::fprintf(stderr, "[%s] Warning: no valid %s path found in %d attempts (rejected: %s), skipped "
					"(try a smaller --clearance, --orbit-max, --move-max, --min-split, --min-coverage or --min-brightness)\n",
					animName.c_str(), type.c_str(), attempts, RejectionSummary(rejections).c_str());
				continue;
			}

			const std::filesystem::path animDir = outDir / animName;
			if (!CreateOutputDirs(animDir, opt, false, true))
				return Shutdown(1);

			// Animation metadata, written before rendering:
			{
				nlohmann::json j;
				j["animation"] = animIndex;
				j["type"] = type;
				j["frame_count"] = opt.frames;
				j["target"] = { { "name", target.name }, { "object_index", target.objectIndex }, { "center", ToJson(target.center) }, { "radius", target.radius },
					{ "aim", ToJson(aimed.center) } };
				j["clearance"] = clearance;
				j["attempts"] = attempts;
				j["rejections"] = RejectionsToJson(rejections);
				j["start"] = { { "coverage", check.coverage }, { "brightness", check.brightness } };
				j["split"] = split;
				j["path_interval"] = { path.s0, path.s1 };
				j["path_length"] = path.length * (path.s1 - path.s0);
				nlohmann::json segments = nlohmann::json::array();
				for (const PathSegment& segment : path.segments)
				{
					segments.push_back(segment.ToJsonDesc());
				}
				j["segments"] = segments;

				CameraComponent cam;
				cam.CreatePerspective((float)opt.width, (float)opt.height, zNear, zFar, XMConvertToRadians(opt.fov));
				nlohmann::json frames = nlohmann::json::array();
				for (int f = 0; f < opt.frames; ++f)
				{
					cam.Eye = poses[f].eye;
					cam.At = poses[f].forward;
					cam.Up = XMFLOAT3(0, 1, 0);
					cam.UpdateCamera();
					frames.push_back({
						{ "frame", f },
						{ "eye", ToJson(poses[f].eye) },
						{ "forward", ToJson(poses[f].forward) },
						{ "view_matrix", ToJson(cam.View) },
					});
				}
				j["camera"] = {
					{ "fov_y_degrees", opt.fov },
					{ "near", zNear },
					{ "far", zFar },
					{ "up", { 0, 1, 0 } },
					{ "projection_matrix", ToJson(cam.Projection) },
				};
				j["frames"] = frames;
				LightsToJson(j, lighting);
				std::ofstream file(animDir / "animation.json");
				file << j.dump(2) << std::endl;
				if (!file)
				{
					std::fprintf(stderr, "Error: failed to write %s\n", (animDir / "animation.json").string().c_str());
					return Shutdown(1);
				}
			}

			std::printf("[%s] %s path around '%s' (%zu segment(s), %d attempt(s)%s, start coverage %.2f, brightness %.2f)\n", animName.c_str(), type.c_str(),
				target.name.c_str(), path.segments.size(), attempts, split ? ", cut to its valid part" : "", check.coverage, check.brightness);
			std::fflush(stdout);

			for (int f = 0; f < opt.frames && !quit; ++f)
			{
				SetCamera(poses[f].eye, poses[f].forward, zNear, zFar);
				char label[96];
				std::snprintf(label, sizeof(label), "[%s frame %d/%d]", animName.c_str(), f + 1, opt.frames);
				const int result = RenderPair(animDir, ViewName(f), label, &poses[std::max(f - 1, 0)]);
				if (result == 2)
					break;
				if (result != 0)
					return Shutdown(1);
			}
			if (quit)
				break;
			written++;
		}
	}

	std::printf("Wrote %d %s to %s\n", written, animationMode ? "animation(s)" : "view(s)", outDir.string().c_str());
	return Shutdown(quit ? 1 : 0);
}
