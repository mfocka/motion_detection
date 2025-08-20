#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "motion_detection/motion_detection.h"

using namespace motion_detection;

static void usage() {
	std::fprintf(stderr, "Usage: motion_runner [--csv file] [--thresholds alt az] [--windows t_detect t_validate t_hold_min]\n");
}

int main(int argc, char** argv) {
	const char* csv = nullptr;
	float alt=5.0f, az=10.0f;
	float tdet=2.0f, tval=5.0f, thold=60.0f;
	for (int i=1;i<argc;i++) {
		if (std::strcmp(argv[i], "--csv") == 0 && i+1<argc) { csv = argv[++i]; }
		else if (std::strcmp(argv[i], "--thresholds") == 0 && i+2<argc) { alt = std::atof(argv[++i]); az = std::atof(argv[++i]); }
		else if (std::strcmp(argv[i], "--windows") == 0 && i+3<argc) { tdet = std::atof(argv[++i]); tval = std::atof(argv[++i]); thold = std::atof(argv[++i]); }
		else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) { usage(); return 0; }
		else { std::fprintf(stderr, "Unknown arg: %s\n", argv[i]); usage(); return 1; }
	}

	MotionDetection_Init();
	MotionDetection_set_thresholds(alt, az);
	MotionDetection_set_windows(tdet, tval, thold);

	// If CSV provided, read lines: ax,ay,az,gx,gy,gz; optionally timestamp ticks
	// Otherwise, just run a small synthetic impulse
	if (csv) {
		FILE* f = std::fopen(csv, "r");
		if (!f) { std::perror("open csv"); return 1; }
		char line[256];
		uint32_t tick = 0;
		while (std::fgets(line, sizeof(line), f)) {
			if (line[0] == '#') continue;
			float ax, ay, az, gx, gy, gz; unsigned t;
			int n = std::sscanf(line, "%f,%f,%f,%f,%f,%f,%u", &ax,&ay,&az,&gx,&gy,&gz,&t);
			if (n < 6) continue;
			if (n >= 7) tick = t; else tick += 1;
			MotionDetection_Update(ax, ay, az, gx, gy, gz, tick);
		}
		std::fclose(f);
	} else {
		// Synthetic: 2s stationary, then abrupt 15deg yaw over ~0.2s via gy (WDS Down axis), then hold still 10s
		const float fs = 104.0f;
		uint32_t tick = 0;
		// 2s still
		for (int i=0;i<(int)(2*fs);++i) {
			MotionDetection_Update(0,0,1, 0,0,0, tick++);
		}
		// 0.2s yaw: 15deg over 0.2s => 75 deg/s about Down (gy in WDS)
		for (int i=0;i<(int)(0.2f*fs);++i) {
			MotionDetection_Update(0,0,1, 0,75.0f,0, tick++);
		}
		// settle 10s
		for (int i=0;i<(int)(10*fs);++i) {
			MotionDetection_Update(0,0,1, 0,0,0, tick++);
		}
	}

	// Report final state
	char s[32]; MotionDetection_GetState(s, sizeof(s));
	std::fprintf(stdout, "FinalState,%s\n", s);
	float az_deg=0, alt_deg=0; MotionDetection_GetDeltas(&az_deg, &alt_deg);
	std::fprintf(stdout, "FinalDeltas,%.3f,%.3f\n", az_deg, alt_deg);
	return 0;
}

