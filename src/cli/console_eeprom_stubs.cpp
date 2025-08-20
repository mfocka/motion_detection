#include <cstdio>
#include <cstdarg>
#include <vector>
#include <mutex>
#include <string>
#include <cstring>

#include "motion_detection/motion_detection.h"

// Simple in-memory EEPROM blob
static std::vector<unsigned char> g_eeprom;
static std::mutex g_mutex;

struct ConsoleImpl : Console {
	void printOutput(const char* fmt, ...) override {
		va_list args; va_start(args, fmt);
		vprintf(fmt, args);
		va_end(args);
	}
	void printOutputWOTime(const char* fmt, ...) override {
		va_list args; va_start(args, fmt);
		vprintf(fmt, args);
		va_end(args);
	}
};

Console* console = new ConsoleImpl();

bool saveToEEPROM(const void* data, uint32_t sizeBytes) {
	std::lock_guard<std::mutex> lk(g_mutex);
	g_eeprom.assign((const unsigned char*)data, (const unsigned char*)data + sizeBytes);
	return true;
}

bool loadFromEEPROM(void* data, uint32_t sizeBytes) {
	std::lock_guard<std::mutex> lk(g_mutex);
	if (g_eeprom.size() != sizeBytes) return false;
	memcpy(data, g_eeprom.data(), sizeBytes);
	return true;
}

