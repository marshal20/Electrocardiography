#pragma once
#include <stdint.h>
#include <vector>

// caller must free the buffer
uint8_t* file_read(const char* file_path, size_t* out_size);

bool file_write(const char* file_path, const uint8_t* data, size_t size);
bool file_write(const char* file_path, const std::vector<uint8_t>& data);
