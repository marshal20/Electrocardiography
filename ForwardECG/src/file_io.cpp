#include "file_io.h"
#include <stdio.h>
#include <stdlib.h>


uint8_t* file_read(const char* file_path, size_t* out_size)
{
	// open file as read binary
	FILE* file = fopen(file_path, "rb");
	if (!file)
	{
		return NULL;
	}

	// get file size
	size_t file_size;
	if (fseek(file, 0, SEEK_END) != 0)
	{
		fclose(file);
		return NULL;
	}
	file_size = ftell(file);
	if (fseek(file, 0, SEEK_SET) != 0)
	{
		fclose(file);
		return NULL;
	}

	// allocate memory for file contents
	uint8_t* contents = (uint8_t*)malloc(file_size);
	if (!contents)
	{
		fclose(file);
		return NULL;
	}

	// read 
	fread(contents, sizeof(uint8_t), file_size, file);

	// close the file
	fclose(file);

	if (out_size)
	{
		*out_size = file_size;
	}
	return contents;
}


bool file_write(const char* file_path, const uint8_t* data, size_t size)
{
	// open file as read binary
	FILE* file = fopen(file_path, "wb");
	if (!file)
	{
		return false;
	}

	// write
	fwrite(data, sizeof(uint8_t), size, file);

	// close the file
	fclose(file);

	return true;
}

bool file_write(const char* file_path, const std::vector<uint8_t>& data)
{
	return file_write(file_path, &data[0], data.size());
}

