#pragma once
#include <stdint.h>
#include <vector>
#include <string>


// Notes:
//   * serializer serializes data to a series of bytes, all data is converted to network byte order.
//   * strings are serialized with the size (uint16_t) + the null-terminated string for security.
//   * arrays are serialized with the size (uint32_t) + the elements of the array


// Serializer

class Serializer
{
	Serializer();
	~Serializer();

	void push_i8(const int8_t val);
	void push_i16(const int16_t val);
	void push_i32(const int32_t val);
	void push_i64(const int64_t val);
	void push_u8(const uint8_t val);
	void push_u16(const uint16_t val);
	void push_u32(const uint32_t val);
	void push_u64(const uint64_t val);
	void push_float(const float val);
	void push_double(const double val);

	void push_string(const std::string& str);
	void push_array_u8(const std::vector<uint8_t>& bytes);
	void push_array_u8(const uint8_t* buffer, const size_t size);

	// push individual bytes
	void push_bytes(const std::vector<uint8_t>& bytes);
	void push_bytes(const uint8_t* buffer, const size_t size);

	const std::vector<uint8_t>& get_data() const;

private:
	std::vector<uint8_t> m_data;
};


// Deserializer

class Deserializer
{
	Deserializer(const std::vector<uint8_t>& data);
	Deserializer(const uint8_t* buffer, const size_t size);
	~Deserializer();

	int8_t parse_i8();
	int16_t parse_i16();
	int32_t parse_i32();
	int64_t parse_i64();
	uint8_t parse_u8();
	uint16_t parse_u16();
	uint32_t parse_u32();
	uint64_t parse_u64();
	float parse_float();
	double parse_double();

	std::string parse_string();
	std::vector<uint8_t> push_array_u8();

	std::vector<uint8_t> parse_bytes(const size_t size);
	void parse_bytes(uint8_t* buffer, const size_t size);

	size_t remaining_size() const;

private:
	std::vector<uint8_t> m_data;
	size_t m_pointer;
};

