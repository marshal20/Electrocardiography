#include "serializer.h"
#include <assert.h>
#include "sockimpl.h"


Serializer::Serializer()
{
	
}

Serializer::~Serializer()
{

}


void Serializer::push_i8(const int8_t val)
{
	push_bytes((const uint8_t*)&val, sizeof(int8_t));
}

void Serializer::push_i16(const int16_t val)
{
	uint16_t val_n = htons(*(uint16_t*)&val);
	push_bytes((const uint8_t*)&val_n, sizeof(int16_t));
}

void Serializer::push_i32(const int32_t val)
{
	uint32_t val_n = htonl(*(uint32_t*)&val);
	push_bytes((const uint8_t*)&val_n, sizeof(int32_t));
}

void Serializer::push_i64(const int64_t val)
{
	uint64_t val_n = htonll(*(uint64_t*)&val);
	push_bytes((const uint8_t*)&val_n, sizeof(int64_t));
}

void Serializer::push_u8(const uint8_t val)
{
	push_bytes((const uint8_t*)&val, sizeof(uint8_t));
}

void Serializer::push_u16(const uint16_t val)
{
	uint16_t val_n = htons(val);
	push_bytes((const uint8_t*)&val_n, sizeof(uint16_t));
}

void Serializer::push_u32(const uint32_t val)
{
	uint32_t val_n = htonl(val);
	push_bytes((const uint8_t*)&val_n, sizeof(uint32_t));
}

void Serializer::push_u64(const uint64_t val)
{
	uint64_t val_n = htonll(val);
	push_bytes((const uint8_t*)&val_n, sizeof(uint64_t));
}

void Serializer::push_float(const float val)
{
	uint32_t val_n = htonf(val);
	push_bytes((const uint8_t*)&val_n, sizeof(float));
}

void Serializer::push_double(const double val)
{
	uint64_t val_n = htond(val);
	push_bytes((const uint8_t*)&val_n, sizeof(double));
}


void Serializer::push_string(const std::string& str)
{
	// push size (16 bits)
	push_u16(str.size());
	// push null-terminated string
	push_bytes((uint8_t*)str.c_str(), str.size()+1);
}

void Serializer::push_array_u8(const std::vector<uint8_t>& bytes)
{
	// push size (32 bits)
	push_u32(bytes.size());
	// push null-terminated string
	push_bytes(bytes);
}

void Serializer::push_array_u8(const uint8_t* buffer, const size_t size)
{
	// push size (32 bits)
	push_u32(size);
	// push null-terminated string
	push_bytes(buffer, size);
}


void Serializer::push_bytes(const std::vector<uint8_t>& bytes)
{
	push_bytes(&bytes[0], bytes.size());
}

void Serializer::push_bytes(const uint8_t* buffer, const size_t size)
{
	m_data.reserve(size);
	for (size_t i = 0; i < size; i++)
	{
		m_data.push_back(buffer[i]);
	}
}


const std::vector<uint8_t>& Serializer::get_data() const
{
	return m_data;
}



// Deserializer


Deserializer::Deserializer(const std::vector<uint8_t>& data)
	: m_data(data), m_pointer(0)
{

}

Deserializer::Deserializer(const uint8_t* buffer, const size_t size)
	: m_pointer(0)
{
	m_data.resize(size);

	for (size_t i = 0; i < size; i++)
	{
		m_data[i] = buffer[i];
	}
}

Deserializer::~Deserializer()
{

}


int8_t Deserializer::parse_i8()
{
	uint8_t val;

	parse_bytes((uint8_t*)&val, sizeof(int16_t));

	return *(int16_t*)&val;
}

int16_t Deserializer::parse_i16()
{
	uint16_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(int16_t));
	uint16_t val = ntohs(val_n);

	return *(int16_t*)&val;
}

int32_t Deserializer::parse_i32()
{
	uint32_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(int32_t));
	uint32_t val = ntohl(val_n);

	return *(int32_t*)&val;
}

int64_t Deserializer::parse_i64()
{
	uint64_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(int64_t));
	uint64_t val = ntohll(val_n);

	return *(int64_t*)&val;
}

uint8_t Deserializer::parse_u8()
{
	uint8_t val;
	
	parse_bytes((uint8_t*)&val, sizeof(uint8_t));
	return val;
}

uint16_t Deserializer::parse_u16()
{
	uint16_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(uint16_t));
	return ntohs(val_n);
}

uint32_t Deserializer::parse_u32()
{
	uint32_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(uint32_t));
	return ntohl(val_n);
}

uint64_t Deserializer::parse_u64()
{
	uint64_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(uint64_t));
	return ntohll(val_n);
}

float Deserializer::parse_float()
{
	uint32_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(float));
	return ntohf(val_n);
}

double Deserializer::parse_double()
{
	uint64_t val_n;

	parse_bytes((uint8_t*)&val_n, sizeof(double));
	return ntohd(val_n);
}


std::string Deserializer::parse_string()
{
	// parse size
	uint16_t size = parse_u16();

	// parse null-terminated string
	std::vector<uint8_t> bytes = parse_bytes(size+1);
	
	// check string size with the null-terminated
	if (size != strlen((const char*)&bytes[0]))
	{
		printf("Warning: string size doesn't match null-terminated string while deserializing\n");
		assert(false);
		return "";
	}

	return std::string((const char*)&bytes[0], size);
}

std::vector<uint8_t> Deserializer::push_array_u8()
{
	// parse size
	uint32_t size = parse_u32();

	// parse array
	return parse_bytes(size);
}


std::vector<uint8_t> Deserializer::parse_bytes(const size_t size)
{
	std::vector<uint8_t> bytes;

	bytes.resize(size);

	for (size_t i = 0; i < size; i++)
	{
		if (remaining_size() == 0)
		{
			printf("Warning: reached the end of the data before deserializing\n");
			assert(false);
			break;
		}

		bytes[i] = m_data[m_pointer++];
	}

	return bytes;
}

void Deserializer::parse_bytes(uint8_t* buffer, const size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		if (remaining_size() == 0)
		{
			printf("Warning: reached the end of the data before deserializing\n");
			assert(false);
			break;
		}

		buffer[i] = m_data[m_pointer++];
	}
}


size_t Deserializer::remaining_size() const
{
	if (m_pointer >= m_data.size())
	{
		return 0;
	}

	return m_data.size() - m_pointer;
}

