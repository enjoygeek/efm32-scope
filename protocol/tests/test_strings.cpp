/*
 * catch_unpack.cpp
 *
 *  Created on: Apr 29, 2017
 *      Author: bulashevich
 */

#include "catch.hpp"

#include <array>
#include <algorithm>

#include "../protocol.h"

template<class InpitIt, typename T>
size_t get_dirty_length(const InpitIt &first, const InpitIt &last, const T& guard_element)
{
	const std::reverse_iterator<InpitIt> rend(last);
	const std::reverse_iterator<InpitIt> rbegin(first);

	const std::reverse_iterator<InpitIt> last_dirty = find_if_not(rend, rbegin,
					[guard_element](const T &x){return x == guard_element;});
	return rbegin - last_dirty;
}

template <typename T>
size_t get_dirty_length(const T *buf, size_t max_len, const T& guard_element)
{
	return get_dirty_length(std::move_iterator<const T*>(&buf[0]), std::move_iterator<const T*>(&buf[max_len]), guard_element);
}


class TestStringFixture
{
public:
	TestStringFixture();
protected:
	const uint8_t guard_value = 0x11;
	std::array<uint8_t, 5000> buf;
	std::array<char, 5000> unpacked;

	void test_parse_header(enum payload_types type) const;
};

TestStringFixture::TestStringFixture()
{
	buf.fill(guard_value);
	unpacked.fill(guard_value);
}

void TestStringFixture::test_parse_header(enum payload_types type) const
{
	size_t parsed_length;
	size_t used_space = get_dirty_length(buf.begin(), buf.end(), guard_value);
	struct packet_info info;

	parsed_length = parse_header(buf.data(), NULL);
	REQUIRE(used_space == parsed_length);

	parsed_length = parse_header(buf.data(), &info);
	REQUIRE(used_space == parsed_length);
	REQUIRE(info.packet_len == parsed_length);
	REQUIRE(info.header_len + info.payload_len == parsed_length);
	REQUIRE(info.type == type);
}

TEST_CASE_METHOD(TestStringFixture, "pack_string", "[string][pack]")
{
	std::string input = "Normal test string";
	size_t output_offset;

	for (size_t limit = 0; limit < input.length()*2; limit += 5)
	{
		output_offset = pack_string(buf.data(), limit, input.data());
		REQUIRE(get_dirty_length(buf.begin(), buf.end(), guard_value) <= limit);
		REQUIRE(get_dirty_length(buf.begin(), buf.end(), guard_value) == output_offset);
	}
}

TEST_CASE_METHOD(TestStringFixture, "parse_header", "[string][parse]")
{
	std::string input = "Normal test string";
	pack_string(buf.data(), 5, input.data());
	test_parse_header(TEXT);

	pack_string(buf.data(), 500, input.data());
	test_parse_header(TEXT);
}

TEST_CASE_METHOD(TestStringFixture, "parse_header for empty payload", "[string][parse]")
{
	pack_string(buf.data(), buf.size(), "");
	test_parse_header(TEXT);

	struct packet_info info;
	parse_header(buf.data(), &info);

	REQUIRE(info.payload_len == 0);
}

TEST_CASE_METHOD(TestStringFixture, "unpack_string", "[string][unpack]")
{
	std::string input = "Normal test string";
	pack_string(buf.data(), buf.size(), input.data());

	struct packet_info info;
	parse_header(buf.data(), &info);
	REQUIRE(info.payload_len < unpacked.size()); //to prevent buffer overrun you MUST check payload_len BEFORE calling unpack_* routines.

	size_t input_offset = unpack_string(buf.data(), unpacked.data());
	std::string output(unpacked.data(), info.payload_len);

	REQUIRE(input == output);
	REQUIRE(get_dirty_length(buf.begin(), buf.end(), guard_value) == input_offset);
	REQUIRE(get_dirty_length(unpacked.begin(), unpacked.end(), guard_value) == output.length());
}

TEST_CASE_METHOD(TestStringFixture, "unpack_string truncated", "[string][unpack]")
{
	std::string input = "Normal test string";
	pack_string(buf.data(), 5, input.data());

	struct packet_info info;
	parse_header(buf.data(), &info);
	REQUIRE(info.payload_len < unpacked.size()); //to prevent buffer overrun you MUST check payload_len BEFORE calling unpack_* routines.

	size_t input_offset = unpack_string(buf.data(), unpacked.data());
	std::string output(unpacked.data(), info.payload_len);

	REQUIRE_THAT(input, Catch::StartsWith(output));
	REQUIRE(get_dirty_length(buf.begin(), buf.end(), guard_value) == input_offset);
	REQUIRE(get_dirty_length(unpacked.begin(), unpacked.end(), guard_value) == output.length());

}
