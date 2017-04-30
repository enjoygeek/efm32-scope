/*
 * catch_unpack.cpp
 *
 *  Created on: Apr 29, 2017
 *      Author: bulashevich
 */

#include "catch.hpp"


#include <vector>
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


TEST_CASE( "Normal string", "[string]")
{
	const uint8_t guard_value = 0x11;
	std::string str = "Hello protocol";
	size_t input_len = str.length();
	std::vector<uint8_t> buf(input_len*2, guard_value);
	std::vector<char> unpacked(input_len*2, guard_value);

	size_t output_offset = pack_string(buf.data(), buf.size(), str.data());
	size_t input_offset = unpack_string(buf.data(), unpacked.data());

	int type;
	size_t payload_offset, payload_len;
	CHECK(parse_header(buf.data(), &type, &payload_offset, &payload_len) == output_offset);
	REQUIRE(type == TEXT);
	REQUIRE(payload_len == input_len);

	CHECK(str == std::string(unpacked.data(), payload_len));
	CHECK(output_offset == input_offset);
	CHECK(get_dirty_length(buf.begin(), buf.end(), guard_value) == output_offset);
	CHECK(get_dirty_length(unpacked.begin(), unpacked.end(), guard_value) == input_len);
}

