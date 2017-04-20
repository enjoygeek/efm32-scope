/*
 * pack_unpack.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: bulashevich
 */
#include "../protocol.h"
#include <iostream>
#include <array>
#include <vector>
#include <cstring>
#include <algorithm>

using namespace std;

template<class InpitIt, typename T>
size_t get_dirty_length(const InpitIt &first, const InpitIt &last, const T& guard_element)
{
	const reverse_iterator<InpitIt> rend(last);
	const reverse_iterator<InpitIt> rbegin(first);

	const reverse_iterator<InpitIt> last_dirty = find_if_not(rend, rbegin,
					[guard_element](const T &x){return x == guard_element;});
	return rbegin - last_dirty;
}

template <typename T>
size_t get_dirty_length(const T *buf, size_t max_len, const T& guard_element)
{
	return get_dirty_length(move_iterator<const T*>(&buf[0]), move_iterator<const T*>(&buf[max_len]), guard_element);
}

template <typename Pkt, typename Data, typename T>
void check_sizes(const Pkt &packed, uint8_t packed_guard, const Data &unpacked, const T& unpacked_guard, size_t packet_len)
{
	size_t parsed_unpacked_len;
	parse_header(packed.data(), NULL, NULL, &parsed_unpacked_len);
	size_t used_packed_len = get_dirty_length(packed.begin(), packed.end(), packed_guard);
	size_t used_unpacked_len = get_dirty_length(unpacked.begin(), unpacked.end(), unpacked_guard);

	if (used_packed_len != packet_len)
		cout << "packed lenght mismatch. reported = " << packet_len << " used = " << used_packed_len << endl;
	if (used_unpacked_len != parsed_unpacked_len)
		cout << "unpacked lenght mismatch. reported = " << parsed_unpacked_len << " used = " << used_unpacked_len << endl;
}

void test_text(const char *str)
{
	const uint8_t guard_value = 0x11;
	cout << __FUNCTION__ << ": " << str << endl;
	size_t len = strlen(str) + 5;
	std::vector<uint8_t> buf(len * 2, guard_value);
	std::vector<char> unpacked(len * 2, guard_value);

	size_t packet_len1 = pack_string(buf.data(), buf.size(), str);
	size_t packet_len2 = unpack_string(buf.data(), unpacked.data());
	if (packet_len1 != packet_len2)
		cout << "packet lenghts differs for pack and unpack functions: " << packet_len1 << ", " << packet_len2 << endl;

	check_sizes(buf, guard_value, unpacked, static_cast<char>(guard_value), packet_len1);
}

template <std::size_t SIZE>
void test_int16array(const std::array<int16_t, SIZE> &input)
{
	const uint8_t byte_guard = 0xFF;
	const int16_t guard = 0xAAAA;
	cout << __FUNCTION__ << ": " << endl;
	array<int16_t, SIZE> output;
	output.fill(guard);

	std::array<uint8_t, SIZE*4 + 10> buf;
	buf.fill(byte_guard);

	size_t packet_len1 = pack_int16array(buf.data(), buf.size(), input.data(), input.size());
	size_t packet_len2 = unpack_int16array(buf.data(), output.data());
	if (packet_len1 != packet_len2)
		cout << "packet lenghts differs for pack and unpack functions: " << packet_len1 << ", " << packet_len2 << endl;

	check_sizes(buf, byte_guard, output, guard, packet_len1);
}


int main()
{
	test_text("");
	test_text("Hello");
	array<int16_t, 4> arr {{0, 1, 5, 8}};
	test_int16array(arr);
}


