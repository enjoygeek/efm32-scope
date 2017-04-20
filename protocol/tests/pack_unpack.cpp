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

using namespace std;
void test_text(const char *str)
{
	cout << __FUNCTION__ << ": " << str << endl;
	size_t len = strlen(str) + 5;
	std::vector<uint8_t> buf(len*2);
	std::vector<char> unpacked(len*2);
	pack_string(buf.data(), buf.size(), str);
	unpack_string(buf.data(), unpacked.data());

	if (strcmp(str, unpacked.data()) != 0)
	{
		cout << "\t unpacked not equal to original:" << endl
			 << "\t original: " << str << endl
			 << "\t unpacked: " << unpacked.data() << endl;
	}
	else
	{
		cout << "\t success" << endl;
	}
}

template <std::size_t SIZE>
void test_int16array(const std::array<int16_t, SIZE> &input)
{
	cout << __FUNCTION__ << ": " << endl;
	array<int16_t, SIZE> output;
	std::array<uint8_t, SIZE*4 + 10> buf;
	pack_int16array(buf.data(), buf.size(), input.data(), input.size());
	unpack_int16array(buf.data(), output.data());

	if (output != input)
	{
		cout << "unpacked not equal to original" << endl;
	}
	else
	{
		cout << "\t success" << endl;
	}
}

int main()
{
	test_text("");
	test_text("Hello");
	array<int16_t, 4> arr {{0, 1, -3, 5}};
	test_int16array(arr);
	return 0;
}


