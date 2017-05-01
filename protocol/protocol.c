/*
 * protocol.c
 *
 *  Created on: Apr 15, 2017
 *      Author: bulashevich
 */

#include "protocol.h"
#include <string.h>

/**
 *
 * @param buf output buffer
 * @param buflen length of output buffer
 * @param str input string
 * @return position of the first free byte in output buffer
 */
size_t pack_string(void* buf, size_t buflen, const char* str)
{
	if (buflen < 2)
		return 0;

	uint8_t *data = buf;
	size_t len = strlen(str);
	len = len > buflen - 2 ? buflen - 2 : len;

	data[0] = TEXT;
	data[1] = len;
	memcpy(&data[2], str, len);
	return parse_header(buf, NULL);
}

/**
 *
 * @param buf input buffer
 * @param str output buffer
 * @return position in input buffer after last byte of unpacked string
 */
size_t unpack_string(const void* buf, char* str)
{
	const uint8_t *data = buf;
	struct packet_info info;
	size_t ret = parse_header(buf, &info);
	memcpy(str, &data[info.header_len], info.payload_len);
	return ret;
}

size_t pack_int16array(void* buf, size_t buflen, const int16_t* src, size_t len)
{
	uint8_t *data = buf;
	data[0] = READINGS;
	data[1] = len*2;
	memcpy(&data[2], src, len*2);
	return len*2 + 2;
}

size_t unpack_int16array(const void* buf, int16_t* dst)
{
	const uint8_t *data = buf;
	memcpy(dst, &data[2], data[1]);
	return 0;
}

/**
 *
 * @param[in] buf input buffer
 * @param[out] info structure describing packet
 * @return size of packet in bytes
 */
size_t parse_header(const void* buf, struct packet_info *info)
{
	const uint8_t *data = buf;
	const size_t offset = 2;
	const size_t len = data[1];
	if (info != NULL)
	{
		info->type = data[0];
		info->payload_len = len;
		info->header_len = offset;
		info->packet_len = offset + len;
	}
	return offset + len;
}
