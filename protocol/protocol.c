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
	size_t len = strlen(str);
	uint8_t *data = buf;

	data[0] = TEXT;
	data[1] = len;
	memcpy(&data[2], str, len);
	return parse_header(buf, NULL, NULL, NULL);
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
	size_t offset;
	size_t len;
	size_t ret = parse_header(buf, NULL, &offset, &len);
	memcpy(str, &data[offset], len);
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
 * @param[out] type type of packet
 * @param[out] payload_len size of unpacked payload data in bytes
 * @return size of packet in bytes
 */
size_t parse_header(const void* buf, int *type, size_t *payload_offset, size_t *payload_len)
{
	const uint8_t *data = buf;
	const size_t offset = 2;
	const size_t len = data[1];
	if (type)
		*type = data[0];
	if (payload_len)
		*payload_len = len;
	if (payload_offset)
		*payload_offset = offset;
	return len + offset;
}
