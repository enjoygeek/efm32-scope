/*
 * protocol.h
 *
 *  Created on: Apr 15, 2017
 *      Author: bulashevich
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

size_t parse_header(const void* buf, int *type, size_t *payload_offset, size_t *payload_len);

/**
 *
 */
size_t pack_string(void* buf, size_t buflen, const char* str);
size_t unpack_string(const void* buf, char* str);

size_t pack_int16array(void* buf, size_t buflen, const int16_t* src, size_t len);
size_t unpack_int16array(const void* buf, int16_t* dst);

struct configuration
{
	unsigned active_channels_mask;
	unsigned multipliers[0];
};
size_t unpack_config(void* buf, size_t buflen, struct configuration* dst, size_t max_channel_number);


struct protocol_header
{
	uint8_t type;
	uint16_t size;
	uint8_t payload[0];
};

enum types_enum {
	TEXT = 0,
	READINGS,
	CONFIG
};

#ifdef __cplusplus
} //extern "C" {
#endif

#endif /* PROTOCOL_PROTOCOL_H_ */
