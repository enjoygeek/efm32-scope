/*
 * protocol.h
 *
 *  Created on: Apr 15, 2017
 *      Author: bulashevich
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_

#include <stdint.h>
#include <stdlib.h>
struct protocol_header;

size_t pack_string(void* buf, size_t buflen, const char* str);
size_t unpack_string(const void* buf, char* str);

size_t pack_readings(void* buf, size_t buflen, const uint16_t* src);
size_t unpack_readings(const void* buf, size_t buflen, uint16_t* dst);

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

#endif /* PROTOCOL_PROTOCOL_H_ */
