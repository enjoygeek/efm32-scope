/*
 * protocol.c
 *
 *  Created on: Apr 15, 2017
 *      Author: bulashevich
 */

#include "protocol.h"
#include <string.h>

size_t pack_string(void* buf, size_t buflen, const char* str)
{
	size_t len = strlen(str);
	uint8_t *data = buf;

	data[0] = TEXT;
	data[1] = len;
	memcpy(&data[2], str, len);
	return len;
}
