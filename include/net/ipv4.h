/*
 * Race Capture Firmware
 *
 * Copyright (C) 2016 Autosport Labs
 *
 * This file is part of the Race Capture firmware suite
 *
 * This is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details. You should
 * have received a copy of the GNU General Public License along with
 * this code. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _IPV4_H_
#define _IPV4_H_

#include "cpp_guard.h"
#include <stdint.h>

CPP_GUARD_BEGIN

typedef uint32_t ipv4_val_t;

struct ipv4_iface {
        ipv4_val_t addr;
        ipv4_val_t mask;
        ipv4_val_t gtwy;
};

ipv4_val_t ipv4_parse_val(const char *str);

ipv4_val_t ipv4_get_broadcast_addr(const ipv4_val_t addr,
                                   const ipv4_val_t mask);

void ipv4_to_str(const ipv4_val_t val);

CPP_GUARD_END

#endif /* _IPV4_H_ */
