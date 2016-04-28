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

#include "net/ipv4.h"
#include <string.h>

ipv4_val_t ipv4_parse_val(const char *str)
{
        const char* oct[4];
        size_t i;
        for (i = 0; i < 4 && str; ++i) {
                oct[i] = str;
                str = strchr(str, '.');
        }

        if (i != 4)
                return 0;


}

ipv4_val_t ipv4_get_broadcast_addr(const ipv4_val_t addr,
                                   const ipv4_val_t mask)
{

}

void ipv4_to_str(const ipv4_val_t val)
{

}
