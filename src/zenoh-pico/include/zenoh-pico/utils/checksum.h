//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

#ifndef ZENOH_PICO_UTILS_CHECKSUM_H
#define ZENOH_PICO_UTILS_CHECKSUM_H

#include <stdint.h>
#include <stdlib.h>

uint32_t _z_crc32(const uint8_t *message, size_t len);

#endif /* ZENOH_PICO_UTILS_CHECKSUM_H */
