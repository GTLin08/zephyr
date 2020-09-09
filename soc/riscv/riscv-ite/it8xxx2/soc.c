/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <kernel.h>

#include <device.h>
#include <init.h>
#include <sys/util.h>


void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);
}
