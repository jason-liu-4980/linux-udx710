/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UDX710_TRIAL_H
#define _UDX710_TRIAL_H
/*
 * Fixed SC2730 RTC spare profile. Only bits 11:9 encode the stage;
 * alarm lock bits 7:0, alarm flag bit8 and bits15:12 stay unchanged.
 */
#define UDX_RTC_ORIGINAL 0xb65au
#define UDX_RTC_ENTRY_GUARD 0xb45au
#define UDX_RTC_IMAGE_ENTRY 0xb05au
#define UDX_RTC_KERNEL_EARLY 0xb25au
#define UDX_RTC_DRIVER_FRAMEWORK 0xb85au
#define UDX_RTC_INITRAMFS 0xba5au
#define UDX_RTC_RECOVERY_READY 0xbc5au
#define UDX_RTC_RUNTIME_VERIFIED 0xbe5au
#define UDX_RTC_OPENWRT_READY UDX_RTC_RUNTIME_VERIFIED
static inline int udx_rtc_stage_value(unsigned int value)
{
	return (value & ~0x0e00u) == 0xb05au;
}
#endif
