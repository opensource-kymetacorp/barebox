#include <common.h>
#include <linux/sizes.h>
#include <io.h>
#include <asm/barebox-arm-head.h>
#include <asm/barebox-arm.h>
#include <asm/cache.h>
#include <debug_ll.h>
#include <mach/arria10-sdram.h>
#include <mach/arria10-regs.h>
#include <mach/arria10-reset-manager.h>
#include <mach/arria10-clock-manager.h>
#include <mach/arria10-pinmux.h>
#include "pll-config-arria10.c"
#include "pinmux-config-arria10.c"
#include <mach/generic.h>

extern char __dtb_socfpga_arria10_plat_start[];

#ifdef CONFIG_ARCH_SOCFPGA_XLOAD

#else

ENTRY_FUNCTION(start_socfpga_arria10_plat, r0, r1, r2)
{
	void *fdt;
	arm_cpu_lowlevel_init();

	arm_early_mmu_cache_invalidate();
	relocate_to_current_adr();
	setup_c();
	puts_ll("Next Stage\n");

	fdt = __dtb_socfpga_arria10_plat_start - get_runtime_offset();
	barebox_arm_entry(0x0, SZ_2G, fdt);
}

#endif
