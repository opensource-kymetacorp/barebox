#ifndef __ARRIA10_PLAT_H
#define __ARRIA10_PLAT_H

/* HANDOFF2 register bit fields */
#define FPGA_BOOT_PARTITION    (1 << 0)
#define FPGA_PROGRAM_FAILED    (1 << 1)
#define FPGA_PROGRAMMED        (1 << 2)

extern bool has_eui64_mac(void);

#endif
