#include <common.h>
#include <init.h>
#include <driver.h>
#include <io.h>
#include <fcntl.h>
#include <command.h>
#include <getopt.h>
#include <magicvar.h>
#include <globalvar.h>
#include <of.h>
#include <environment.h>
#include "arria10-plat.h"

static int mmc_bus_of_fixup(struct device_node *root, void *ctx);

/* MAC addresses are stored in EUI64 EEPROM chip */
bool has_eui64_mac(void)
{
	return false;
}

/* Board has only 4-bit eMMC bus */
static bool has_4bit_mmc(void)
{
	return true;
}

/* Adjust MMC bus width */
static int mmc_bus_of_fixup(struct device_node *root, void *ctx)
{
	struct device_node *node;

	if (!has_4bit_mmc())
		return 0;

	for_each_compatible_node_from(node, root, NULL, "altr,socfpga-dw-mshc") {
		struct device_node *slot;

		if (!of_device_is_available(node))
			continue;

		/* Get the controller node */
		of_property_write_u32(node, "bus-width", 4);
		/* and any slots if it's using those */
		for (slot = of_find_node_with_property(node, "bus-width"); slot;
			slot = of_find_node_with_property(slot, "bus-width")) {
			of_property_write_u32(slot, "bus-width", 4);
		}
	}
	return 0;
}

static int register_board_fixups(void)
{
	of_register_fixup(mmc_bus_of_fixup, NULL);
	return 0;
}
late_initcall(register_board_fixups);

