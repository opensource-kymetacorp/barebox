#include <common.h>
#include <types.h>
#include <driver.h>
#include <init.h>
#include <environment.h>
#include <globalvar.h>
#include <magicvar.h>
#include <asm/armlinux.h>
#include <malloc.h>
#include <linux/micrel_phy.h>
#include <linux/phy.h>
#include <linux/sizes.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <fcntl.h>
#include <mach/arria10-regs.h>
#include <mach/arria10-sdram.h>
#include <mach/arria10-system-manager.h>
#include <i2c/i2c.h>
#include <net.h>
#include <io.h>
#include <envfs.h>
#include <fs.h>
#include <progress.h>
#include <restart.h>
#include "arria10-plat.h"

/* Locations of eeproms containing network device MAC addresses.  This
 * really would be more appropriate to place in the device tree
 * somehow. */
#define EEPROM_MAC_GBE0 "eeprom1"
#define EEPROM_MAC_GBE1 "eeprom2"

/* EEPROM with boot status information */
#define BOOT_EEPROM     "eeprom1"
#define BOOT_MAXCOUNT   4        /* Max boots before switching partitions */

/* Board specific network initialization code */
#ifdef CONFIG_NET

/* Read a MAC address from I2C EEPROM containing an EUI-48 or EUI-64
 * address, like a 24AA025E48 or 24AA025E64.
 *
 * If the chip has an EUI-64, then this involves converting from EUI-64 to
 * EUI-48, which is not legal.  The conversion is done by dropping the first two
 * octets of the 40 bit extension identifier.  Which means the extension will
 * no longer be unique!
 */
static int i2c_get_mac_addr(const char *eeprom_name, bool eui64, uint8_t *mac_addr)
{
	struct cdev *eeprom;
	u8 buf[eui64 ? 8 : 6];
	int num_bytes_read;

	eeprom = cdev_open(eeprom_name, O_RDONLY);
	if (!eeprom)
		return -ENODEV;

	/* MAC ends at offset 0x100, starts sizeof(buf) bytes before that */
	num_bytes_read = cdev_read(eeprom, buf, sizeof(buf), 0x100 - sizeof(buf), 0);
	if (num_bytes_read != sizeof(buf))
		return num_bytes_read < 0 ? num_bytes_read : -EIO;

	if (eui64) {
		memcpy(mac_addr, buf, 3);  /* Copy OUI */
		memcpy(mac_addr + 3, buf + 5, 3);  /* Copy last three bytes of EID */
	} else
		memcpy(mac_addr, buf, 6);

	return 0;
}

static void register_i2c_ethaddr(const char *alias, const char *eeprom_name)
{
	struct device_node *dnode = of_find_node_by_alias(of_get_root_node(), alias);

	if (dnode && of_device_is_available(dnode)) {
		uint8_t mac[6];
		if (i2c_get_mac_addr(eeprom_name, has_eui64_mac(), mac) != 0) {
			pr_err("Error reading %s MAC address from EEPROM %s\n",
				alias, eeprom_name);
		} else {
			of_eth_register_ethaddr(dnode, mac);
			pr_debug("Set MAC address of %s from I2C EEPROM %s\n",
				 alias, eeprom_name);
		}
	}
}

static int arria10_plat_i2c_mac_init(void)
{
	if (!of_machine_is_compatible("arria10,plat"))
		return 0;

	/* Set GBE0 & GBE1 MAC addresses */
	register_i2c_ethaddr("ethernet0", EEPROM_MAC_GBE0);
	register_i2c_ethaddr("ethernet1", EEPROM_MAC_GBE1);

	return 0;
}
device_initcall(arria10_plat_i2c_mac_init);

#endif /* CONFIG_NET */


static int socfpga_console_init(void)
{
	if (!of_machine_is_compatible("arria10,plat"))
		return 0;
	return 0;
}
console_initcall(socfpga_console_init);

/* Use default compiled in env */
static int arria10_plat_env_init(void)
{
	defaultenv_append_directory(env);

	return 0;
}
device_initcall(arria10_plat_env_init);

#if defined(CONFIG_ARCH_SOCFPGA_XLOAD)
#include <mach/generic.h>

static struct socfpga_barebox_part boot_part[] = {
	{ .mmc_disk = "mmc0.boot0", },
	{ .mmc_disk = "mmc0.boot1", },
	{ /* sentinel */ }
};

static int arria10_plat_fpga_program(void)
{
	int ret=0, fd;
	uint8_t buf[512], part[128];
	struct cdev *fpga_manager;
	size_t counter = 0;
	struct stat st;
	uint16_t handoff;
	struct device_node *fpga_binary;
	const char *file_path;

	handoff = readl(ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);
	strcpy(part, "/dev/");
	strcat(part, boot_part[handoff & FPGA_BOOT_PARTITION].mmc_disk);
	pr_info("handoff 0x%08x %s\n", handoff, part);

	if (handoff & FPGA_PROGRAMMED) {
		goto outa;
	}

	pr_info("Mounting filesystem for FPGA image\n");
	/* Get device env is on and mount it */
	ret = mount(part, "fat", "/", NULL);
	if (ret) {
		pr_err("mount failed %s\n", strerror(-ret));
		goto out4;
	}

	pr_info("Opening files\n");
	fpga_manager = cdev_open("/dev/socfpga-fpga", O_RDWR);
	if (!fpga_manager) {
		ret = -ENODEV;
		pr_err("fpga manager open failed: %s\n", strerror(-ret));
		goto out3;
	}

	fpga_binary = of_find_node_by_path("/chosen/fpga-binary");
	if (!fpga_binary) {
		ret = -ENOENT;
		pr_err("Couldn't find fpga-binary node: %s\n", strerror(-ret));
		goto out2;
	}

	ret = of_property_read_string(fpga_binary, "file-path", &file_path);
	if (ret) {
		pr_err("No file-path in fpga-binary node: %s\n", strerror(-ret));
		goto out2;
	}

	fd = open(file_path, O_RDONLY);
	if (fd < 0) {
		ret = fd;
		pr_err("Failed to open fpga binary file: %s\n", strerror(-ret));
		goto out2;
	}
	fstat(fd, &st);

	pr_info("Programming FPGA\n");
	init_progression_bar(st.st_size);
	while (1) {
		ret = read(fd, buf, sizeof(buf));
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			pr_err("Failed read: %s\n", strerror(-ret));
			goto out1;
		} else {
			counter += ret;
			if (cdev_write(fpga_manager, buf, ret, 0, 0) != ret) {
				pr_err("Failed write: %s\n", ret < 0 ? strerror(-ret) : "short write");
				goto out1;
			}
			if (st.st_size && st.st_size != FILESIZE_MAX) {
				show_progress(counter);
			}
		}
	}
	writel((handoff | FPGA_PROGRAMMED), ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);

	pr_info("\nDone, total %zd.\n", counter);
	close(fd);
	cdev_close(fpga_manager);
	umount("/");
	restart_machine();
outa:
	arria10_ddr_calibration_sequence();
	return ret;

out1:
	close(fd);
out2:
	cdev_close(fpga_manager);
out3:
	umount("/");
out4:
	if (handoff & FPGA_PROGRAM_FAILED) {
		pr_err("Failed to program FPGA using both partitions!\n");
	} else {
		// Attempt to boot from alternate partition
		if (handoff & FPGA_BOOT_PARTITION)
			writel(((handoff & ~FPGA_BOOT_PARTITION) | FPGA_PROGRAM_FAILED), ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);
		else
			writel(((handoff | FPGA_BOOT_PARTITION) | FPGA_PROGRAM_FAILED), ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);

		pr_info("Restarting to program FPGA from alternate partition\n\n");
		restart_machine();
	}
	arria10_ddr_calibration_sequence();
	return ret;
}
crypto_initcall(arria10_plat_fpga_program);

static void arria10_plat_check_fpga_boot_partition(uint8_t partition)
{
	uint16_t handoff;
	handoff = readl(ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);
	if (handoff & FPGA_PROGRAM_FAILED) {
		pr_warning("FPGA programming failure occurred:\n"
				"    check FPGA version for compatibility issues!\n");
		// Clear failed bit
		writel((handoff & ~FPGA_PROGRAM_FAILED), ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);
	} else if (partition != (handoff & FPGA_BOOT_PARTITION)) {
		if (partition) {
			// Set the fpga boot partition to 1
			writel(((handoff | FPGA_BOOT_PARTITION) & ~FPGA_PROGRAMMED), ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);
		} else {
			// Set the fpga boot partition to 0
			writel(((handoff & ~FPGA_BOOT_PARTITION) & ~FPGA_PROGRAMMED), ARRIA10_SYSMGR_ROM_ISW_HANDOFF2);
		}
		pr_info("Restarting to program FPGA from alternate partition\n\n");
		restart_machine();
	}
}

static int arria10_plat_choose_boot(void)
{
	struct cdev *eeprom = cdev_open(BOOT_EEPROM, O_RDWR);
	uint8_t buf[2], check[2];
	int ret;

	if (!eeprom)
		return -ENODEV;

	ret = cdev_read(eeprom, buf, sizeof(buf), 0, 0);
	if (ret != sizeof(buf)) {
		pr_err("Boot status read failed: %s\n", ret < 0 ? strerror(-ret) : "short read");
		goto out;
	}

	if (++buf[1] > BOOT_MAXCOUNT) {
		pr_info("!!!!!!!!      Boot count exceeded  !!!!  Switching partitions     !!!!!!!!\n");
		buf[1] = 1;
		buf[0] = (buf[0] + 1) % (ARRAY_SIZE(boot_part)-1);
	}
	if (buf[0] >= (ARRAY_SIZE(boot_part)-1)) {
		pr_err("########  Partition number %u invalid! Resetting to 0\n", buf[0]);
		buf[0] = 0;
	}
	barebox_part = &boot_part[buf[0]];

	pr_info("========  Boot count %d/%d  ==  Booting from partition %s  ========\n",
		buf[1], BOOT_MAXCOUNT, barebox_part->mmc_disk);

	arria10_plat_check_fpga_boot_partition(buf[0]);

	ret = cdev_write(eeprom, buf, sizeof(buf), 0, 0);
	if (ret != sizeof(buf)) {
		pr_err("########  Boot status update failed: %s\n", ret < 0 ? strerror(-ret) : "short write");
		goto out;
	}
	ret = cdev_read(eeprom, check, sizeof(check), 0, 0);
	if (ret != sizeof(check)) {
		pr_err("########  Boot status read-back failed: %s\n", ret < 0 ? strerror(-ret) : "short read");
		goto out;
	}
	if (memcmp(buf, check, sizeof(buf))) {
		pr_err("########  Boot status update failed\n");
		ret = -EIO;
		goto out;
	}

	/* Success */
	ret = 0;

	out:
	cdev_close(eeprom);
	return ret;
}
late_initcall(arria10_plat_choose_boot);
#endif


/* Code for boot stuff when have scripts */
#if defined(CONFIG_COMMAND_SUPPORT)
#include <bootsource.h>

/* Insert boot source into device tree properties */
static int bootid_of_fixup(struct device_node *root, void *ctx)
{
	char id[4];
	struct device_node *chosen;

	chosen = of_find_node_by_path_from(root, "/chosen");
	if (!chosen) {
		pr_err("Couldn't find the /chosen node!\n");
		pr_err("Insert Star Wars chosen one joke here\n");
		return -ENOENT;
	}

	snprintf(id, sizeof(id), "%d", bootsource_get_instance());
	debug("Set /chosen/bootsource to '%s'\n", id);
	return of_set_property(chosen, "bootsource", id, strlen(id)+1, 1);
}

/* This sets $bootsource_instance to the software load number */
static int arria10_plat_bootinfo(void)
{
	struct cdev *eeprom = cdev_open(BOOT_EEPROM, O_RDWR);
	struct device_node *node;
	uint8_t buf[2];
	const char *devpath;
	char value[256];
	int ret, len;

	if (!eeprom)
		return -ENODEV;

	ret = cdev_read(eeprom, buf, sizeof(buf), 0, 0);
	cdev_close(eeprom);
	if (ret != sizeof(buf)) {
		pr_err("Boot status read failed: %s\n", ret < 0 ? strerror(-ret) : "short read");
		return ret < 0 ? ret : -EIO;
	}

	bootsource_set_instance(buf[0]);

	/* Adjust partition in env dt node.  We could also find the node via an
	 * alias.  We could also have env nodes for each partition and enable
	 * the one we want to use.  */
	node = of_find_node_by_path("/chosen/environment@0");
	if (!node) {
		pr_err("Couldn't find env device node\n");
		return -ENOENT;
	}
	ret = of_property_read_string(node, "device-path", &devpath);
	if (ret) {
		pr_err("No device-path in device node\n");
		return ret;
	}
	len = sprintf(value, "%s%cpartname:boot%d", devpath, '\0', bootsource_get_instance());
	of_set_property(node, "device-path", value, len+1, 0);
	debug("Set boot instance to %d and boot partition to '%s'\n",
		bootsource_get_instance(), strchr(value, '\0')+1);

	/* This will put the boot device into the kernel's device-tree */
	of_register_fixup(bootid_of_fixup, NULL);
	return 0;
}
crypto_initcall(arria10_plat_bootinfo);
#endif
