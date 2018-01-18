obj-$(CONFIG_FIRMWARE_ALTERA_SERIAL) += altera_serial.o
ifdef CONFIG_ARCH_SOCFPGA_ARRIA10
obj-$(CONFIG_FIRMWARE_ALTERA_SOCFPGA) += arria10_socfpga.o
else
obj-$(CONFIG_FIRMWARE_ALTERA_SOCFPGA) += cyclone5_socfpga.o
endif
