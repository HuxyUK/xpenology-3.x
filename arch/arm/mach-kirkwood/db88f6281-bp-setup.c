/*
 * arch/arm/mach-kirkwood/db88f6281-bp-setup.c
 *
 * Marvell DB-88F6281-BP Development Board Setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/ata_platform.h>
#include <linux/mv643xx_eth.h>
#include <linux/i2c.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/spi/orion_spi.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/kirkwood.h>
#include <plat/mvsdio.h>
#include "common.h"
#include "mpp.h"
#include "ctrlEnv/mvCtrlEnvLib.h"

static struct mtd_partition db88f6281_nand_parts[] = {
	{
		.name = "u-boot",
		.offset = 0,
		.size = SZ_1M
	}, {
		.name = "uImage",
		.offset = MTDPART_OFS_NXTBLK,
		.size = SZ_4M
	}, {
		.name = "root",
		.offset = MTDPART_OFS_NXTBLK,
		.size = MTDPART_SIZ_FULL
	},
};

static const struct flash_platform_data db88f6281_spi_slave_data = {
	.type		= "m25p128",
};

static struct spi_board_info __initdata db88f6281_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &db88f6281_spi_slave_data,
		.irq		= -1,
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
};

static struct mv643xx_eth_platform_data db88f6281_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};

static struct mv643xx_eth_platform_data db88f6281_ge01_data= {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(0),
};

static struct mv_sata_platform_data db88f6281_sata_data = {
	.n_ports	= 2,
};

static struct mvsdio_platform_data db88f6281_mvsdio_data = {
	.gpio_write_protect	= 37,
	.gpio_card_detect	= 38,
};

static struct i2c_board_info i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("cs42l51", 0x4a),
	},
};

static void __init db88f6281_init(void)
{
	u32 dev, rev;

	/*
	 * Init board id (Marvell-internal) and MPPs
	 */
	mvBoardIdSet(DB_88F6282A_BP_ID);
	mvBoardEnvInit();
	mvCtrlEnvInit();

	/*
	 * Basic setup. Needs to be called early.
	 */
	kirkwood_init();

	kirkwood_nand_init(ARRAY_AND_SIZE(db88f6281_nand_parts), 25);
	kirkwood_ehci_init();
	kirkwood_ge00_init(&db88f6281_ge00_data);
	kirkwood_ge01_init(&db88f6281_ge01_data);
	kirkwood_sata_init(&db88f6281_sata_data);

	spi_register_board_info(db88f6281_spi_slave_info,
				ARRAY_SIZE(db88f6281_spi_slave_info));
	kirkwood_spi_init();

	kirkwood_uart0_init();
	kirkwood_sdio_init(&db88f6281_mvsdio_data);

	kirkwood_i2c_init();
	i2c_register_board_info(0, i2c_board_info, ARRAY_SIZE(i2c_board_info));
	kirkwood_audio_init();

	kirkwood_pcie_id(&dev, &rev);
	if (dev == MV88F6282_DEV_ID)
		kirkwood_hwmon_init();
}

static int __init db88f6281_pci_init(void)
{
	if (machine_is_db88f6281_bp()) {
		u32 dev, rev;

		kirkwood_pcie_id(&dev, &rev);
		if (dev == MV88F6282_DEV_ID)
			kirkwood_pcie_init(KW_PCIE1 | KW_PCIE0);
		else
			kirkwood_pcie_init(KW_PCIE0);
	}
	return 0;
}
subsys_initcall(db88f6281_pci_init);

/* In order to print DB-88F6282-BP board name,
 * MACHINE_START should be created.
 */
MACHINE_START(DB88F6282_BP, "Marvell DB-88F6282-BP Development Board")
	.atag_offset	= 0x100,
	.init_machine	= db88f6281_init,
	.map_io		= kirkwood_map_io,
	.init_early	= kirkwood_init_early,
	.init_irq	= kirkwood_init_irq,
	.timer		= &kirkwood_timer,
MACHINE_END

MACHINE_START(DB88F6281_BP, "Marvell DB-88F6281-BP Development Board")
	.atag_offset	= 0x100,
	.init_machine	= db88f6281_init,
	.map_io		= kirkwood_map_io,
	.init_early	= kirkwood_init_early,
	.init_irq	= kirkwood_init_irq,
	.timer		= &kirkwood_timer,
MACHINE_END
