/*
 * Configuration settings for the Arrow BALTO board
 *
 * Copyright (C) 2015 Arrow Europe
 * Copyright (C) 2015 Pierluigi Passaro <info@phoenixsoftware.it>
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */

#ifndef __BALTO_H
#define __BALTO_H

#define CONFIG_ARMV7		1	/* This is an ARM V7 CPU core */
#define CONFIG_CPU_RZA1		1
#define CONFIG_BOARD_LATE_INIT	1
#define CONFIG_MACH_TYPE MACH_TYPE_BALTO
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_USB
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_FAT
#define CONFIG_CMD_SF
#define CONFIG_CMD_I2C
#define CONFIG_PCA953X
#define CONFIG_CMD_PCA953X
#define CONFIG_CMD_PCA953X_INFO
#define CONFIG_CMD_DATE
#define CONFIG_DOS_PARTITION
#define CONFIG_MAC_PARTITION
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_SNTP
#define CONFIG_BOOTP_NTPSERVER
#define CONFIG_BOOTP_TIMEOFFSET

#define CONFIG_OF_LIBFDT
#define CONFIG_CMDLINE_EDITING
#define CONFIG_CMDLINE_TAG

#define CONFIG_SYS_NO_FLASH	/* no NOR on BALTO */

#ifndef _CONFIG_CMD_DEFAULT_H
# include <config_cmd_default.h>
#endif

#define CONFIG_BAUDRATE		115200
#define CONFIG_BOOTARGS		"console=ttySC3,115200"
#define CONFIG_IPADDR		192.168.84.170
#define CONFIG_SERVERIP		192.168.84.1
#define CONFIG_ETHADDR		42:20:00:08:41:70
#define CONFIG_BOOTDELAY	1
#define CONFIG_SYS_BAUDRATE_TABLE	{ CONFIG_BAUDRATE }

#define CONFIG_SYS_LONGHELP		/* undef to save memory	*/
#define CONFIG_SYS_PROMPT	"=> "	/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE	256	/* Boot Argument Buffer Size */
#define CONFIG_SYS_PBSIZE	256	/* Print Buffer Size */
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */
#define CONFIG_AUTO_COMPLETE

#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2      "> "

#define CONFIG_EXTRA_ENV_SETTINGS \
	"panel=-1\0" \
	"sdev=/dev/mmcblk0p1 rootwait rootdelay=3 init=/init\0" \
	"xdev=/dev/null rootflags=physaddr=0x18800000 init=/init\0" \
	"baseargs=console=ttySC3,115200\0" \
	"qd=qspi dual a4 d4 sdr\0" \
	"s0=echo Booting Linux to external RAM...\0" \
	"s1=sf probe 0; sf read 09800000 C0000 10000\0" \
	"s2=sf probe 0:1; sf read 09000000 100000 500000\0" \
	"s3=bootm start 0x09000000 - 0x09800000 ; bootm loados ; fdt memory 0x08000000 0x08000000\0" \
	"s_boot=run s0 s1 s2 s3 qd; set bootargs ${baseargs} root=${sdev} panel=${panel}; fdt chosen; bootm go\0" \
	"m0=echo Booting XIP Linux to external RAM...\0" \
	"m1=sf probe 0; sf read 09800000 C0000 10000; fdt addr 09800000\0" \
	"m2=fdt memory 0x08000000 0x08000000\0" \
	"m_boot=run m0 m1 m2 qd; set bootargs ${baseargs} root=${xdev} panel=${panel}; fdt chosen; bootx 18200000 09800000\0" \
	"x0=echo Booting XIP Linux to Internal RAM...\0" \
	"x1=sf probe 0; sf read 20500000 C0000 10000; fdt addr 20500000\0" \
	"x2=fdt memory 0x20000000 0x00A00000\0" \
	"x_boot=run x0 x1 x2 qd; set bootargs ${baseargs} root=${xdev} panel=${panel}; fdt chosen; bootx 18200000 20500000\0"

/*#define CONFIG_BOOTCOMMAND "run s_boot"*/
#define CONFIG_BOOTCOMMAND "run x_boot"

#define CONFIG_SYS_ARM_CACHE_WRITETHROUGH

/* Serial */
#define CONFIG_SCIF_CONSOLE
#define CONFIG_CONS_SCIF3
#define SCIF3_BASE			0xE8008800
#define CONFIG_SH_SCIF_CLK_FREQ CONFIG_SYS_CLK_FREQ

/* Memory */
/* u-boot relocated to top 256KB of ram */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_TEXT_BASE		0x18000000

#define USE_INTERNAL_RAM 1
#ifdef USE_INTERNAL_RAM
 #define CONFIG_SYS_SDRAM_BASE		0x20000000
 #define CONFIG_SYS_SDRAM_SIZE		(10 * 1024 * 1024)
#else
 #define CONFIG_SYS_SDRAM_BASE		0x08000000
 #define CONFIG_SYS_SDRAM_SIZE		(128 * 1024 * 1024)
#endif
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_SDRAM_BASE + CONFIG_SYS_SDRAM_SIZE - GENERATED_GBL_DATA_SIZE)

#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_SDRAM_BASE + 0x4000000)
#define CONFIG_SYS_MALLOC_LEN		(512 * 1024)
#define CONFIG_SYS_MONITOR_LEN		(128 * 1024)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 4*1024*1024)
#define	CONFIG_LOADADDR			CONFIG_SYS_SDRAM_BASE

#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_OFFSET	0x80000
#define CONFIG_ENV_SECT_SIZE	0x40000

#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE
#define CONFIG_ENV_ADDR		(CONFIG_SYS_FLASH_BASE + CONFIG_ENV_OFFSET)
#define CONFIG_ENV_OVERWRITE	1

#define __io

/* Spi-Flash configuration */
#define CONFIG_RZ_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_RZA1_BASE_QSPI0		0x3FEFA000
#define CONFIG_SPI_FLASH_BAR		/* For SPI Flash bigger than 16MB */

/* I2C configuration */
#define CONFIG_SH_RIIC
#define CONFIG_HARD_I2C
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_MAX_I2C_BUS		4
#define CONFIG_SYS_I2C_SPEED		100000 /* 100 kHz */
#define CONFIG_SYS_I2C_PCA953X_ADDR	0x20
#define CONFIG_SYS_I2C_MODULE		2
#define CONFIG_SH_I2C_BASE0		0xFCFEE000
#define CONFIG_SH_I2C_BASE1		0xFCFEE400
#define CONFIG_SH_I2C_BASE2		0xFCFEE800
#define CONFIG_SH_I2C_BASE3		0xFCFEEc00

/* RTC configuration */
#define CONFIG_RTC_RZA1
#define CONFIG_RTC_RZA1_BASE_ADDR	0xFCFF1000

/* Board Clock */
/*#define CONFIG_SYS_CLK_FREQ	66666666*/ /* P1 clock. */
#define CONFIG_SYS_CLK_FREQ	62500000 /* P1 clock. */
#define CONFIG_SYS_HZ		1000

/* Network interface */
#define CONFIG_SH_ETHER
#define CONFIG_SH_ETHER_USE_PORT	0
#define CONFIG_SH_ETHER_PHY_ADDR	0
#define CONFIG_SH_ETHER_PHY_MODE	PHY_INTERFACE_MODE_MII
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_BITBANGMII
#define CONFIG_BITBANGMII_MULTI

/* USB host controller */
#define CONFIG_USB_R8A66597_HCD
#define CONFIG_R8A66597_BASE_ADDR	0xE8010000
#define CONFIG_R8A66597_XTAL		0x0000	/* 48MHz */
#define CONFIG_R8A66597_ENDIAN		0x0000	/* little */

/*
 * Lowlevel configuration
 */
/* Disable WDT */
#define WTCSR_D		0xA518
#define WTCNT_D		0x5A00

/* Set clocks based on 13.3333MHz xtal */
#define FRQCR_D		0x1035	/* CPU= 300-400 MHz */

/* Enable all peripherals */
#define STBCR3_D	0x00000000
#define STBCR4_D	0x00000000
#define STBCR5_D	0x00000000
#define STBCR6_D	0x00000000
#define STBCR7_D	0x00000024
#define STBCR8_D	0x00000005
#define STBCR9_D	0x00000000
#define STBCR10_D	0x00000000
#define STBCR11_D	0x000000c0
#define STBCR12_D	0x000000f0

/* Port Control register */
/* Port1 Control register Reset */
#define PIBC1_D		0x0000
#define PBDC1_D		0x0000
#define PM1_D		0xffff
#define PMC1_D		0x0000
#define PIPC1_D		0x0000

/* Port1 Control register Set */
#define PBDC1_S		0x0030
#define PFC1_S		0x4000	/* SCL2, SDA2, ET_COL */
#define PFCE1_S		0x4000
#define PFCAE1_S	0x0000
#define PIPC1_S		0x4030
#define PMC1_S		0x4030
#define P1_S		0x0000
#define PM1_S		0xffff
#define PIBC1_S		0x0000

/* Port2 Control register Reset */
#define PIBC2_D		0x0000
#define PBDC2_D		0x0000
#define PM2_D		0xffff
#define PMC2_D		0x0000
#define PIPC2_D		0x0000

/* Port2 Control register Set */
#define PBDC2_S		0xf000
#define PFC2_S		0xfffd	/* ET_xxx, but P2_1 */
#define PFCE2_S		0xf000	/* SPBIO01_0, SPBIO11_0, SPBIO21_0, SPBIO31_0 */
#define PFCAE2_S	0x0000
#define PIPC2_S		0xfffd
#define PMC2_S		0xfffd
#define P2_S		0x0000
#define PM2_S		0xffff
#define PIBC2_S		0x0000

/* Port3 Control register Reset */
#define PIBC3_D		0x0000
#define PBDC3_D		0x0000
#define PM3_D		0xffff
#define PMC3_D		0x0000
#define PIPC3_D		0x0000

/* Port3 Control register Set */
#define PBDC3_S		0x0008
#define PFC3_S		0x0078
#define PFCE3_S		0x0000	/* ET_MDIO, ET_RXCCLK, ET_RXER ET_RXDV */
#define PFCAE3_S	0x0000
#define PIPC3_S		0x0078
#define PMC3_S		0x0078
#define P3_S		0x0000
#define PM3_S		0xffff
#define PIBC3_S		0x0000

/* Port4 Control register Reset */
#define PIBC4_D		0x0000
#define PBDC4_D		0x0000
#define PM4_D		0xffff
#define PMC4_D		0x0000
#define PIPC4_D		0x0000

/* Port4 Control register Set */
#define PBDC4_S		0x0000
#define PFC4_S		0x0000
#define PFCE4_S		0x0000
#define PFCAE4_S	0x0000
#define PIPC4_S		0x0000
#define PMC4_S		0x0000
#define P4_S		0x0000
#define PM4_S		0xffff
#define PIBC4_S		0x0000

/* Port5 Control register Reset */
#define PIBC5_D		0x0000
#define PBDC5_D		0x0000
#define PM5_D		0xffff
#define PMC5_D		0x0000
#define PIPC5_D		0x0000

/* Port5 Control register Set */
#define PBDC5_S		0x0000
#define PFC5_S		0x0300	/* CS2, ET_MDC */
#define PFCE5_S		0x0000
#define PFCAE5_S	0x0100
#define PIPC5_S		0x0300
#define PMC5_S		0x0300
#define P5_S		0x0000
#define PM5_S		0xffff
#define PIBC5_S		0x0000

/* Port6 Control register Reset */
#define PIBC6_D		0x0000
#define PBDC6_D		0x0000
#define PM6_D		0xffff
#define PMC6_D		0x0000
#define PIPC6_D		0x0000

/* Port6 Control register Set */
#define PBDC6_S		0xffff
#define PFC6_S		0x0000	/* D1 - D15 */
#define PFCE6_S		0x0000
#define PFCAE6_S	0x0000
#define PIPC6_S		0xffff
#define PMC6_S		0xffff
#define P6_S		0x0000
#define PM6_S		0xffff
#define PIBC6_S		0x0000

/* Port7 Control register Reset */
#define PIBC7_D		0x0000
#define PBDC7_D		0x0000
#define PM7_D		0xffff
#define PMC7_D		0x0000
#define PIPC7_D		0x0000

/* Port7 Control register Set */
#define PBDC7_S		0x0000
#define PFC7_S		0x0000	/* WE0/DQMLL, RD/WR, RD, CS0 */
#define PFCE7_S		0x0000	/* CS3, CKE, CAS, RAS, WE1/DQMLL, A7-A1 */
#define PFCAE7_S	0x0000
#define PIPC7_S		0xffff
#define PMC7_S		0xffff
#define P7_S		0x0000
#define PM7_S		0xffff
#define PIBC7_S		0x0000

/* Port8 Control register Reset */
#define PIBC8_D		0x0000
#define PBDC8_D		0x0000
#define PM8_D		0xffff
#define PMC8_D		0x0000
#define PIPC8_D		0x0000

/* Port8 Control register Set */
#define PBDC8_S		0x0000
#define PFC8_S		0x0000	/* RxD3, TxD3 */
#define PFCE8_S		0x0300	/* A15-A8 SDRAM */
#define PFCAE8_S	0x0300
#define PIPC8_S		0x03ff
#define PMC8_S		0x03ff
#define P8_S		0x0000
#define PM8_S		0xffff
#define PIBC8_S		0x0000

/* Port9 Control register Reset */
#define PIBC9_D		0x0000
#define PBDC9_D		0x0000
#define PM9_D		0xffff
#define PMC9_D		0x00fc
#define PIPC9_D		0x00fc

/* Port9 Control register Set */
#define PBDC9_S		0x0000
#define PFC9_S		0x00fc	/* P9_2-P9_7(SPBxxx) SPI Flash */
#define PFCE9_S		0x0000	/* SPBIO00_0, SPBIO10_0, SPBIO20_0, SPBIO20_0 */
#define PFCAE9_S	0x0000
#define PIPC9_S		0x00fc
#define PMC9_S		0x00fc
#define P9_S		0x0000
#define PM9_S		0xffff
#define PIBC9_S		0x0000

/* Port10 Control register Reset */
#define PIBC10_D	0x0000
#define PBDC10_D	0x0000
#define PM10_D		0xffff
#define PMC10_D		0x0000
#define PIPC10_D	0x0000

/* Port10 Control register Set */
#define PBDC10_S	0x0000
#define PFC10_S		0x0000
#define PFCE10_S	0x0000
#define PFCAE10_S	0x0000
#define PIPC10_S	0x0000
#define PMC10_S		0x0000
#define P10_S		0x0000
#define PM10_S		0xffff
#define PIBC10_S	0x0000

/* Port11 Control register Reset */
#define PIBC11_D	0x0000
#define PBDC11_D	0x0000
#define PM11_D		0xffff
#define PMC11_D		0x0000
#define PIPC11_D	0x0000

/* Port11 Control register Set */
#define PBDC11_S	0x0000
#define PFC11_S		0x0000
#define PFCE11_S	0x0000
#define PFCAE11_S	0x0000
#define PIPC11_S	0x0000
#define PMC11_S		0x0000
#define P11_S		0x0000
#define PM11_S		0xffff
#define PIBC11_S	0x0000

/* Configure NOR Flash (CS0, CS1) */
#define CS0WCR_D	0x00000b40
#define CS0BCR_D	0x10000C00
#define CS1WCR_D	0x00000b40
#define CS1BCR_D	0x10000C00

/* Configure SDRAM (CS2, CS3) */
#define CS2BCR_D	0x00004C00
#define CS2WCR_D	0x00000080
#define CS3BCR_D	0x00004C00
#define CS3WCR_D	0x00002492
#define SDCR_D		0x00120812
#define RTCOR_D		0xA55A0020
#define RTCSR_D		0xA55A0010

#endif	/* __BALTO_H */
