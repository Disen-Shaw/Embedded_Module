
/**
 *******************************************************************************
 * @file    module_mcp3464.h
 * @brief   mcp346x 设备驱动相关定义
 * @author  Disen-Shaw <DisenShaw@gmail.com>
 * @date    2024-01-30
 * @version V1.0
 *******************************************************************************
 */

#ifndef __MODULE_MCP3464_H__
#define __MODULE_MCP3464_H__

#include "gd32f10x.h"

#define BYTE_DEF_START
#define BYTE_DEF_END


/**
 * @brief MCP346x 指令定义
 * @{
 */

#define MCP346x_DEVICE_ADDRESS			(0x01)
#define MCP346x_SPI_ADR   				(MCP346x_DEVICE_ADDRESS << 6U)

/**
 * @brief 设备类型
 */
#define MCP3461_DEVICE_TYPE				((uint16_t)0x0800U)					//!< MCP3461 设备ID
#define MCP3462_DEVICE_TYPE				((uint16_t)0x0900U)					//!< MCP3462 设备ID
#define MCP3464_DEVICE_TYPE				((uint16_t)0x0B00U)					//!< MCP3464 设备ID

/**
 * @brief Fast Command
 */
#define MCP346x_CMD_CONVERSION			(MCP346x_SPI_ADR | 0b101000)
#define MCP346x_CMD_STANDBY				(MCP346x_SPI_ADR | 0b101100)		//!< 
#define MCP346x_CMD_SHUTDOWN			(MCP346x_SPI_ADR | 0b110000)		//!< 关闭模拟采样
#define MCP346x_CMD_FULL_SHUTDOWN		(MCP346x_SPI_ADR | 0b110100)		//!< 完全关闭
#define MCP346x_CMD_RESET         		(MCP346x_SPI_ADR | 0b111000)		//!< 重置
#define MCP346x_CMD_SREAD         		(MCP346x_SPI_ADR | 0b01)			//!< 静态读取
#define MCP346x_CMD_IREAD         		(MCP346x_SPI_ADR | 0b11)			//!< 增量式读取
#define MCP346x_CMD_IWRITE        		(MCP346x_SPI_ADR | 0b10)			//!< 增量式写入

/**
 * @brief Register Address Defination
 */
#define MCP346x_ADR_ADCDATA				(MCP346x_SPI_ADR | (0x0 << 2)) 	//! ADC Data Reg
#define MCP346x_ADR_CONFIG0				(MCP346x_SPI_ADR | (0x1 << 2))	//! Config0 Reg
#define MCP346x_ADR_CONFIG1				(MCP346x_SPI_ADR | (0x2 << 2))	//! Config1 Reg
#define MCP346x_ADR_CONFIG2				(MCP346x_SPI_ADR | (0x3 << 2))	//! Config1 Reg
#define MCP346x_ADR_CONFIG3				(MCP346x_SPI_ADR | (0x4 << 2))	//! Config1 Reg
#define MCP346x_ADR_IRQ					(MCP346x_SPI_ADR | (0x5 << 2))	//! IRQ Reg
#define MCP346x_ADR_MUX					(MCP346x_SPI_ADR | (0x6 << 2))	//! MUX Reg
#define MCP346x_ADR_SCAN				(MCP346x_SPI_ADR | (0x7 << 2))	//! SCAN Reg
#define MCP346x_ADR_TIMER				(MCP346x_SPI_ADR | (0x8 << 2))	//! TIMER Reg
#define MCP346x_ADR_OFFSET				(MCP346x_SPI_ADR | (0x9 << 2))	//! OFFSET Reg
#define MCP346x_ADR_GAIN				(MCP346x_SPI_ADR | (0xA << 2))	//! GAIN Reg
#define MCP346x_ADR_RESERVED1			(MCP346x_SPI_ADR | (0xB << 2))	//! Reserved1 Reg
#define MCP346x_ADR_RESERVED2			(MCP346x_SPI_ADR | (0xC << 2))	//! Reserved2 Reg
#define MCP346x_ADR_LOCK				(MCP346x_SPI_ADR | (0xD << 2))	//! Lock Reg
#define MCP346x_ADR_RESERVED3			(MCP346x_SPI_ADR | (0xE << 2))	//! Reserved3 Reg
#define MCP346x_ADR_CRCCFG				(MCP346x_SPI_ADR | (0xF << 2))	//! CRC_CFG Reg

/**
 * @}
 */

/**
 * @brief MCP346x 寄存器位定义
 * @{
 */

/**
 * @brief CONFIG0[1:0] ADC 模式
 */
#define MCP346x_ADC_MODE_CONVERSION			((uint8_t)0b11U)	//!< 转化模式
#define MCP346x_ADC_MODE_STANDBY			((uint8_t)0b10U)	//!< Standby 模式
#define MCP346x_ADC_MODE_SHUTDOWN			((uint8_t)0b00U)	//!< 关断模式 (default)

/**
 * @brief CONFIG0[3:2] ADC 偏置电流选择
 */
#define MCP346x_CS_SEL_15uA					((uint8_t)0b11U)	//!< 15uA 偏置源电流
#define MCP346x_CS_SEL_3p7uA				((uint8_t)0b10U)	//!< 3.7uA 偏置源电流
#define MCP346x_CS_SEL_0p9uA				((uint8_t)0b01U)	//!< 0.9uA 偏置源电流
#define MCP346x_CS_SEL_NONE					((uint8_t)0b00U)	//!< 无偏置电流源 (default)

/**
 * @brief CONFIG[5:4] ADC 时钟选择
 */
#define MCP346x_CLK_SEL_INTERNEL_OUTPUT		((uint8_t)0b11U)	//!< 内部时钟, 输出
#define MCP346x_CLK_SEL_INTERNEL			((uint8_t)0b10U)	//!< 内部时钟, 不输出
#define MCP346x_CLK_SEL_EXTERNAL			((uint8_t)0b00U)	//!< 外部时钟 (default)

/**
 * @brief CONFIG1[5:2] OSR 配置
 */
#define MCP346x_OSR_98304					((uint8_t)0b1111U)	//!< OSR: 98304
#define MCP346x_OSR_81920					((uint8_t)0b1110U)	//!< OSR: 81920
#define MCP346x_OSR_49152					((uint8_t)0b1101U)	//!< OSR: 49152
#define MCP346x_OSR_40960					((uint8_t)0b1100U)	//!< OSR: 40960
#define MCP346x_OSR_24576					((uint8_t)0b1011U)	//!< OSR: 24576
#define MCP346x_OSR_20480					((uint8_t)0b1010U)	//!< OSR: 20480
#define MCP346x_OSR_16384					((uint8_t)0b1001U)	//!< OSR: 16384
#define MCP346x_OSR_8192					((uint8_t)0b1000U)	//!< OSR: 8192
#define MCP346x_OSR_4096					((uint8_t)0b0111U)	//!< OSR: 4096
#define MCP346x_OSR_2048					((uint8_t)0b0110U)	//!< OSR: 2048
#define MCP346x_OSR_1024					((uint8_t)0b0101U)	//!< OSR: 1024
#define MCP346x_OSR_512						((uint8_t)0b0100U)	//!< OSR: 512
#define MCP346x_OSR_256						((uint8_t)0b0011U)	//!< OSR: 256 (default)
#define MCP346x_OSR_128						((uint8_t)0b0010U)	//!< OSR: 128
#define MCP346x_OSR_64						((uint8_t)0b0001U)	//!< OSR: 64
#define MCP346x_OSR_32						((uint8_t)0b0000U)	//!< OSR: 32

/**
 * @brief CONFIG1[7:6] 时钟分频 
 */
#define MCP346x_PRESCALER_8					((uint8_t)0b11U)	//!< AMCLK = MCLK/8
#define MCP346x_PRESCALER_4					((uint8_t)0b10U)	//!< AMCLK = MCLK/4
#define MCP346x_PRESCALER_2					((uint8_t)0b01U)	//!< AMCLK = MCLK/2
#define MCP346x_PRESCALER_NONE				((uint8_t)0b00U)	//!< AMCLK = MCLK (default)


/**
 * @brief CONFIG2[7:6] BOOST 偏置电流增益选择
 */
#define MCP346x_BOOST_x2					((uint8_t)0b11U)	//!< 偏置电流 x2
#define MCP346x_BOOST_x1					((uint8_t)0b10U)	//!< 偏置电流 x1 (default)
#define MCP346x_BOOST_x0p66					((uint8_t)0b01U)	//!< 偏置电流 x0.66
#define MCP346x_BOOST_x0p5					((uint8_t)0b00U)	//!< 偏置电流 x0.5

/**
 * @brief CONFIG2[5:4] GAIN 增益选择
 */
#define MCP346x_GAIN_x64					((uint8_t)0b111U)	//!< 增益为 x64, x16 analog, x4 digital
#define MCP346x_GAIN_x32					((uint8_t)0b110U)	//!< 增益为 x32, x16 analog, x2 digital
#define MCP346x_GAIN_x16					((uint8_t)0b101U)	//!< 增益为 x16
#define MCP346x_GAIN_x8						((uint8_t)0b100U)	//!< 增益为 x8
#define MCP346x_GAIN_x4						((uint8_t)0b011U)	//!< 增益为 x4
#define MCP346x_GAIN_x2						((uint8_t)0b010U)	//!< 增益为 x2
#define MCP346x_GAIN_x1						((uint8_t)0b001U)	//!< 增益为 x1 (无增益) (default)
#define MCP346x_GAIN_x0p33					((uint8_t)0b000U)	//!< 增益为 x1/3

/**
 * @brief CONFIG2[2] 自我调零
 */
#define MCP346x_AZ_MUX_ENABLE				((uint8_t)0b1U)		//!< 自动调零开启
#define MCP346x_AZ_MUX_DISABLE				((uint8_t)0b0U)		//!< 自动调零关闭 (default)

/**
 * @brief CONFIG3[0] ADC 操作模式选择开关
 */
#define MCP346x_EN_GAINAL_ENABLE			((uint8_t)0b1U)		//!< 开启
#define MCP346x_EN_GAINAL_DISABLE			((uint8_t)0b0U)		//!< 关闭 (default)

/**
 * @brief CONFIG3[1] 数字偏移校准开关
 */
#define MCP346x_EN_OFFCAL_ENABLE			((uint8_t)0b1U)		//!< 开启数字校准开关
#define MCP346x_EN_OFFCAL_DISABLE			((uint8_t)0b0U)		//!< 关闭数字校准开关 (default)

/**
 * @brief CONFIG3[2] 读操作时CRC校验开关
 */
#define MCP346x_EN_CRCCOM_ENABLE			((uint8_t)0b1U)		//!< 开启CRC校验开关
#define MCP346x_EN_CRCCOM_DISABLE			((uint8_t)0b0U)		//!< 关闭CRC校验开关 (default)

/**
 * @brief CONFIG3[3] CRC 检验码格式
 */
#define MCP346x_CRC_FMT_32BIT				((uint8_t)0b1U)		//!< 32位CRC, 16bit CRC + 16Bit zero
#define MCP346x_CRC_FMT_16BIT				((uint8_t)0b0U)		//!< 16位CRC (default)

/**
 * @brief CONFIG3[5:4] ADC输出数据格式选择
 */
#define MCP346x_DATA_FMT_ID_SGN_DATA		((uint8_t)0b11U)	//!< 32位 CHID[3:0]-SGN扩展[12:0]-16bit ADC 数据, 可超量程
#define MCP346x_DATA_FMT_SGN_DATA			((uint8_t)0b10U)	//!< 32位 SGN[16bits]-16bit ADC 数据, 可超量程
#define MCP346x_DATA_FMT_SGN_DATA_ZERO		((uint8_t)0b01U)	//!< 32位 SGN[16bits]-16bit ADC 数据， 不可超量程
#define MCP346x_DATA_FMT_DATA				((uint8_t)0b00U)	//!< 16位 ADC 数据, 不可超量程 (default)

/**
 * @brief CONFIG3[7:6] ADC 转化模式选择
 */
#define MCP346x_CONV_MODE_CONTINUOUS		((uint8_t)0b11U)	//!< 连续转换
#define MCP346x_CONV_MODE_STANDBY			((uint8_t)0b10U)	//!< 单次转换, 转换之后将 CONFIG0 的 ADC_MODE[1:0] 设置为 Standby 
#define MCP346x_CONV_MODE_ONESHUT			((uint8_t)0b00U)	//!< 单次转换 (default) 

/**
 * @brief IRQ[0] 转化完成中断开关
 */
#define MCP346x_EN_STP_ENABLE				((uint8_t)0b1U)		//!< 使能转化完成中断 (default)
#define MCP346x_EN_STP_DISABLE				((uint8_t)0b0U)		//!< 不使能转化完成中断

/**
 * @brief IRQ[1] FASTCMD 开关
 */
#define MCP346x_EN_FASTCMD_ENABLE			((uint8_t)0b1U)		//!< 使能 FastCMD (default)
#define MCP346x_EN_FASTCMD_DISABLE			((uint8_t)0b0U)		//!< 关闭 FastCMD

/**
 * @brief IRQ[3:2] IRQ 模式
 */
#define MCP346x_IRQ_MODE_MDAT_DEF_H			((uint8_t)0b11U)	//!< MDAT 输出, 只有 POR 和 CRC 异常中断可出现在该引脚, 非活动为高电平
#define MCP346x_IRQ_MODE_MDAT_DEF_Z			((uint8_t)0b10U)	//!< MDAT 输出, 只有 POR 和 CRC 异常中断可出现在该引脚, 非活动为高阻态, 需要上拉电阻
#define MCP346x_IRQ_MODE_IRQ_DEF_HIGH		((uint8_t)0b01U)	//!< IRQ 输出, 所有异常都可以输出到引脚, 非活动状态为高电平
#define MCP346x_IRQ_MODE_IRQ_DEF_Z			((uint8_t)0b00U)	//!< IRQ 输出, 所有异常都可以输出到引脚, 非活动状态为高阻态, 需要上拉电阻

/**
 * @brief MUX 寄存器配置
 */
#define MCP346x_MUX_OFFSET					((uint8_t)0x88U)
#define MCP346x_MUX_VCM						((uint8_t)0xF8U)
#define MCP346x_MUX_AVDD					((uint8_t)0x98U)
#define MCP346x_MUX_TEMP					((uint8_t)0xDEU)
#define MCP346x_MUX_DIFFD					((uint8_t)0x67U)
#define MCP346x_MUX_DIFFC					((uint8_t)0x45U)
#define MCP346x_MUX_DIFFB					((uint8_t)0x23U)
#define MCP346x_MUX_DIFFA					((uint8_t)0x01U)
#define MCP346x_MUX_CH7						((uint8_t)0x78U)
#define MCP346x_MUX_CH6						((uint8_t)0x68U)
#define MCP346x_MUX_CH5						((uint8_t)0x58U)
#define MCP346x_MUX_CH4						((uint8_t)0x48U)
#define MCP346x_MUX_CH3						((uint8_t)0x38U)
#define MCP346x_MUX_CH2						((uint8_t)0x28U)
#define MCP346x_MUX_CH1						((uint8_t)0x18U)
#define MCP346x_MUX_CH0						((uint8_t)0x08U)

/**
 * @brief SCAN[23:21] 扫描间隔
 */
#define MCP346x_SCAN_DLY_512				((uint8_t)0b111U)		//!< 扫描间隔 512 
#define MCP346x_SCAN_DLY_256				((uint8_t)0b110U)		//!< 扫描间隔 256
#define MCP346x_SCAN_DLY_128				((uint8_t)0b101U)		//!< 扫描间隔 128
#define MCP346x_SCAN_DLY_64					((uint8_t)0b100U)		//!< 扫描间隔 64
#define MCP346x_SCAN_DLY_32					((uint8_t)0b011U)		//!< 扫描间隔 32
#define MCP346x_SCAN_DLY_16					((uint8_t)0b010U)		//!< 扫描间隔 16
#define MCP346x_SCAN_DLY_8					((uint8_t)0b001U)		//!< 扫描间隔 8
#define MCP346x_SCAN_DLY_NONE				((uint8_t)0b000U)		//!< 扫描间隔 None


/**
 * @brief SCAN[15:0] 通道选择
 *
 * @note 字节高低位做了调整
 */
#define MCP346x_SCAN_CHANNEL_CH0			((uint16_t)0x0100U)
#define MCP346x_SCAN_CHANNEL_CH1			((uint16_t)0x0200U)
#define MCP346x_SCAN_CHANNEL_CH2			((uint16_t)0x0400U)
#define MCP346x_SCAN_CHANNEL_CH3			((uint16_t)0x0800U)
#define MCP346x_SCAN_CHANNEL_CH4			((uint16_t)0x1000U)
#define MCP346x_SCAN_CHANNEL_CH5			((uint16_t)0x2000U)
#define MCP346x_SCAN_CHANNEL_CH6			((uint16_t)0x4000U)
#define MCP346x_SCAN_CHANNEL_CH7			((uint16_t)0x8000U)
#define MCP346x_SCAN_CHANNEL_DIFA			((uint16_t)0x0001U)
#define MCP346x_SCAN_CHANNEL_DIFB			((uint16_t)0x0002U)
#define MCP346x_SCAN_CHANNEL_DIFC			((uint16_t)0x0004U)
#define MCP346x_SCAN_CHANNEL_DIFD			((uint16_t)0x0008U)
#define MCP346x_SCAN_CHANNEL_TEMP			((uint16_t)0x0010U)
#define MCP346x_SCAN_CHANNEL_AVDD			((uint16_t)0x0020U)
#define MCP346x_SCAN_CHANNEL_VCM			((uint16_t)0x0040U)
#define MCP346x_SCAN_CHANNEL_OFFSET			((uint16_t)0x0080U)

/**
 * @brief Config0 寄存器
 */
typedef union {
	struct __attribute__((packed)) {
		BYTE_DEF_START
		uint8_t adc_mode 				: 2;			//!< ADC 模式
		uint8_t cs_sel 					: 2;			//!< 偏置电流选择
		uint8_t clk_sel					: 2;			//!< 时钟选择
		uint8_t shutdown				: 2;			//!< 默认 11
		BYTE_DEF_START
	};
	uint8_t raw;
} __REG_CONFIG0;

/**
 * @brief CONFIG1 寄存器
 */
typedef union {
	struct __attribute__((packed)) {
		BYTE_DEF_START
		uint8_t 						: 2;			//!< Reserved Bits set as 2'b00
		uint8_t osr						: 4;			//!< OSR
		uint8_t prescalers				: 2;			//!< prescalers
		BYTE_DEF_END
	};
	uint8_t raw;
} __REG_CONFIG1;

/**
 * @brief Config2 寄存器
 */
typedef union {	
	struct __attribute__((packed)) {
		BYTE_DEF_START
		uint8_t reserved				: 2;			//!< Reserved Bits Set as 2'b11
		uint8_t az_mux					: 1;			//!< Auto Zeroing Mux Setting
		uint8_t gain					: 3;			//!< ADC 采样增益
		uint8_t boost					: 2;			//!< 偏置电流增益;
		BYTE_DEF_START
	};
	uint8_t raw;
} __REG_CONFIG2;

/**
 * @brief config3 寄存器
 */
typedef union {
	struct __attribute__((packed)) {
		BYTE_DEF_START
		uint8_t en_gaincal				: 1;			//!< ADC 模式选择开关
		uint8_t en_offsetcal			: 1;			//!< 数字偏移校准开关
		uint8_t en_crccom				: 1;			//!< CRC 校验开关
		uint8_t crc_fmt					: 1;			//!< CRC 格式
		uint8_t data_fmt				: 2;			//!< 数据格式选择
		uint8_t conv_mode				: 2;			//!< 转化模式选择
		BYTE_DEF_START
	};
	uint8_t raw;
} __REG_CONFIG3;

/**
 * @brief IRQ 寄存器
 */
typedef union __attribute__((packed)) {
	struct __attribute__((packed)){
		uint8_t en_stp 					: 1;			//!< 中断输出使能开关
		uint8_t en_fastcmd				: 1;			//!< 快速指令开关
		uint8_t irq_mode				: 2;			//!< IRQ 模式
		uint8_t 						: 4;			//!< IRQ 寄存器 高四位只读
	};
	uint8_t raw;
} __REG_IRQ;

/**
 * @brief MUX 寄存器
 */
typedef uint8_t __REG_MUX;

/**
 * @brief SCAN 寄存器
 */
typedef union {

	struct __attribute__((packed)) {
		uint8_t 						: 5;
		uint8_t delay 					: 3;
		uint16_t channel				: 16;
	};

	uint8_t raw[3];
} __REG_SCAN;

/**
 * @brief TIMER 寄存器 
 */
typedef union {
	struct __attribute__((packed)){
		uint8_t reg_low;								//!< Low Byte default 0
		uint8_t reg_mid;								//!< Middle Byte default 0
		uint8_t reg_high;								//!< High Byte default 0
	};
	uint8_t raw[3];
} __REG_TIMER;

/**
 * @brief 偏移校准寄存器
 *
 * @note: OFFSETCAL(V) = Vref * (OFFSETCAL[23:8) / (32768 * GAIN)
 */
typedef union {
	struct __attribute__((packed)){
		uint8_t reg_low;								//!< Low Byte
		uint8_t reg_mid;								//!< Middle Byte
		uint8_t reg_high;								//!< High Byte
	};
	uint8_t raw[3];
} __REG_OFFSETCAL;

/**
 * @brief 增益校准寄存器
 *
 * @note GAINCAL(V/V) = (GAINVAL[23:8] Unsigned Decimal Code)/32768
 */
typedef union {
	struct __attribute__((packed)){
		uint8_t reg_low;								//!< Low Byte
		uint8_t reg_mid;								//!< Middle Byte
		uint8_t reg_high;								//!< High Byte
	};
	uint8_t raw[3];
} __REG_GAINCAL;

typedef uint8_t __REG_LOCK;

/**
 * @brief CRCCFG 寄存器配置
 */
typedef union {
	struct {
		uint8_t reg_low;
		uint8_t reg_high;
	};
	uint16_t raw;
} __REG_CRCCFG;

/**
 * @}
 */

/**
 * @brief MCP346x 配置
 */
typedef union {
	struct __attribute__((packed)) {
		__REG_CONFIG0 config0;							//!< CONFIG0 寄存器		
		__REG_CONFIG1 config1;							//!< CONFIG1 寄存器
		__REG_CONFIG2 config2;							//!< CONFIG2 寄存器
		__REG_CONFIG3 config3;							//!< CONFIG3 寄存器
		__REG_IRQ irq;									//!< IRQ 寄存器
		__REG_MUX mux;									//!< Mux 寄存器
		__REG_SCAN scan;								//!< SCAN 寄存器
		__REG_TIMER timer;								//!< TIMER 寄存器
	};
	uint8_t raw[12];
} MCP346x_Settings;

/**
 * @brief MCP346x 寄存器状态
 */
typedef union {
	struct __attribute__((packed)) {
		__REG_CONFIG0 config0;
		__REG_CONFIG1 config1;
		__REG_CONFIG2 config2;
		__REG_CONFIG3 config3;
		__REG_IRQ irq;
		__REG_MUX mux;
		__REG_SCAN scan;
		__REG_TIMER timer;
		__REG_OFFSETCAL offsetcal;
		__REG_GAINCAL gaincal;
		uint8_t reserved1[3];
		uint8_t reserved2;
		__REG_LOCK lock;
		uint16_t device_id;
		__REG_CRCCFG crccfg;
	};
	uint8_t raw[27];
} MCP346x_Status;

/**
 * @brief 通信返回值
 */
typedef union {
	struct __attribute__((packed)) {
		uint8_t por_status					: 1;
		uint8_t crccfg_status				: 1;
		uint8_t dr_status					: 1;
		uint8_t 							: 1;		
		uint8_t addr 						: 2;
		uint8_t 							: 2;
	};
	uint8_t raw;
} __STATUS;

/**
 * @brief MCP346x 设备类型
 */
typedef struct __MCP346x {
	/**
	 * @note SPI 接口
	 */
	void (*spi_cs_low)(void);									//!< 片选线拉低
	void (*spi_cs_high)(void);									//!< 片选线拉高
	uint8_t (*spi_rw_byte)(uint8_t);							//!< spi 读写单个字节
	void (*spi_write_data)(const uint8_t *, size_t len);		//!< 发送 多个字节
	void (*spi_read_data)(uint8_t *, size_t len);				//!< 读取 多个字节
	
	/**
	 * @note 功能接口
	 */
	void (*fastcmd)(struct __MCP346x *, uint8_t);				//!< 发送 fastcmd 指令
	void (*start_convert)(struct __MCP346x *);					//!< 发送 Start Convert 快速指令
	void (*write_settings)(struct __MCP346x *);					//!< 发送 寄存器配置
	void (*read_config_status)(struct __MCP346x *);				//!< 读取设备寄存器信息
	void (*read_adc_data)(struct __MCP346x *);					//!< 读取 ADC 数据
	void (*show_settings)(struct __MCP346x *);					//!< 打印配置信息
	void (*show_config_status)(struct __MCP346x *);				//!< 打印设备寄存器信息
	void (*show_dev_status)(struct __MCP346x *);				//!< 打印 MCP346x 状态信息
	void (*set_signal_channel)(struct __MCP346x *, uint8_t);	//!< 设置单端通道
	void (*set_diff_channel)(struct __MCP346x *, uint8_t);		//!< 设置差分通道
	void (*set_conv_mode)(struct __MCP346x *, uint8_t);			//!< 设置转化模式
	uint16_t (*parse_adc_data)(struct __MCP346x *);				//!< 解析 ADC 数据
	void (*irq_handler)(struct __MCP346x *);					//!< 中断处理函数
	
	/**
	 * @brief 信息
	 */
	MCP346x_Settings settings;									//!< MCP346x 配置信息
	MCP346x_Status config_status;								//!< MCP346x 当前状态
	__STATUS dev_status;										//!< MCP346x 操作返回状态
	uint8_t adc_data[4];										//!< ADC 数据
} MCP346x;

uint8_t mcp346x_init(
	MCP346x *_mcp346x_inst, 									//!< MCP346x 实例
	void (*_cs_low_func)(void),									//!< SPI 片选拉低使能函数接口
	void (*_cs_high_func)(void),								//!< SPI 片选拉高失能函数接口
	uint8_t (*_spi_rw_func)(uint8_t),							//!< SPI 字节读写函数接口
	void (*_spi_write_data_func)(const uint8_t *, size_t len),	//!< SPI 写数据函数接口
	void (*_spi_read_data_func)(uint8_t *, size_t len));
void mcp346x_set_configration(MCP346x *_mcp346x_inst, const MCP346x_Settings *_settings);
void mcp346x_show_config_default(void);

#endif /* __MODULE_MCP3464_H__ */
