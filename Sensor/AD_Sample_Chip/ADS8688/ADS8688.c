
/**
 * @file    module_ads8688.c
 * @brief   ADS8688 Functions Defination
 * @author  Disen-Shaw <DisenShaw@gmail.com>
 * @date    2024-04-10
 * @version lts 1.0
 */

#include "module_ads8688.h"

static uint16_t s_cmd_write_reg(struct __dev_ads8688 *dev, uint8_t cmd);
static uint8_t s_prog_write_reg(struct __dev_ads8688 *dev, uint8_t addr, uint8_t value);
static void s_error_handler(struct __dev_ads8688 *dev);
static uint16_t s_cmd_no_op(struct __dev_ads8688 *dev);
static uint16_t s_cmd_standby(struct __dev_ads8688 *dev);
static uint16_t s_cmd_power_dn(struct __dev_ads8688 *dev);
static uint16_t s_cmd_reset(struct __dev_ads8688 *dev);
static uint16_t s_cmd_auto_rst(struct __dev_ads8688 *dev);
static uint16_t s_cmd_manual_channel(struct __dev_ads8688 *dev, uint8_t channel);
static uint8_t s_write_settings(struct __dev_ads8688 *dev, 
								const __ads8688_prog_reg_settings *settings);

/**
 * @defgroup ADS8688
 * @brief Global Interface
 * @{
 */

/**
 * @brief ADS8688 Initiailized
 *
 * @param[in] spi_cs_enable_hook SPI CS Enable FuncPtr
 * @param[in] spi_cs_disable_hook SPI CS Disable FuncPtr
 * @param[in] spi_rw_byte_hook SPI RW One Byte via SPI
 *
 * @retval
 *   \li 0: Initialized Error
 *   \li 1: Initialized Successfully
 */
uint8_t ads8688_init( 	s_dev_ads8688 *dev,
						void (*spi_cs_enable_hook)(void),
						void (*spi_cs_disable_hook)(void),
						uint8_t (*spi_rw_byte_hook)(uint8_t),
						void (*chip_reset_enable_hook)(void),
						void (*chip_reset_disable_hook)(void))
{
	if( !(spi_cs_enable_hook && spi_cs_disable_hook && spi_rw_byte_hook && dev) ) {
		return 0;
	} else {
		dev->spi_cs_enable 		= spi_cs_enable_hook;
		dev->spi_cs_disable 	= spi_cs_disable_hook;
		dev->spi_rw_byte		= spi_rw_byte_hook;
		dev->chip_reset_enable	= chip_reset_enable_hook;
		dev->chip_reset_disable = chip_reset_disable_hook;
		
		dev->cmd_no_op 			= s_cmd_no_op;
		dev->cmd_standby		= s_cmd_standby;
		dev->cmd_power_dn 		= s_cmd_power_dn;
		dev->cmd_reset			= s_cmd_reset;
		dev->cmd_auto_rst		= s_cmd_auto_rst;
		dev->cmd_manual_channel	= s_cmd_manual_channel;
		dev->write_settings		= s_write_settings;
		return 1;
	}
}

/**
 * @}
 */
 
/**
 * @defgroup ADS8688 Local Functions
 * @brief ADS8688 Internal Functions
 * @{
 */

/**
 * @brief 向 ADS8688 写入 PROG 配置信息
 *
 * @param __ads8688_prog_reg_settings settings 配置信息
 *
 * @retval
 *    \li 0: 写入异常, 需检查 <b>硬件连接</b> 或 <b>SPI 模式</b>
 *    \li 1: 写入成功
 */
static uint8_t s_write_settings(struct __dev_ads8688 *dev, 
								const __ads8688_prog_reg_settings *settings)
{
	uint8_t ret = 0x00;
	
	// 复位芯片
	dev->chip_reset_enable();
	for(int i=0; i<0xFFFF; i++);
	dev->chip_reset_disable();
	for(int i=0; i<0xFFFF; i++);
	
	// 写入 AUTO_SEQ_EN 寄存器
	ret = s_prog_write_reg(dev, PROG_ADDR_AUTO_SEQ_EN, settings->ch_seq_en_reg.raw);
	if(ret != settings->ch_seq_en_reg.raw) {
		// 写入异常, 复位芯片后退出
		s_error_handler(dev);
		return 0;
	} else {
		dev->settings.ch_seq_en_reg.raw = ret;
	}
	
	// 写入 Channel Power Down 寄存器
	ret = s_prog_write_reg(dev, PROG_ADDR_CHANNEL_PWR_DN, settings->ch_pwr_dn_reg.raw);
	if(ret != settings->ch_pwr_dn_reg.raw) {
		s_error_handler(dev);
		return 0;
	} else {
		dev->settings.ch_pwr_dn_reg.raw = ret;
	}
	
	// 写入 Feature Select 寄存器
	ret = s_prog_write_reg(dev, PROG_ADDR_FT_SEL, settings->ft_sel_reg.raw);
	if(ret != settings->ft_sel_reg.raw) {
		s_error_handler(dev);
		return 0;
	} else {
		dev->settings.ft_sel_reg.raw = ret;
	}
	
	// 写入 Channel x Input Range 寄存器
	for(int i=0; i<ADS8688_CHANNEL_COUNT; i++) {
		ret = s_prog_write_reg(dev, PROG_ADDR_INPUT_RANGE_CH0 + i, 
								settings->ch_input_range_reg[i].raw);
		if(ret != settings->ch_input_range_reg[i].raw) {
			s_error_handler(dev);
			return 0;
		} else {
			dev->settings.ch_input_range_reg[i].raw = ret;
		}
	}
	
	return 1;
}

/**
 * @brief 向 ADS8688 写入 NO_OP 指令
 *
 * @param none
 *
 * @retval DATA for Sample N
 */
static uint16_t s_cmd_no_op(struct __dev_ads8688 *dev)
{
	return s_cmd_write_reg(dev, CMD_NO_OP);
}

/**
 * @brief 向 ADS8688 写入 STANDBY 指令
 *
 * @param none
 *
 * @retval DATA for Sample N
 */
static uint16_t s_cmd_standby(struct __dev_ads8688 *dev)
{
	return s_cmd_write_reg(dev, CMD_STDBY);
}

/**
 * @brief 向 ADS8688 写入 PowerDown 指令
 *
 * @param none
 *
 * @retval DATA for Sample N
 */ 
static uint16_t s_cmd_power_dn(struct __dev_ads8688 *dev)
{
	return s_cmd_write_reg(dev, CMD_PWR_DN);
}

/**
 * @brief 向 ADS8688 写入 Reset 指令
 *
 * @param none
 *
 * @retval DATA for Sample N
 */
static uint16_t s_cmd_reset(struct __dev_ads8688 *dev)
{
	return s_cmd_write_reg(dev, CMD_RST);
}

/**
 * @brief 向 ADS8688 写入 Auto_Rst 指令
 *
 * @param none
 *
 * @retval DATA for Sample N
 */	
static uint16_t s_cmd_auto_rst(struct __dev_ads8688 *dev)
{
	return s_cmd_write_reg(dev, CMD_AUTO_RST);
}


/**
 * @brief 向 ADS8688 写入 通道选择指令 (Manual Channel Sel)
 *
 * @param[in] uint8_t channel 通道选择
 *
 * @retval DATA for Sample N
 */
static uint16_t s_cmd_manual_channel(struct __dev_ads8688 *dev, uint8_t channel)
{
	switch(channel) {
	case 0:
		return s_cmd_write_reg(dev, CMD_MAN_CH0);
	case 1:
		return s_cmd_write_reg(dev, CMD_MAN_CH1);
	case 2:
		return s_cmd_write_reg(dev, CMD_MAN_CH2);
	case 3:
		return s_cmd_write_reg(dev, CMD_MAN_CH3);
	case 4:
		return s_cmd_write_reg(dev, CMD_MAN_CH4);
	case 5:
		return s_cmd_write_reg(dev, CMD_MAN_CH5);
	case 6:
		return s_cmd_write_reg(dev, CMD_MAN_CH6);
	case 7:
		return s_cmd_write_reg(dev, CMD_MAN_CH7);
	case 0xFF:
		return s_cmd_write_reg(dev, CMD_MAN_AUX);
	default:
		// 异常通道不做操作
		break;
	}
	return 0x0000;
}

/**
 * @brief private 向 ADS8688 写指令
 *
 * @param cmd 指令
 *
 * @retval DATA for Sample N
 */
static uint16_t s_cmd_write_reg(struct __dev_ads8688 *dev, uint8_t cmd)
{
	uint16_t ret = 0x00;
	uint8_t rx_byte;
	dev->spi_cs_enable();
	dev->spi_rw_byte(cmd);
	dev->spi_rw_byte(0x00);
	rx_byte = dev->spi_rw_byte(0x00);
	ret |= (rx_byte << 8);
	rx_byte = dev->spi_rw_byte(0x00);
	ret |= (rx_byte & 0xFF);
	dev->spi_cs_disable();
	return ret;
}


/**
 * @brief 向 PROGRAM REG 中写入 指定值
 *
 * @param[in] addr 寄存器地址
 * @param[in] value 写入的值
 *
 * @retval
 *    写入芯片的值
 */ 
static uint8_t s_prog_write_reg(struct __dev_ads8688 *dev, uint8_t addr, uint8_t value)
{
	uint8_t ret = 0;
	dev->spi_cs_enable();
	dev->spi_rw_byte( (addr << 1) | 0x01 );
	dev->spi_rw_byte(value);
	ret = dev->spi_rw_byte(0x00);
	dev->spi_cs_disable();
	return ret;
}


/**
 * @brief 异常处理, 直接复位芯片
 *
 * @param none
 *
 * @retval none
 */
static void s_error_handler(struct __dev_ads8688 *dev)
{
	dev->chip_reset_enable();
	for(volatile int i=0; i<0xFFFF; i++);
	dev->chip_reset_disable();
	for(volatile int i=0; i<0xFFFF; i++);
}

/**
 * @}
 */



