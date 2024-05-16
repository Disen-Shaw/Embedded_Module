
/**
 *******************************************************************************
 * @file    module_mcp3464.C
 * @brief   mcp346x 设备驱动
 * @author  Disen-Shaw <DisenShaw@gmail.com>
 * @date    2024-01-30
 * @version V1.0
 *******************************************************************************
 */

#include "generic_log.h"
#include "generic_gpio.h"

#include "module_mcp346x.h"

#include "bsp_systick.h"

#pragma diag_suppress 177
#pragma diag_suppress 550

#define MCP346x_DEBUG	(0U)

/**
 * @brief 本地私有功能函数
 */
static void _mcp346x_fastcmd(MCP346x *_mcp346x_inst, uint8_t fastcmd);
static void _mcp346x_start_convert(MCP346x *_mcp346x_inst);
static void _mcp346x_write_settings(MCP346x *_mcp346x_inst);
static void _mcp346x_read_config_status(MCP346x *_mcp346x_inst);
static void _mcp346x_read_adc_data(MCP346x *_mcp346x_inst);
static void _mcp346x_show_settings(MCP346x *_mcp346x_inst);
static void _mcp346x_show_config(MCP346x *_mcp346x_inst);
static void _mcp346x_show_dev_status(MCP346x *_mcp346x_inst);
static uint16_t _mcp346x_parse_adc_data(MCP346x *_mcp346x_inst);
static void _mcp346x_irq_handler(MCP346x *_mcp346x_inst);

/* 配置采样通道 */
static void _mcp346x_set_signal_channel(MCP346x *_mcp346x_inst, uint8_t _channel);
static void _mcp346x_set_diff_channel(MCP346x *_mcp346x_inst, uint8_t _channel);
static void _mcp346x_set_conv_mode(MCP346x *_mcp346x_inst, uint8_t _mode);

/**
 * @brief MCP346x 芯片上电默认配置
 */
static const MCP346x_Status MCP346x_DEFAULT_SETTINGS = 
{
	.config0.raw 			=   0xC0,
	.config1.raw 			=   0x0C,
	.config2.raw 			=   0x8B,
	.config3.raw 			=   0x00,
	.irq.raw  				=   0x73,
	.mux					=   0x01,
	.scan.raw				= { 0x00, 0x00, 0x00 },
	.timer.raw 				= { 0x00, 0x00, 0x00 },
	.offsetcal.raw 			= { 0x00, 0x00, 0x00 },
	.gaincal.raw 			= { 0x00, 0x00, 0x00 },
	.reserved1 				= { 0x90, 0x00, 0x00 },
	.reserved2				=   0x50, 
	.lock 					=   0xA5,
	.device_id				=   0x0000,
	.crccfg.raw 			=   0x0000
};

/**
 * @brief 单端采样 MUX 寄存器映射
 */
static const uint8_t MCP346x_MUX_SIGNAL_CHANNEL_MAP[8] = 
{
	MCP346x_MUX_CH0,											//!< 单端通道 0
	MCP346x_MUX_CH1,											//!< 单端通道 1
	MCP346x_MUX_CH2,											//!< 单端通道 2
	MCP346x_MUX_CH3,											//!< 单端通道 3
	MCP346x_MUX_CH4,											//!< 单端通道 4
	MCP346x_MUX_CH5,											//!< 单端通道 5
	MCP346x_MUX_CH6,											//!< 单端通道 6
	MCP346x_MUX_CH7,											//!< 单端通道 7
};

/**
 * @brief 差分采样 MUX 寄存器映射
 */
static const uint8_t MCP346x_MUX_DIFF_CHANNEL_MAP[4] = 
{
	MCP346x_MUX_DIFFA,											//!< 差分输入 A
	MCP346x_MUX_DIFFB,											//!< 差分输入 B
	MCP346x_MUX_DIFFC,											//!< 差分输入 C
	MCP346x_MUX_DIFFD,											//!< 差分输入 D
};

/**
 * @brief 其他 MUX 寄存器映射
 */
static const uint8_t MCP346x_MUX_OTHER_CHANNEL_MAP[4] = 
{
	MCP346x_MUX_TEMP,											//!< 内置温度传感器
	MCP346x_MUX_AVDD,											//!< $V_{ADD}$
	MCP346x_MUX_VCM,											//!< $V_{CM}$
	MCP346x_MUX_OFFSET											//!< OFFSET
};

/**
 * @brief 单端通道 SCAN 通道选择映射
 */
static const uint16_t MCP346x_SCAN_SIGNAL_CHANNEL_MAP[8] = 
{
	MCP346x_SCAN_CHANNEL_CH0,									//!< 单端通道 0
	MCP346x_SCAN_CHANNEL_CH1,									//!< 单端通道 1
	MCP346x_SCAN_CHANNEL_CH2,									//!< 单端通道 2
	MCP346x_SCAN_CHANNEL_CH3,									//!< 单端通道 3
	MCP346x_SCAN_CHANNEL_CH4,									//!< 单端通道 4
	MCP346x_SCAN_CHANNEL_CH5,									//!< 单端通道 5
	MCP346x_SCAN_CHANNEL_CH6,									//!< 单端通道 6
	MCP346x_SCAN_CHANNEL_CH7									//!< 单端通道 7
};

/**
 * @brief 差分通道 SCAN 通道选择映射
 */
static const uint16_t MCP346x_SCAN_DIFF_CHANNEL_MAP[4] = 
{
	MCP346x_SCAN_CHANNEL_DIFA,									//!< 单端通道 0
	MCP346x_SCAN_CHANNEL_DIFB,									//!< 单端通道 1
	MCP346x_SCAN_CHANNEL_DIFC,									//!< 单端通道 2
	MCP346x_SCAN_CHANNEL_DIFD,									//!< 单端通道 3
};


/**
 * @brief 其他通道 SCAN 通道选择映射
 */
static const uint16_t MCP346x_SCAN_OTHER_CHANNEL_MAP[4] = 
{
	MCP346x_SCAN_CHANNEL_TEMP,									//!< 温度传感器
	MCP346x_SCAN_CHANNEL_AVDD,									//!< $V_{ADD}$
	MCP346x_SCAN_CHANNEL_VCM,									//!< $V_{CM}$
	MCP346x_SCAN_CHANNEL_OFFSET,								//!< OFFSET
};


/**
 * @brief MCP346x 设备类型初始化
 */
uint8_t mcp346x_init(
	MCP346x *_mcp346x_inst, 									//!< MCP346x 实例
	void (*_cs_low_func)(void),									//!< SPI 片选拉低使能函数接口
	void (*_cs_high_func)(void),								//!< SPI 片选拉高失能函数接口
	uint8_t (*_spi_rw_func)(uint8_t),							//!< SPI 字节读写函数接口
	void (*_spi_write_data_func)(const uint8_t *, size_t len),	//!< SPI 写数据函数接口
	void (*_spi_read_data_func)(uint8_t *, size_t len))
{
	// 判断传入指针是否有效
	if(!(_mcp346x_inst 				&& 
		_cs_high_func 				&& 
		_spi_rw_func 				&&
		_spi_write_data_func 		&&
		_spi_read_data_func			&&
		_cs_low_func)) {
		return 0;
	}
	
	// 外部 SPI 接口对接
	_mcp346x_inst->spi_cs_low 			= _cs_low_func;
	_mcp346x_inst->spi_cs_high 			= _cs_high_func;
	_mcp346x_inst->spi_rw_byte			= _spi_rw_func;
	_mcp346x_inst->spi_write_data 		= _spi_write_data_func;
	_mcp346x_inst->spi_read_data 		= _spi_read_data_func;
	
	// 内部接口对接
	_mcp346x_inst->fastcmd 				= _mcp346x_fastcmd;
	_mcp346x_inst->start_convert 		= _mcp346x_start_convert;
	_mcp346x_inst->write_settings 		= _mcp346x_write_settings;
	_mcp346x_inst->read_config_status 	= _mcp346x_read_config_status;
	_mcp346x_inst->read_adc_data 		= _mcp346x_read_adc_data;
	_mcp346x_inst->show_settings 		= _mcp346x_show_settings;
	_mcp346x_inst->show_config_status 	= _mcp346x_show_config;
	_mcp346x_inst->show_dev_status 		= _mcp346x_show_dev_status;
	_mcp346x_inst->set_signal_channel 	= _mcp346x_set_signal_channel;
	_mcp346x_inst->set_diff_channel   	= _mcp346x_set_diff_channel;
	_mcp346x_inst->set_conv_mode 		= _mcp346x_set_conv_mode;
	_mcp346x_inst->irq_handler			= _mcp346x_irq_handler;
	_mcp346x_inst->parse_adc_data 		= _mcp346x_parse_adc_data;
	
	return 1;
}

/**
 * @brief 打印芯片默认配置
 *
 * @param none
 *
 * @retval none
 */
void mcp346x_show_config_default(void)
{
	LOG_LINE_LIGHT;
	LOG("Default Settings: ");
	LOG_LINE_LIGHT;
	LOG("CONFIG0\t\t\t%2.2X", MCP346x_DEFAULT_SETTINGS.config0.raw);
	LOG("CONFIG1\t\t\t%2.2X", MCP346x_DEFAULT_SETTINGS.config1.raw);
	LOG("CONFIG2\t\t\t%2.2X", MCP346x_DEFAULT_SETTINGS.config2.raw);
	LOG("CONFIG3\t\t\t%2.2X", MCP346x_DEFAULT_SETTINGS.config3.raw);
	LOG("IRQ\t\t\t\t%2.2X", MCP346x_DEFAULT_SETTINGS.irq.raw);
	LOG("MUX\t\t\t\t%2.2X", MCP346x_DEFAULT_SETTINGS.mux);
	
	for(int i=0; i<3; i++) {
		LOG("SCAN[%d]\t\t\t%2.2X", i, MCP346x_DEFAULT_SETTINGS.scan.raw[i]);
	}
	
	for(int i=0; i<3; i++) {
		LOG("TIMER[%d]\t\t\t%2.2X", i, MCP346x_DEFAULT_SETTINGS.timer.raw[i]);
	}
	for(int i=0; i<3; i++) {
		LOG("OFFSETCAL[%d]\t\t\t%2.2X", i, MCP346x_DEFAULT_SETTINGS.offsetcal.raw[i]);
	}
	for(int i=0; i<3; i++) {
		LOG("Reserved1[%d]\t\t\t%2.2X", i, MCP346x_DEFAULT_SETTINGS.reserved1[i]);
	}
	LOG("Reserved2\t\t\t%2.2X",  MCP346x_DEFAULT_SETTINGS.reserved2);
	LOG("Lock\t\t\t\t%2.2X",  MCP346x_DEFAULT_SETTINGS.lock);
	
	LOG("Device ID\t\t\t%4.4x", MCP346x_DEFAULT_SETTINGS.device_id);
	
	LOG("CRCCFG\t\t\t\t%4.4X", MCP346x_DEFAULT_SETTINGS.crccfg.raw);
	LOG_LINE_LIGHT;
}

/**
 * @brief 设置 MCP3464 的配置信息
 *
 * @param[in] _mcp346x_inst 设备描述地址
 * @param[in] _settings 传入配置信息地址
 *
 * @retval none
 */
void mcp346x_set_configration(MCP346x *_mcp346x_inst, const MCP346x_Settings *_settings)
{
	if((!_mcp346x_inst) || (!_settings))
		return;
	for(int i=0; i<sizeof(MCP346x_Settings); i++) {
		*(_mcp346x_inst->settings.raw+i) = *(_settings->raw+i);
	}
}


/**
 * @brief 发送快速指令
 *
 * @param[in] _mcp346x_inst 设备描述地址
 * @param[in] fastcmd 要发送的快速指令
 * 
 * @retval 设备状态 __STATUS
 */
static void _mcp346x_fastcmd(MCP346x *_mcp346x_inst, uint8_t fastcmd)
{	
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(fastcmd);
	_mcp346x_inst->spi_cs_high();
}

/**
 * @brief 打印 MCP346x 状态信息
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval none
 *
 * @note 使用前应对 MCP346x 设备进行读写操作
 */
static void _mcp346x_show_dev_status(MCP346x *_mcp346x_inst)
{
	LOG_U8_BIT_SHOW(_mcp346x_inst->dev_status.raw);
}

/**
 * @brief 发送启动采样指令
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval 设备状态 __STATUS
 */
static void _mcp346x_start_convert(MCP346x *_mcp346x_inst)
{
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(MCP346x_CMD_CONVERSION);
	_mcp346x_inst->spi_cs_high();
}

/**
 * @brief 发送寄存器配置
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval 设备状态 __STATUS
 */
static void _mcp346x_write_settings(MCP346x *_mcp346x_inst)
{
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(MCP346x_CMD_IWRITE | MCP346x_ADR_CONFIG0);
	_mcp346x_inst->spi_write_data(_mcp346x_inst->settings.raw, sizeof(MCP346x_Settings));
	_mcp346x_inst->spi_cs_high();
}

/**
 * @brief 打印 MCP346x 设备的配置信息
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval none
 */
static void _mcp346x_show_settings(MCP346x *_mcp346x_inst)
{
	LOG_LINE_LIGHT;
	LOG("MCP Device Settings: ");
	LOG_LINE_LIGHT;
	LOG("CONFIG0\t\t\t%2.2X", _mcp346x_inst->settings.config0.raw);
	LOG("CONFIG1\t\t\t%2.2X", _mcp346x_inst->settings.config1.raw);
	LOG("CONFIG2\t\t\t%2.2X", _mcp346x_inst->settings.config2.raw);
	LOG("CONFIG3\t\t\t%2.2X", _mcp346x_inst->settings.config3.raw);
	LOG("IRQ\t\t\t\t%2.2X", _mcp346x_inst->settings.irq.raw);
	LOG("MUX\t\t\t\t%2.2X", _mcp346x_inst->settings.mux);
	for(int i=0; i<3; i++) {
		LOG("SCAN[%d]\t\t\t%2.2X", i, _mcp346x_inst->settings.scan.raw[i]);
	}
	for(int i=0; i<3; i++) {
		LOG("TIMER[%d]\t\t\t%2.2X", i, _mcp346x_inst->settings.timer.raw[i]);
	}
}

/**
 * @brief 打印 MCP346x 设备的寄存器信息
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval none
 */
static void _mcp346x_show_config(MCP346x *_mcp346x_inst)
{
	LOG_LINE_LIGHT;
	LOG("MCP Device Register Config: ");
	LOG_LINE_LIGHT;
	LOG("CONFIG0\t\t\t%2.2X", _mcp346x_inst->config_status.config0.raw);
	LOG("CONFIG1\t\t\t%2.2X", _mcp346x_inst->config_status.config1.raw);
	LOG("CONFIG2\t\t\t%2.2X", _mcp346x_inst->config_status.config2.raw);
	LOG("CONFIG3\t\t\t%2.2X", _mcp346x_inst->config_status.config3.raw);
	LOG("IRQ\t\t\t\t%2.2X", _mcp346x_inst->config_status.irq.raw);
	LOG("MUX\t\t\t\t%2.2X", _mcp346x_inst->config_status.mux);
	for(int i=0; i<3; i++) {
		LOG("SCAN[%d]\t\t\t%2.2X", i, _mcp346x_inst->config_status.scan.raw[i]);
	}
	for(int i=0; i<3; i++) {
		LOG("TIMER[%d]\t\t\t%2.2X", i, _mcp346x_inst->config_status.timer.raw[i]);
	}
	for(int i=0; i<3; i++) {
		LOG("OFFSETCAL[%d]\t\t\t%2.2X", i, _mcp346x_inst->config_status.offsetcal.raw[i]);
	}
	for(int i=0; i<3; i++) {
		LOG("GAINCAL[%d]\t\t\t%2.2X", i, _mcp346x_inst->config_status.gaincal.raw[i]);
	}
	for(int i=0; i<3; i++) {
		LOG("Reserved1[%d]\t\t\t%2.2X", i, _mcp346x_inst->config_status.reserved1[i]);
	}
	LOG("Reserved2\t\t\t%2.2X",  _mcp346x_inst->config_status.reserved2);
	LOG("Lock\t\t\t\t%2.2X",  _mcp346x_inst->config_status.lock);
	LOG("Device ID\t\t\t%4.4x", _mcp346x_inst->config_status.device_id);
	LOG("CRCCFG\t\t\t\t%4.4X", _mcp346x_inst->config_status.crccfg.raw);
	LOG_LINE_LIGHT;
}

/**
 * @brief 读取 MCP346x 中寄存器的状态
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval 设备状态 __STATUS
 */
static void _mcp346x_read_config_status(MCP346x *_mcp346x_inst)
{
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(MCP346x_CMD_IREAD | MCP346x_ADR_CONFIG0);
	_mcp346x_inst->spi_read_data(_mcp346x_inst->config_status.raw, sizeof(MCP346x_Status));
	_mcp346x_inst->spi_cs_high();
}

/**
 * @brief 读取 ADC 数据
 *
 * @param[in] _mcp346x_inst 设备描述地址
 * @param[out] _data 输出数据, 需要根据 CONFIG3 的数据模式进行解析
 *
 * @retval 设备状态 __STATUS
 */
static void _mcp346x_read_adc_data(MCP346x *_mcp346x_inst)
{
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(MCP346x_CMD_IREAD | MCP346x_ADR_ADCDATA);
	_mcp346x_inst->spi_read_data(_mcp346x_inst->adc_data, 4);
	_mcp346x_inst->spi_cs_high();
}

/**
 * @brief 设置单端采样通道
 *
 * @param[in] _mcp346x_inst 设备描述地址
 * @param[in] _channel 采样通道, 根据不同类型设备有不同的最大索引
 *    \ref 0 配置为采样通道 0
 *    \ref 1 配置为采样通道 1
 *    \ref 2 配置为采样通道 2
 *    \ref 3 配置为采样通道 3
 *    \ref 4 配置为采样通道 4
 *    \ref 5 配置为采样通道 5
 *    \ref 6 配置为采样通道 6
 *    \ref 7 配置为采样通道 7
 *
 * @retval none
 */
static void _mcp346x_set_signal_channel(MCP346x *_mcp346x_inst, uint8_t _channel)
{
	if(_channel > 7)
		return;
	
	_mcp346x_inst->settings.mux = MCP346x_MUX_SIGNAL_CHANNEL_MAP[_channel];
	_mcp346x_inst->settings.scan.channel = MCP346x_SCAN_SIGNAL_CHANNEL_MAP[_channel];
	
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(MCP346x_CMD_IWRITE | MCP346x_ADR_MUX);
	_mcp346x_inst->spi_rw_byte(_mcp346x_inst->settings.mux);
	_mcp346x_inst->spi_write_data(_mcp346x_inst->settings.scan.raw, sizeof(_mcp346x_inst->settings.scan));
	_mcp346x_inst->spi_cs_high();
	
	_mcp346x_inst->config_status.mux = MCP346x_MUX_SIGNAL_CHANNEL_MAP[_channel];
	_mcp346x_inst->config_status.scan.channel = MCP346x_SCAN_SIGNAL_CHANNEL_MAP[_channel];
}

/**
 * @brief 设置差分采样通道
 *
 * @param[in] _mcp346x_inst 设备描述地址
 * @param[in] _channel 采样通道, 根据不同类型设备有不同的最大索引
 *    \ref 0 配置差分采样 A 通道
 *    \ref 1 配置差分采样 B 通道
 *    \ref 2 配置差分采样 C 通道
 *    \ref 3 配置差分采样 D 通道
 *
 * @retval none
 */
static void _mcp346x_set_diff_channel(MCP346x *_mcp346x_inst, uint8_t _channel)
{
	if(_channel > 3)
		return;
	
	_mcp346x_inst->settings.mux = MCP346x_MUX_DIFF_CHANNEL_MAP[_channel];
	_mcp346x_inst->settings.scan.channel = MCP346x_SCAN_DIFF_CHANNEL_MAP[_channel];
	
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(MCP346x_CMD_IWRITE | MCP346x_ADR_MUX);
	_mcp346x_inst->spi_rw_byte(_mcp346x_inst->settings.mux);
	_mcp346x_inst->spi_write_data(_mcp346x_inst->settings.scan.raw, sizeof(_mcp346x_inst->settings.scan));
	_mcp346x_inst->spi_cs_high();
	
	_mcp346x_inst->config_status.mux = MCP346x_MUX_DIFF_CHANNEL_MAP[_channel];
	_mcp346x_inst->config_status.scan.channel = MCP346x_SCAN_DIFF_CHANNEL_MAP[_channel];
}

/**
 * @brief 设置 ADC 转化模式
 *
 * @param[in] _mcp346x_inst 设备描述地址
 * @param[in] _mode 采样模式
 *    \ref MCP346x_CONV_MODE_CONTINUOUS 连续转换模式
 *    \ref MCP346x_CONV_MODE_STANDBY 单次转换之后设置位 Standby 模式
 *    \ref MCP346x_CONV_MODE_ONESHUT 单次转换
 *
 * @retval none
 */
static void _mcp346x_set_conv_mode(MCP346x *_mcp346x_inst, uint8_t _mode)
{
	if(_mode > 3) 
		return;
	
	_mcp346x_inst->settings.config3.conv_mode = _mode;
	
	_mcp346x_inst->spi_cs_low();
	_mcp346x_inst->dev_status.raw = _mcp346x_inst->spi_rw_byte(MCP346x_CMD_IWRITE | MCP346x_ADR_CONFIG3);
	_mcp346x_inst->spi_rw_byte(_mcp346x_inst->settings.config3.raw);
	_mcp346x_inst->spi_cs_high();
	
	_mcp346x_inst->config_status.config3.conv_mode = _mode;
}

/**
 * @brief 解析 SPI 接收到的 ADC 数据
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval none
 */
static uint16_t _mcp346x_parse_adc_data(MCP346x *_mcp346x_inst)
{
	switch (_mcp346x_inst->settings.config3.data_fmt) {
	case MCP346x_DATA_FMT_ID_SGN_DATA:
	case MCP346x_DATA_FMT_SGN_DATA:	
		return _mcp346x_inst->adc_data[2] << 8 | _mcp346x_inst->adc_data[3];
	case MCP346x_DATA_FMT_SGN_DATA_ZERO:
	case MCP346x_DATA_FMT_DATA:
		return _mcp346x_inst->adc_data[0] << 8 | _mcp346x_inst->adc_data[1];
	}
	return 0;
}

/**
 * @brief MCP346x 中断处理, 更新 ADC 数据
 *
 * @param[in] _mcp346x_inst 设备描述地址
 *
 * @retval none
 */
static void _mcp346x_irq_handler(MCP346x *_mcp346x_inst)
{	
	static uint8_t counter = 0;
	while(_mcp346x_inst->dev_status.dr_status) {
		_mcp346x_inst->read_adc_data(_mcp346x_inst);
	}
	if(counter == 5) {
		counter = 0;
		gpio_bit_toggle(GPIOB, GPIO_PIN_0);
	}
	counter++;
#if (MCP346x_DEBUG)
	for(int i=0;i<4;i++) {
		LOG("%d", _mcp346x_inst->adc_data[i]);
	}
	LOG_LINE_LIGHT;
#endif
}
