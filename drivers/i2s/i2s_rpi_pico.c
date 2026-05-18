#include <hardware/clocks.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_SOC_SERIES_RP2040)
#include <zephyr/dt-bindings/dma/rpi-pico-dma-rp2040.h>
#elif defined(CONFIG_SOC_SERIES_RP2350)
#include <zephyr/dt-bindings/dma/rpi-pico-dma-rp2350.h>
#endif

#define DT_DRV_COMPAT raspberrypi_pico_i2s

#define SM_SIDESET_BIT_COUNT 2
#define SM_AUTOPULL_THRESHOLD 32

LOG_MODULE_REGISTER(i2s_rpi_pico, CONFIG_I2S_LOG_LEVEL);

RPI_PICO_PIO_DEFINE_PROGRAM(i2s_tx, 0, 7, 
		//     .wrap_target
	0xb822, //  0: mov    x, y            side 3
	0x7001, //  1: out    pins, 1         side 2
	0x1841, //  2: jmp    x--, 1          side 3
	0x6001, //  3: out    pins, 1         side 0
	0xa822, //  4: mov    x, y            side 1
	0x6001, //  5: out    pins, 1         side 0
	0x0845, //  6: jmp    x--, 5          side 1
	0x7001, //  7: out    pins, 1         side 2
		//     .wrap
);

struct i2s_q_item
{
	void *mem_block;
	size_t size;
};

struct i2s_dev_config
{
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	uint32_t clock_pin_base;
	uint32_t data_pin;
};

struct stream
{
	enum i2s_state state;
	struct k_msgq *msgq;
	struct i2s_config i2s_cfg;
	const struct device *dev_dma;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_block_cfg;
	size_t sm;
	void *mem_block;
	bool last_block;
};

struct i2s_dev_data
{
	struct stream tx;
};

static void stream_queue_drop(struct stream *strm)
{
	struct i2s_q_item item;

	while (!k_msgq_get(strm->msgq, &item, K_NO_WAIT)) {
		LOG_DBG("Dropping item from queue");
		k_mem_slab_free(strm->i2s_cfg.mem_slab, item.mem_block);
	}
}

static void pio_i2s_set_bclk(PIO pio, uint8_t sm, struct i2s_config *cfg)
{
	const uint32_t sysclk_freq = clock_get_hz(clk_sys);
	const uint32_t bclk_freq = cfg->frame_clk_freq * cfg->channels * cfg->word_size;
	const uint32_t divider = 256ULL * sysclk_freq / (2ULL * bclk_freq); // TODO magic numbers

	pio_sm_set_clkdiv_int_frac(pio, sm, divider >> 8U, divider & 0xFFU);
}

static int pio_i2s_tx_init(PIO pio, uint32_t sm, uint32_t data_pin, uint32_t clock_pin_base) 
{
	const uint32_t pin_mask = (1U << data_pin) | (3U << clock_pin_base);
	uint32_t offset;
	pio_sm_config cfg;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_tx))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(i2s_tx));

	cfg = pio_get_default_sm_config(); 
	sm_config_set_out_shift(&cfg, false, true, SM_AUTOPULL_THRESHOLD);
	sm_config_set_out_pins(&cfg, data_pin, 1);
	sm_config_set_sideset(&cfg, SM_SIDESET_BIT_COUNT, false, false);
	sm_config_set_sideset_pins(&cfg, clock_pin_base);
	sm_config_set_wrap(&cfg, 
				offset + RPI_PICO_PIO_GET_WRAP_TARGET(i2s_tx),
				offset + RPI_PICO_PIO_GET_WRAP(i2s_tx));
	pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
	pio_sm_set_pins_with_mask(pio, sm, pin_mask, pin_mask);
	pio_gpio_init(pio, data_pin);
	pio_gpio_init(pio, clock_pin_base);
	pio_gpio_init(pio, clock_pin_base + 1);
	pio_sm_init(pio, sm, offset, &cfg);

	return 0;
}

static void pio_i2s_tx_set_word_size(PIO pio, uint32_t sm, uint8_t word_size)
{
	/* PIO program expects register Y to be initialized to word_size - 2 */
	pio_sm_exec(pio, sm, pio_encode_set(pio_y, word_size - 2));
}

static int i2s_rpi_pico_initialize(const struct device *dev)
{
	const struct i2s_dev_config *dev_cfg = dev->config;
	struct i2s_dev_data *dev_data = dev->data;
	PIO pio;
	int ret;

	if (!device_is_ready(dev_data->tx.dev_dma)) {
		LOG_ERR("%s not ready", dev_data->tx.dev_dma->name);
		return -ENODEV;
	}

	pio = pio_rpi_pico_get_pio(dev_cfg->piodev);

	ret = pio_rpi_pico_allocate_sm(dev_cfg->piodev, &dev_data->tx.sm);
	if (ret < 0) {
		LOG_ERR("pio_rpi_pico_allocate_sm() failed: %d", ret);
		return ret;
	}

	dev_data->tx.dma_cfg.user_data = (void *)dev;
	dev_data->tx.dma_cfg.dma_slot = RPI_PICO_DMA_DREQ_TO_SLOT(pio_get_dreq(pio, dev_data->tx.sm, true));

	ret = pio_i2s_tx_init(pio, dev_data->tx.sm, dev_cfg->data_pin, dev_cfg->clock_pin_base);
	if (ret < 0) {
		LOG_ERR("pio_tx_init() failed: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state() failed: %d", ret);
		return ret;
	}

	ret = dma_config(dev_data->tx.dev_dma, dev_data->tx.dma_channel, &dev_data->tx.dma_cfg);
	if (ret < 0) {
		LOG_ERR("dma_config() failed: %d", ret);
		return ret;
	}

	return 0;
}

static int start_dma(struct stream *strm, void *src, bool src_addr_incr, void *dst, bool dst_addr_incr, uint32_t size)
{
	int ret;

	strm->dma_block_cfg.block_size = size;
	strm->dma_block_cfg.source_address = (uint32_t)src;
	strm->dma_block_cfg.dest_address = (uint32_t)dst;
	strm->dma_block_cfg.source_addr_adj = src_addr_incr ? DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;
	strm->dma_block_cfg.dest_addr_adj = dst_addr_incr ? DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;

	ret = dma_config(strm->dev_dma, strm->dma_channel, &strm->dma_cfg);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(strm->dev_dma, strm->dma_channel);

	return ret;
}

static int reload_dma(struct stream *strm, void *src, void *dst, uint32_t size)
{
	int ret;

	ret = dma_reload(strm->dev_dma, strm->dma_channel, (uint32_t)src, (uint32_t)dst, size);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(strm->dev_dma, strm->dma_channel);

	return ret;
}

static int tx_stream_start(const struct device *dev)
{
	const struct i2s_dev_config *dev_cfg = dev->config;
	struct i2s_dev_data *dev_data = dev->data;
	struct stream *strm = &dev_data->tx;
	struct i2s_q_item block;
	int ret;
	PIO pio;

	ret = k_msgq_get(strm->msgq, &block, SYS_TIMEOUT_MS(strm->i2s_cfg.timeout));
	if (ret < 0) {
		LOG_ERR("k_msgq_get() failed: %d", ret);
		return ret;
	}

	strm->mem_block = block.mem_block;
	pio = pio_rpi_pico_get_pio(dev_cfg->piodev);

	ret = start_dma(strm, block.mem_block, true, (void *)&pio->txf[strm->sm], false, block.size);
	if (ret < 0) {
		LOG_ERR("start_dma() failed: %d", ret);
		return ret;
	}

	pio_sm_set_enabled(pio, strm->sm, true);

	return 0;
}

static void tx_stream_disable(const struct device *dev)
{
	const struct i2s_dev_config *dev_cfg = dev->config;
	struct i2s_dev_data *dev_data = dev->data;
	struct stream *strm = &dev_data->tx;
	PIO pio;

	dma_stop(strm->dev_dma, strm->dma_channel);

	if (strm->mem_block != NULL) {
		k_mem_slab_free(strm->i2s_cfg.mem_slab, strm->mem_block);
		strm->mem_block = NULL;
	}

	pio = pio_rpi_pico_get_pio(dev_cfg->piodev);
	pio_sm_set_enabled(pio, strm->sm, false);
}

static void i2s_dma_tx_callback(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	const struct device *dev = arg;
	const struct i2s_dev_config *dev_cfg = dev->config;
	struct i2s_dev_data *dev_data = dev->data;
	struct stream *strm = &dev_data->tx;
	struct i2s_q_item block;
	int ret;
	PIO pio;

	/* Stop if DMA error */
	if (status < 0) {
		LOG_ERR("DMA tx callback error: %d", status);
		strm->state = I2S_STATE_ERROR;
		tx_stream_disable(dev);
		return;
	}

	/* Entire block sent, free it */
	k_mem_slab_free(strm->i2s_cfg.mem_slab, strm->mem_block);
	strm->mem_block = NULL;

	/* Stop if I2S error */
	if (strm->state == I2S_STATE_ERROR) {
		LOG_ERR("I2S tx error");
		tx_stream_disable(dev);
		return;
	}

	/* STOP trigger, stop immediately */
	if (strm->last_block) {
		strm->state = I2S_STATE_READY;
		tx_stream_disable(dev);
		return;
	}

	/* Prepare to send the next data block */
	ret = k_msgq_get(strm->msgq, &block, K_NO_WAIT);
	if (ret < 0) {
		/* Queue drained after DRAIN trigger */
		if (strm->state == I2S_STATE_STOPPING) {
			strm->state = I2S_STATE_READY;
			LOG_DBG("Queue drained");
		}
		else {
			strm->state = I2S_STATE_ERROR;
			LOG_ERR("k_msgq_get() failed: %d", ret);
		}
		tx_stream_disable(dev);
		return;
	}

	strm->mem_block = block.mem_block;
	pio = pio_rpi_pico_get_pio(dev_cfg->piodev);

	/* Start next DMA transfer */
	ret = reload_dma(strm, block.mem_block, (void *)&pio->txf[strm->sm], block.size);
	if (ret < 0) {
		LOG_ERR("reload_dma() failed: %d", ret);
		tx_stream_disable(dev);
	}
}

static int i2s_rpi_pico_config(const struct device *dev, enum i2s_dir dir, const struct i2s_config *i2s_cfg)
{
	const struct i2s_dev_config *dev_cfg = dev->config;
	struct i2s_dev_data *dev_data = dev->data;
	struct stream *strm;
	PIO pio;

	if (dir == I2S_DIR_TX) {
		strm = &dev_data->tx;
	}
	else {
		LOG_ERR("Only tx direction supported");
		return -ENOSYS;
	}

	strm->i2s_cfg = *i2s_cfg;

	if ((strm->state != I2S_STATE_NOT_READY) && (strm->state != I2S_STATE_READY)) {
		LOG_ERR("Invalid state: %d", strm->state);
		return -EINVAL;
	}

	if ((strm->i2s_cfg.options & (I2S_OPT_FRAME_CLK_TARGET | I2S_OPT_BIT_CLK_TARGET)) != 0) {
		LOG_ERR("Target role not supported");
		return -ENOSYS;
	}

	if (strm->i2s_cfg.frame_clk_freq == 0) {
		LOG_ERR("frame_clk_freq cannot be zero");
		strm->state = I2S_STATE_NOT_READY;
		return -EINVAL;
	}

	switch (strm->i2s_cfg.word_size) {
		case 16:
		case 32:
			break;
		case 24:
			strm->i2s_cfg.word_size = 32; // 24-bit sample uses 32-bit frame length
			break;
		default:
			LOG_ERR("word_size %u not supported", strm->i2s_cfg.word_size);
			return -ENOSYS;
	}

	if ((strm->i2s_cfg.format & I2S_FMT_DATA_FORMAT_MASK) != I2S_FMT_DATA_FORMAT_I2S) {
		LOG_ERR("Only I2S format supported");
		return -ENOSYS;
	}

	if ((strm->i2s_cfg.format & (I2S_FMT_CLK_FORMAT_MASK | I2S_FMT_FRAME_CLK_INV)) != 0) {
		LOG_ERR("Inverted clock polarity not supported");
		return -ENOSYS;
	}

	pio = pio_rpi_pico_get_pio(dev_cfg->piodev);
	pio_i2s_set_bclk(pio, strm->sm, &strm->i2s_cfg);
	pio_i2s_tx_set_word_size(pio, strm->sm, strm->i2s_cfg.word_size);

	strm->state = I2S_STATE_READY;

	return 0;
}

const struct i2s_config *i2s_rpi_pico_config_get(const struct device *dev, enum i2s_dir dir)
{
	return NULL; // TODO
}

static int i2s_rpi_pico_read(const struct device *dev, void **mem_block, size_t *size)
{
	return -ENOTSUP;
}

static int i2s_rpi_pico_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_dev_data *dev_data = dev->data;
	struct stream *strm = &dev_data->tx;
	struct i2s_q_item block;
	int ret;

	if ((strm->state != I2S_STATE_RUNNING) && (strm->state != I2S_STATE_READY)) {
		LOG_ERR("Invalid state: %d", strm->state);
		return -EIO;
	}

	if (size > strm->i2s_cfg.block_size) {
		LOG_ERR("Max write size is %uB", strm->i2s_cfg.block_size);
		return -EINVAL;
	}

	block.mem_block = mem_block;
	block.size = size;

	ret = k_msgq_put(strm->msgq, &block, SYS_TIMEOUT_MS(strm->i2s_cfg.timeout));
	if (ret < 0) {
		LOG_ERR("k_msgq_put() failed: %d", ret);
		return ret;
	}

	return 0;
}

static int i2s_rpi_pico_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_dev_data *dev_data = dev->data;
	struct stream *strm = &dev_data->tx;
	int ret = 0;
	unsigned int key;

	// TODO DIR_BOTH

	key = irq_lock();

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (strm->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}
		ret = tx_stream_start(dev);
		if (ret < 0) {
			LOG_ERR("tx_stream_start() failed: %d", ret);
			break; 
		}
		strm->state = I2S_STATE_RUNNING;
		strm->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		if (strm->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}
		strm->state = I2S_STATE_STOPPING;
		strm->last_block = true;
		break;

	case I2S_TRIGGER_DRAIN:
		if (strm->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}
		strm->state = I2S_STATE_STOPPING;
		break;

	case I2S_TRIGGER_DROP:
		if (strm->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}
		tx_stream_disable(dev);
		stream_queue_drop(strm);
		strm->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (strm->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}
		stream_queue_drop(strm);
		strm->state = I2S_STATE_READY;
		break;

	default:
		LOG_ERR("Unsupported trigger %d", cmd);
		ret = -EINVAL;
		break;
	}


	irq_unlock(key);

	return ret;
}

static DEVICE_API(i2s, i2s_rpi_pico_driver_api) = {
	.configure = i2s_rpi_pico_config,
	.read = i2s_rpi_pico_read,
	.write = i2s_rpi_pico_write,
	.config_get = i2s_rpi_pico_config_get,
	.trigger = i2s_rpi_pico_trigger,
};

#define I2S_RPI_PICO_INIT(inst)										\
	PINCTRL_DT_INST_DEFINE(inst);									\
													\
	static const struct i2s_dev_config i2s##inst##_dev_config = {					\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(inst)),						\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),						\
		.clock_pin_base = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, tx_pins, 0),	\
		.data_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, tx_pins, 1),		\
	};												\
													\
	K_MSGQ_DEFINE(i2s##inst##_tx_queue, sizeof(struct i2s_q_item),					\
			CONFIG_I2S_RX_BLOCK_COUNT, 4);							\
													\
	static struct i2s_dev_data i2s##inst##_dev_data = {						\
		.tx = {											\
			.msgq = &i2s##inst##_tx_queue,							\
			.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, tx)),			\
			.dma_channel = DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel),			\
			.dma_cfg = {									\
				.source_burst_length = 1,                                   		\
				.dest_burst_length = 1,                                     		\
				.dma_callback = i2s_dma_tx_callback,               			\
				.block_count = 1,							\
				.head_block = &i2s##inst##_dev_data.tx.dma_block_cfg,			\
				.channel_direction = MEMORY_TO_PERIPHERAL,				\
				.source_data_size = 4,  /* TODO configurable */				\
				.dest_data_size = 4,    /* TODO configurable */				\
			},										\
		},											\
	};												\
													\
	DEVICE_DT_INST_DEFINE(inst, i2s_rpi_pico_initialize, NULL,					\
			&i2s##inst##_dev_data, &i2s##inst##_dev_config,					\
			POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,						\
			&i2s_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_RPI_PICO_INIT)
