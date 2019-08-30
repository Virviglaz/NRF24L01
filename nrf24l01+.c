#include "nrf24l01+.h"

#define CONFIG_REG				0x00
#define EN_AA_REG					0x01
#define EN_RXADDR_REG				0x02
#define SETUP_AW_REG				0x03
#define SETUP_RETR_REG			0x04
#define RF_CH_REG					0x05
#define RF_SETUP_REG				0x06
#define STATUS_REG				0x07
#define OBSERV_TX_REG				0x08
#define CD_REG					0x09
#define RX_ADDR_P0_REG			0x0A
#define RX_ADDR_P1_REG			0x0B
#define RX_ADDR_P2_REG			0x0C
#define RX_ADDR_P3_REG			0x0D
#define RX_ADDR_P4_REG			0x0E
#define RX_ADDR_P5_REG			0x0F
#define TX_ADDR_REG				0x10
#define RX_PW_P0_REG				0x11
#define RX_PW_P1_REG				0x12
#define RX_PW_P2_REG				0x13
#define RX_PW_P3_REG				0x14
#define RX_PW_P4_REG				0x15
#define RX_PW_P5_REG				0x16
#define FIFO_STATUS_REG			0x17

#define SEND_PAYLOAD_CMD			0xA0
#define RX_DR_IRQ_MASK			0x40
#define TX_DS_IRQ_MASK			0x20
#define MAX_RT_IRQ_MASK			0x10
#define FLUSH_TX_CMD				0xE1
#define FLUSH_RX_CMD				0xE2
#define RX_PAYLOAD_CMD			0x61
#define PIPE_BITMASK				0x0E
#define RX_FIFO_EMPTY_MASK		0x01
#define INVERT_IRQ_BITS				0x70

static struct nrf24l01_conf *local_driver;

static void write_reg(uint8_t reg, uint8_t value)
{
	local_driver->interface.error =
		local_driver->interface.write((0x1F & reg) | (1 << 5),
			&value, sizeof(value));
}

static uint8_t read_reg(uint8_t reg)
{
	uint8_t res;
	local_driver->interface.error =
		local_driver->interface.read(0x1F & reg, &res, sizeof(res));

	return res;
}

static void write_address(uint8_t reg, uint8_t *address)
{
	if (address)
		local_driver->interface.write(reg | (1 << 5), address, 5);
}

static bool read_irq(uint8_t irq_mask)
{
	if (local_driver->interface.read_irq)
		return !local_driver->interface.read_irq();

	return (read_reg(STATUS_REG)) & irq_mask;
}

static void clear_irq(uint8_t irq_mask)
{
	write_reg(STATUS_REG, irq_mask);
}

static void flush_buffer(uint8_t cmd)
{
	local_driver->interface.write(cmd, 0, 0);
}

static void update_config(void)
{
	write_reg(CONFIG_REG, *(uint8_t *)&local_driver->config ^ INVERT_IRQ_BITS);
}

static void disable_radio (void)
{
	write_reg(CONFIG_REG, 0x00);
}

static bool send(uint8_t *dest, uint8_t *data, uint8_t size)
{
	uint32_t cnt = local_driver->read_cnt;
	bool res, in_rx = local_driver->config.mode == RADIO_RX;
	uint8_t tx_irq = local_driver->auto_retransmit.count ?
		MAX_RT_IRQ_MASK | TX_DS_IRQ_MASK : TX_DS_IRQ_MASK;

	if (dest) /* Change distanation if needed */
		write_address(TX_ADDR_REG, dest);
	
	local_driver->interface.radio_en(true);
	
	local_driver->mode(RADIO_TX, true);

	local_driver->interface.write(SEND_PAYLOAD_CMD, data, size);
	
	while (!read_irq(tx_irq) && cnt--);

	res = !(cnt == 0);

	clear_irq(tx_irq);
	flush_buffer(FLUSH_TX_CMD);
	
	/* Switch back to RX */
	if (in_rx)
		local_driver->mode(RADIO_RX, true);
	else 
		local_driver->interface.radio_en(false);

	return res;
}

static uint8_t recv(uint8_t *data, uint8_t *pipe_num)
{
	uint8_t num;
	
	if (local_driver->config.mode == RADIO_TX) {
		local_driver->mode(RADIO_RX, true);
		local_driver->interface.radio_en(true);
	}

	if (!read_irq(RX_DR_IRQ_MASK))
		return 0; /* FIFO empty */

	/* Calculate pipe num */
	num = read_reg(STATUS_REG);
	num &= PIPE_BITMASK;
	num >>= 1;

	if (pipe_num)
		*pipe_num = num;
	num = local_driver->rx_pipe_size[num];

	local_driver->interface.read(RX_PAYLOAD_CMD, data, num);

	/* Clear IRQ only if last data received (using FIFO) */
	if (read_reg(FIFO_STATUS_REG) & RX_FIFO_EMPTY_MASK) {
		clear_irq(RX_DR_IRQ_MASK);
		flush_buffer(FLUSH_RX_CMD);
	}

	/* Return number of bytes in pipe */
	return num;
}

static void switch_mode(enum radio_mode mode, bool power_enable)
{
	local_driver->config.mode = mode;
	local_driver->config.power_enable = power_enable;

	disable_radio();

	if (mode == RADIO_TX) {
		local_driver->config.tx_irq = true;
		local_driver->config.rx_irq = false;
		local_driver->config.max_rt_irq = local_driver->auto_retransmit.count;
	} else {
		local_driver->config.rx_irq = true;
		local_driver->config.tx_irq = false;
		local_driver->config.max_rt_irq = false;
	}

	update_config();
}

static void sleep(void)
{
	local_driver->interface.radio_en(false);
	switch_mode(local_driver->config.mode, false);
}

static void wakeup(void)
{
	switch_mode(local_driver->config.mode, true);

	if (local_driver->config.mode == RADIO_RX)
		local_driver->interface.radio_en(true);
}

uint8_t nrf24l01_init(struct nrf24l01_conf *driver)
{
	local_driver = driver;
	local_driver->send = send;
	local_driver->recv = recv;
	local_driver->mode = switch_mode;
	local_driver->sleep = sleep;
	local_driver->wakeup = wakeup;

	disable_radio();
	write_reg(EN_AA_REG, *(uint8_t *)&local_driver->auto_acknowledgment);
	write_reg(EN_RXADDR_REG, *(uint8_t *)&local_driver->enabled_rx_addresses);
	write_reg(SETUP_AW_REG, *(uint8_t *)&local_driver->address_widths);
	write_reg(SETUP_RETR_REG, *(uint8_t *)&local_driver->auto_retransmit);
	write_reg(RF_CH_REG, local_driver->channel & 0x7F);
	write_reg(RF_SETUP_REG, *(uint8_t *)&local_driver->setup);
	write_address(RX_ADDR_P0_REG, local_driver->rx_address_p0);
	write_address(RX_ADDR_P1_REG, local_driver->rx_address_p1);
	write_address(TX_ADDR_REG, local_driver->tx_address);
	write_reg(RX_ADDR_P2_REG, local_driver->rx_address_p2);
	write_reg(RX_ADDR_P3_REG, local_driver->rx_address_p3);
	write_reg(RX_ADDR_P4_REG, local_driver->rx_address_p4);
	write_reg(RX_ADDR_P5_REG, local_driver->rx_address_p5);
	write_reg(RX_PW_P0_REG, local_driver->rx_pipe_size[0] & 0x3F);
	write_reg(RX_PW_P1_REG, local_driver->rx_pipe_size[1] & 0x3F);
	write_reg(RX_PW_P2_REG, local_driver->rx_pipe_size[2] & 0x3F);
	write_reg(RX_PW_P3_REG, local_driver->rx_pipe_size[3] & 0x3F);
	write_reg(RX_PW_P4_REG, local_driver->rx_pipe_size[4] & 0x3F);
	write_reg(RX_PW_P5_REG, local_driver->rx_pipe_size[5] & 0x3F);

	/* invert irq enable bits */
	update_config();

	if (local_driver->config.mode == RADIO_RX)
		local_driver->interface.radio_en(true);

	return local_driver->interface.error;;
}
