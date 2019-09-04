/*
 * This file is provided under a MIT license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   MIT License
 *
 *   Copyright (c) 2019 Pavel Nadein
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * nRF24L01 open source library/driver
 *
 * Contact Information:
 * Pavel Nadein <pavel.nadein@gmail.com>
 */

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

#define MAX_PIPE_SIZE			32
#define MAX_TX_TIMEOUT_MS		100

void NRF24L01::write_reg(uint8_t reg, uint8_t value)
{
	conf.interface.error =
		conf.interface.write((0x1F & reg) | (1 << 5), &value, sizeof(value));
}

uint8_t NRF24L01::read_reg(uint8_t reg)
{
	uint8_t res;
	conf.interface.error = conf.interface.read(0x1F & reg, &res, sizeof(res));

	return res;
}

void NRF24L01::write_address(uint8_t reg, uint8_t *address)
{
	if (address) conf.interface.write(reg | (1 << 5), address, 5);
}

bool NRF24L01::read_irq(uint8_t irq_mask)
{
	if (conf.interface.read_irq)
		return !conf.interface.read_irq();

	return (read_reg(STATUS_REG)) & irq_mask;
}

void NRF24L01::clear_irq(uint8_t irq_mask)
{
	write_reg(STATUS_REG, irq_mask);
}

void NRF24L01::flush_buffer(uint8_t cmd)
{
	conf.interface.write(cmd, 0, 0);
}

void NRF24L01::update_config(void)
{
	write_reg(CONFIG_REG, *(uint8_t *)&conf.config ^ INVERT_IRQ_BITS);
}

void NRF24L01::disable_radio(void)
{
	write_reg(CONFIG_REG, 0x00);
}

bool NRF24L01::Send(uint8_t *dest, uint8_t *data, uint8_t size)
{
	bool res, in_rx = conf.config.mode == RADIO_RX;
	uint8_t tx_irq = conf.auto_retransmit.count ?
		MAX_RT_IRQ_MASK | TX_DS_IRQ_MASK : TX_DS_IRQ_MASK;
	*conf.timeout_ms = MAX_TX_TIMEOUT_MS;

	if (dest) /* Change distanation if needed */
		write_address(TX_ADDR_REG, dest);

	conf.interface.radio_en(true);

	switch_mode(RADIO_TX, true);

	conf.interface.write(SEND_PAYLOAD_CMD, data, size);

	while (!read_irq(tx_irq) && *conf.timeout_ms);

	if (conf.auto_retransmit.count)
		/* Check maximum retransmit is set, return false */
		res = !(read_reg(STATUS_REG) & MAX_RT_IRQ_MASK);
	else
		/* Just check that chip is responding */
		res = !(*conf.timeout_ms == 0);

	clear_irq(tx_irq);
	flush_buffer(FLUSH_TX_CMD);

	/* Switch back to RX */
	if (in_rx)
		switch_mode(RADIO_RX, true);
	else
		conf.interface.radio_en(false);

	return res;
}

uint8_t NRF24L01::Recv(uint8_t *data, uint8_t *pipe_num)
{
	uint8_t num;

	if (conf.config.mode == RADIO_TX) {
		switch_mode(RADIO_RX, true);
		conf.interface.radio_en(true);
	}

	if (!read_irq(RX_DR_IRQ_MASK))
		return 0; /* FIFO empty */

	/* Calculate pipe num */
	num = read_reg(STATUS_REG);
	num &= PIPE_BITMASK;
	num >>= 1;

	if (pipe_num)
		*pipe_num = num;
	num = conf.rx_pipe_size[num];

	conf.interface.read(RX_PAYLOAD_CMD, data, num);

	/* Clear IRQ only if last data received (using FIFO) */
	if (read_reg(FIFO_STATUS_REG) & RX_FIFO_EMPTY_MASK) {
		clear_irq(RX_DR_IRQ_MASK);
		flush_buffer(FLUSH_RX_CMD);
	}

	/* Return number of bytes in pipe */
	return num;
}

void NRF24L01::switch_mode(enum radio_mode mode, bool power_enable)
{
	conf.config.mode = mode;
	conf.config.power_enable = power_enable;

	disable_radio();

	if (mode == RADIO_TX) {
		conf.config.tx_irq = true;
		conf.config.rx_irq = false;
		conf.config.max_rt_irq = conf.auto_retransmit.count;
	}
	else { /* RX MODE */
		conf.config.rx_irq = true;
		conf.config.tx_irq = false;
		conf.config.max_rt_irq = false;

		/* Turn on radio in RX mode */
		conf.interface.radio_en(power_enable);
	}

	update_config();
}

void NRF24L01::Sleep(void)
{
	conf.interface.radio_en(false);
	switch_mode(conf.config.mode, false);
}

void NRF24L01::WakeUp(void)
{
	switch_mode(conf.config.mode, true);

	if (conf.config.mode == RADIO_RX)
		conf.interface.radio_en(true);
}

uint8_t NRF24L01::Init()
{
	disable_radio();
	write_reg(EN_AA_REG, *(uint8_t *)&conf.auto_acknowledgment);
	write_reg(EN_RXADDR_REG, *(uint8_t *)&conf.enabled_rx_addresses);
	write_reg(SETUP_AW_REG, *(uint8_t *)&conf.address_widths);
	write_reg(SETUP_RETR_REG, *(uint8_t *)&conf.auto_retransmit);
	write_reg(RF_CH_REG, conf.channel & 0x7F);
	write_reg(RF_SETUP_REG, *(uint8_t *)&conf.setup);
	write_address(RX_ADDR_P0_REG, conf.rx_address_p0);
	write_address(RX_ADDR_P1_REG, conf.rx_address_p1);
	write_address(TX_ADDR_REG, conf.tx_address);
	write_reg(RX_ADDR_P2_REG, conf.rx_address_p2);
	write_reg(RX_ADDR_P3_REG, conf.rx_address_p3);
	write_reg(RX_ADDR_P4_REG, conf.rx_address_p4);
	write_reg(RX_ADDR_P5_REG, conf.rx_address_p5);
	write_reg(RX_PW_P0_REG, MIN(conf.rx_pipe_size[0], MAX_PIPE_SIZE));
	write_reg(RX_PW_P1_REG, MIN(conf.rx_pipe_size[1], MAX_PIPE_SIZE));
	write_reg(RX_PW_P2_REG, MIN(conf.rx_pipe_size[2], MAX_PIPE_SIZE));
	write_reg(RX_PW_P3_REG, MIN(conf.rx_pipe_size[3], MAX_PIPE_SIZE));
	write_reg(RX_PW_P4_REG, MIN(conf.rx_pipe_size[4], MAX_PIPE_SIZE));
	write_reg(RX_PW_P5_REG, MIN(conf.rx_pipe_size[5], MAX_PIPE_SIZE));

	switch_mode(conf.config.mode, conf.config.power_enable);

	return conf.interface.error;
}
