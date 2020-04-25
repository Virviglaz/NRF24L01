/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2020 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * nRF24L01 open source library/driver
 *
 * Contact Information:
 * Pavel Nadein <pavel.nadein@gmail.com>
 */

#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>

#define SPI_FREQ_MAX_HZ			8000000u
#define SPI_MODE				SPI_MODE_0
#define SPI_BITS_PER_WORD		8u

#define MIN(a,b) (((a)<(b))?(a):(b))

enum radio_mode { RADIO_TX, RADIO_RX };
enum crc_size { CRC_1B, CRC_2B };
enum address_size {
	ADDRESS_3_BYTES = 1,
	ADDRESS_4_BYTES = 2,
	ADDRESS_5_BYTES = 3,
};
enum tx_power { /* LNA gain =1 */
	P_18dBm = 0x01,
	P_12dBm = 0x03,
	P_6dBm = 0x05,
	P_0dBm = 0x07,
};
enum data_rate { R_1MPS, R_2MPS };

struct nrf24l01_conf
{
	/* Interface functions */
	struct
	{
		uint8_t(*write)(uint8_t reg, uint8_t *buf, uint8_t size);
		uint8_t(*read)(uint8_t reg, uint8_t *buf, uint8_t size);
		void(*radio_en)(bool state);
		bool(*read_irq)(void); /* Optional */
		uint8_t error;
	} interface;

	/* Configuration Register */
	struct
	{
		enum radio_mode mode : 1;
		bool power_enable : 1;
		enum crc_size crc_config : 1;
		bool crc_enable : 1;

		/* IRQ */
		bool max_rt_irq : 1;
		bool tx_irq : 1;
		bool rx_irq : 1;
	} config;

	/* Enhanced ShockBurst™ Auto Acknowledgment */
	struct
	{
		bool pipe0 : 1;
		bool pipe1 : 1;
		bool pipe2 : 1;
		bool pipe3 : 1;
		bool pipe4 : 1;
		bool pipe5 : 1;
	} auto_acknowledgment;

	/* Enabled RX Addresses */
	struct
	{
		bool pipe0 : 1;
		bool pipe1 : 1;
		bool pipe2 : 1;
		bool pipe3 : 1;
		bool pipe4 : 1;
		bool pipe5 : 1;
	} enabled_rx_addresses;

	/* Setup of Address Widths */
	struct
	{
		enum address_size address_len : 2;
	} address_widths;

	/* Setup of Automatic Retransmission */
	struct
	{
		uint8_t count : 4;
		uint8_t delay : 4;
	} auto_retransmit;

	uint8_t channel;

	/* RF Setup Register */
	struct
	{
		enum tx_power power : 3;
		enum data_rate data_rate : 1;
	} setup;

	/* Set to NULL if not used */
	uint8_t *rx_address_p0;
	uint8_t *rx_address_p1;
	uint8_t rx_address_p2;
	uint8_t rx_address_p3;
	uint8_t rx_address_p4;
	uint8_t rx_address_p5;
	uint8_t *tx_address;

	/* Size of buffer for each rx pipe */
	uint8_t rx_pipe_size[6];

	/* Decrement this variable outside by timer to limit execution time */
	volatile uint8_t *timeout_ms;

	/* Keep rx enabled after tx if needed */
	bool rx_state;

#ifndef __cplusplus


	/* Public functions */
	bool(*send) (uint8_t *dest, uint8_t *data, uint8_t size);
	uint8_t(*recv) (uint8_t *data, uint8_t *pipe_num);
	void(*mode) (enum radio_mode mode, bool power_enable);
	void(*sleep) (void);
	void(*wakeup) (void);
};

uint8_t nrf24l01_init(struct nrf24l01_conf *driver);
void spi_deinit(void);

#else /* __cplusplus */
};

#include <string.h>

class NRF24L01
{
public:
	/* Constructor empty class */
	NRF24L01()
	{
		memset(&conf, 0, sizeof(conf));
		conf.timeout_ms = &timeout_counter;
	};

	/* Constructor with interface */
	NRF24L01
	(
		uint8_t(*write) (uint8_t reg, uint8_t *buf, uint8_t size),
		uint8_t(*read) (uint8_t reg, uint8_t *buf, uint8_t size),
		void(*radio_en) (bool state),
		bool(*read_irq)(void)
	)
	{
		memset(&conf, 0, sizeof(conf));
		conf.interface.write = write;
		conf.interface.read = read;
		conf.interface.radio_en = radio_en;
		conf.interface.read_irq = read_irq;
		conf.timeout_ms = &timeout_counter;
	};

	/* Pointer to SPI write function */
	void set_write_func(uint8_t(*write) (uint8_t reg, uint8_t *buf, uint8_t size))
	{
		conf.interface.write = write;
	}

	/* Pointer to SPI read function */
	void set_read_func(uint8_t(*read) (uint8_t reg, uint8_t *buf, uint8_t size))
	{
		conf.interface.read = read;
	}

	/* Pointer to GPIO CE control function, can be dummy. You can just connect CE to VCC */
	void set_radio_en_func(void(*radio_en) (bool state))
	{
		conf.interface.radio_en = radio_en;
	}

	/* Optional. Pointer to read IRQ function. Can be NULL. Status reg will be used instead */
	void set_read_irq_func(bool(*read_irq)(void))
	{
		conf.interface.read_irq = read_irq;
	}

	/* Optional. To be used inside timer interrupt to limit execution time */
	volatile uint8_t *get_timeout_ptr()
	{
		return conf.timeout_ms;
	}

	/* Setup radio specific settings */
	void set_mode(enum radio_mode mode, bool enable, enum address_size size,
		enum tx_power power, enum data_rate rate, uint8_t channel)
	{
		conf.config.mode = mode;
		conf.config.power_enable = enable;
		conf.address_widths.address_len = size;
		conf.setup.power = power;
		conf.setup.data_rate = rate;
		conf.channel = channel;
	}

	/* Enable/disable retransmit for all pipes.
	 * Don't forget to enable ack for corresponding pipe */
	void set_retransmit(uint8_t count, uint8_t delay)
	{
		conf.auto_retransmit.count = count;
		conf.auto_retransmit.delay = delay;
	}

	/* CRC is used for automatic check data corruption.
	 * Turn off CRC for RX mode to get noise from air
	 * and check chip functionality if not sure it works */
	void set_crc(bool enable, enum crc_size crc_config)
	{
		conf.config.crc_enable = enable;
		conf.config.crc_config = crc_config;
	}

	/* Enable/disable RX pipe and acknowledge function */
	void set_pipe(uint8_t pipe_num, bool enable, bool auto_ack, uint8_t size)
	{
		switch (pipe_num)
		{
		case 0:
			conf.enabled_rx_addresses.pipe0 = enable;
			conf.auto_acknowledgment.pipe0 = auto_ack;
			break;
		case 1:
			conf.enabled_rx_addresses.pipe1 = enable;
			conf.auto_acknowledgment.pipe1 = auto_ack;
			break;
		case 2:
			conf.enabled_rx_addresses.pipe2 = enable;
			conf.auto_acknowledgment.pipe2 = auto_ack;
			break;
		case 3:
			conf.enabled_rx_addresses.pipe3 = enable;
			conf.auto_acknowledgment.pipe3 = auto_ack;
			break;
		case 4:
			conf.enabled_rx_addresses.pipe4 = enable;
			conf.auto_acknowledgment.pipe4 = auto_ack;
			break;
		case 5:
			conf.enabled_rx_addresses.pipe5 = enable;
			conf.auto_acknowledgment.pipe5 = auto_ack;
			break;
		}
		if (pipe_num < 6)
			conf.rx_pipe_size[pipe_num] = size;
	}

	/* Set 5 bytes address provided as the pointer */
	void set_tx_address(uint8_t *tx_address)
	{
		conf.tx_address = tx_address;
	}

	/* Set 5 bytes address provided as the pointer */
	void set_rx_address(uint8_t pipe_num, uint8_t *rx_address)
	{
		if (pipe_num)
			conf.rx_address_p1 = rx_address;
		else
			conf.rx_address_p0 = rx_address;
	}

	/* For pipes 2..5 only one bytes is used as address */
	void set_rx_address(uint8_t pipe_num, uint8_t rx_address)
	{
		switch (pipe_num)
		{
		case 2:
			conf.rx_address_p2 = rx_address; break;
		case 3:
			conf.rx_address_p3 = rx_address; break;
		case 4:
			conf.rx_address_p4 = rx_address; break;
		case 5:
			conf.rx_address_p5 = rx_address; break;
		}
	}

	/* Public functions */
	uint8_t Init();
	bool Send(uint8_t *dest, uint8_t *data, uint8_t size);
	uint8_t Recv(uint8_t *data, uint8_t *pipe_num);
	void Sleep(void);
	void WakeUp(void);

private:
	struct nrf24l01_conf conf;
	uint8_t timeout_counter;

	/* Internal functions */
	void write_reg(uint8_t reg, uint8_t value);
	uint8_t read_reg(uint8_t reg);
	void write_address(uint8_t reg, uint8_t *address);
	bool read_irq(uint8_t irq_mask);
	void clear_irq(uint8_t irq_mask);
	void flush_buffer(uint8_t cmd);
	void update_config(void);
	void disable_radio(void);
	void switch_mode(enum radio_mode mode, bool power_enable);
};

#endif /* __cplusplus */

#endif /* NRF24L01_H */
