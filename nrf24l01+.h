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

#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>

#define SPI_FREQ_MAX_HZ			8000000u
#define SPI_MODE				SPI_MODE_0
#define SPI_BITS_PER_WORD		8u

#ifdef __cplusplus
extern "C" {
#endif

enum radio_mode { RADIO_TX, RADIO_RX };

struct nrf24l01_conf
{
	/* Interface functions */
	struct
	{
		uint8_t (*write) (uint8_t reg, uint8_t *buf, uint8_t size);
		uint8_t (*read) (uint8_t reg, uint8_t *buf, uint8_t size);
		void (*radio_en) (bool state);
		bool (*read_irq) (void); /* Optional */
		uint8_t error;
	} interface;

	/* Configuration Register */
	struct
	{
		enum radio_mode mode : 1;
		bool power_enable : 1;
		enum { CRC_1B, CRC_2B } crc_config : 1;
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
		enum {
			ADDRESS_3_BYTES = 1,
			ADDRESS_4_BYTES = 2,
			ADDRESS_5_BYTES = 3,
		} address_len : 2;
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
		enum { /* LNA gain =1 */
			P_18dBm = 0x01,
			P_12dBm = 0x03,
			P_6dBm = 0x05,
			P_0dBm = 0x07,
		} power : 3;
		enum { R_1MPS, R_2MPS } data_rate : 1;
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

	/* Public functions */
	bool (* send) (uint8_t *dest, uint8_t *data, uint8_t size);
	uint8_t (* recv) (uint8_t *data, uint8_t *pipe_num);
	void (* mode) (enum radio_mode mode, bool power_enable);
	void (* sleep) (void);
	void (* wakeup) (void);
};

uint8_t nrf24l01_init (struct nrf24l01_conf *driver);
void spi_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01_H */
