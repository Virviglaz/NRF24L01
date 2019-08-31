# NRF24L01

Cross-platform written in plain C NRF24L01 library

## Installing

Just clone repo files inside your project folder

$git clone https://github.com/Virviglaz/NRF24L01.git .

### Prerequisites

This is a cross-platform open source library which is not related to any hw or platform. To use SPI and GPIO interfaces
you need to provide them as the functions to info strusture according to example below.

## Example of use

```C
/* This functions is just a pointing library to your hw interface */
uint8_t write_reg(uint8_t reg, uint8_t *buf, uint8_t size)
{
	error = spi.send_reg(reg, buf, size); // -> this goes to your local spi driver
	return 0;
}

uint8_t read_reg (uint8_t reg, uint8_t *buf, uint8_t size)
{
	error = spi.recv_reg(reg, buf, size);
	return 0;
}

void radio_en (bool state)
{
	gpio.enable(NRF24L01_EN_PIN); // -> this goes to your local gpio driver
}

/* STM32 example of radio_en function */
static void radio_en (bool state)
{
	if (state)
		GPIO_SetBits(RF_CE_PIN);
	else
		GPIO_ResetBits(RF_CE_PIN);
}
```
### Init
```C
/* Local variable (can be a pointer as well) */
struct nrf24l01_conf radio;
...
void init (void)
{
	memset(&radio, 0, sizeof(radio));
	
	/* interface */
	radio.interface.write = spi_write;
	radio.interface.read = spi_read;
	radio.interface.radio_en = radio_en;
	radio.interface.read_irq = read_irq;
	
	/* config */
	radio.config.mode = RADIO_TX;
	radio.config.power_enable = true;
	radio.config.crc_config = CRC_1B;
	radio.config.crc_enable = true;
	radio.config.rx_irq = true;
	radio.config.tx_irq = true;
	radio.enabled_rx_addresses.pipe0 = true;
	radio.address_widths.address_len = ADDRESS_5_BYTES;
	radio.channel = 72;
	radio.setup.data_rate = R_2MPS;
	radio.setup.power = P_0dBm;
	
	/* pipes */
	radio.rx_address_p0 = (uint8_t *)"HALLO";
	radio.tx_address = radio.rx_address_p0;
	radio.rx_pipe_size[0] = 32;
	radio.read_cnt = 100000; //this value limits the waiting for IRQ time in case of chip failure
	
  /* Init radio */
	nrf24l01_init(&radio);
  
  /* Let's be online */
	radio_en(true);
}
```
### Usage
```C
/* Send 32 bytes of data */
bool tx_res; // will become 'true' if send successfull
tx_res = radio.send(tx_address, rxbuf, 32); //tx_address can be NULL to use address provided before:
tx_res = radio.send(NULL, rxbuf, 32); //send to radio.tx_address
```
```C
uint8_t pipe_n; // can be NULL. Pipe num will be returned
radio.recv(rx_buf, &pipe_n); // will return true if we have a data received
```
## Testing
Tested with OrangePi PC2, STM32F103 and STM8
Achived performance: STM32F103 TX -> RX OrangePi PC2:
  32 bytes x 952 packets per second = 30464 bytes/sec (29.8 kB/s) with non-ack mode

## Authors

* **Pavel Nadein** - [Virviglaz](https://github.com/Virviglaz)
