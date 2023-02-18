/*
 * Part of librfm3 (a utility library built on librfn and libopencm3)
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <librfm3/i2c_ctx.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <librfn/regdump.h>
#include <librfn/time.h>
#include <librfn/util.h>

#define I2C_WRITE			0
#define I2C_READ            1
uint16_t i2caddr;


#define D(x) { #x, x }
static const regdump_desc_t i2c_isr_desc[] = { { "I2C_ISR", 0 },
					       D(I2C_ISR_ALERT),
					       D(I2C_ISR_TIMEOUT),
					       D(I2C_ISR_PECERR),
					       D(I2C_ISR_OVR),
					       D(I2C_ISR_NACKF),
					       D(I2C_ISR_ARLO),
					       D(I2C_ISR_BERR),
					       D(I2C_ISR_STOPF),
					       D(I2C_ISR_TC),
					       D(I2C_ISR_ADDR),
					       { NULL, 0 } };
#undef D

#define D(x) { #x, x }
static const regdump_desc_t i2c_cr1_desc[] = { { "I2C_CR1", 0 },
					       D(I2C_CR1_TXIE),
					       D(I2C_CR1_RXIE),
					       D(I2C_CR1_SBC),
					       { NULL, 0 } };
#undef D

static bool i2c_ctx_is_timed_out(i2c_ctx_t *c)
{
	if (cyclecmp32(time_now(), c->timeout) > 0) {
		if (c->verbose) {
			printf("I2C TRANSACTION TIMED OUT\n");
			regdump(I2C_CR1(c->i2c), i2c_cr1_desc);
		}

		return true;
	}

	return false;
}

void i2c_ctx_init(i2c_ctx_t *c, uint32_t pi2c)
{
	memset(c, 0, sizeof(*c));

	c->i2c = pi2c;
	c->timeout = time_now() + 100000;
}

void i2c_ctx_reset(i2c_ctx_t *c)
{
	/* TODO: Latest opencm3 has this code built in */
	switch (c->i2c) {
	case I2C1:
		rcc_periph_reset_pulse(RST_I2C1);
		rcc_periph_reset_pulse(RST_I2C1);
		break;
	case I2C2:
		rcc_periph_reset_pulse(RST_I2C2);
		rcc_periph_reset_pulse(RST_I2C2);
		break;
#ifdef STM32F4
	case I2C3:
		rcc_periph_reset_pulse(RST_I2C3);
		rcc_periph_reset_pulse(RST_I2C3);
		break;
#endif
	}

	/* freq's numeric value ends up in MHz (i.e. in this case, 30) */
//#ifdef STM32F4
//	uint16_t freq = I2C_CR2_FREQ_30MHZ;
//#else
//	uint16_t freq = 36;
//#endif

	/* CCR is the number of APB bus cycles in *half* an I2C bus
	 * cycle. For Sm (100Khz) this ends up as:
	 *   freq * 1MHz / 2 * 100KHz
	 *   freq * 1000000 / 200000
	 *   freq * 5
	 *
	 * Similar trise is the number of APB bus cycles in the rise
	 * time (plus 1). For Sm (1us) this ends up as:
	 *   freq * 1Mhz / (1/1us)  + 1
	 *   freq * 1MHz / 1MHz  + 1
	 *   freq + 1
	 */

	/* peripheral configuration
	i2c_peripheral_disable(c->i2c);
	i2c_set_clock_frequency(c->i2c, freq);
	i2c_set_ccr(c->i2c, freq * 5);
	i2c_set_trise(c->i2c, freq + 1);
	i2c_set_own_7bit_slave_address(c->i2c, 0x32);
	i2c_peripheral_enable(c->i2c);
	*/

//	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, 0);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);
	i2c_set_7bit_addr_mode(I2C1);
	i2c_peripheral_enable(I2C1);
	i2c_set_own_7bit_slave_address(c->i2c, 0x00);
	
	 for (uint32_t loop = 0; loop < 150; ++loop) {
        __asm__("nop");
    } 
}

pt_state_t i2c_ctx_start(i2c_ctx_t *c, uint16_t addr, uint16_t size, int dir)
{
	PT_BEGIN(&c->leaf);
	i2c_clear_nack(c->i2c);
	i2c_clear_stop(c->i2c);
	i2c_set_7bit_address(c->i2c, addr);
	if (dir == 1) {
		i2c_set_read_transfer_dir(c->i2c);
	} else {
		i2c_set_write_transfer_dir(c->i2c);
	}
	i2c_set_bytes_to_transfer(c->i2c, size);
	i2c_disable_autoend(c->i2c);
	i2c_send_start(c->i2c);
    
    while (i2c_is_start(c->i2c)) {
        ;
	}

    if (i2c_nack(c->i2c) == 0) {
        ;   //todo i dunno something
    } else {
        i2c_send_stop(c->i2c);
        i2c_clear_stop(c->i2c);
        i2c_clear_nack(c->i2c);
        c->err = EIO;
    }
	PT_END();
}

pt_state_t i2c_ctx_sendaddr(i2c_ctx_t *c, uint16_t addr,
				   uint8_t bytes_to_read)
{
	PT_BEGIN(&c->leaf);

	c->bytes_remaining = bytes_to_read;

	i2c_set_7bit_address(c->i2c, addr);

	while (!i2c_ctx_is_timed_out(c) &&		// bad
	       !(I2C_CR1(c->i2c) & I2C_CR1_PE) &&	// bad
	       !(I2C_CR1(c->i2c) & I2C_ISR_ADDR))	// good
		PT_YIELD();

	if (!(I2C_CR1(c->i2c) & I2C_ISR_ADDR)) {
		i2c_ctx_reset(c);
		c->err = EIO;
	}

	/* If we are only reading one byte we must get ready to NACK the
	 * final byte.
	 */
	if (c->bytes_remaining == 1)
		I2C_CR1(c->i2c) &= ~I2C_CR2_NACK;
	else if (c->bytes_remaining >= 2)
		I2C_CR1(c->i2c) |= I2C_CR2_NACK;

	/* Read sequence has side effect or clearing I2C_CR1_ADDR */
	uint32_t reg32 __attribute__((unused));
	reg32 = I2C_CR2(c->i2c);

	if (c->bytes_remaining == 1)
		i2c_send_stop(c->i2c);

	PT_END();
}

pt_state_t i2c_ctx_senddata(i2c_ctx_t *c, uint8_t *data, uint16_t size)
{
	PT_BEGIN(&c->leaf);
	if (size != 0) {
    int i = 0;
	while (size--) {
	if (i2c_stop_detected(c->i2c)) {
	    printf("i2c send stop detected\r\n"); 
		/* Clear potential stop detection */
		i2c_clear_stop(c->i2c);
	}
	if (i2c_nack(c->i2c)) {
		/* Stop transaction on nack */
		printf("i2c nack detected stop\r\n");
		i2c_clear_nack(c->i2c);
		i2c_send_stop(c->i2c);
		c->err = EIO;
	}
	
	i2c_send_data(c->i2c, *data++);
	printf("sent data = 0x%04X\r\n", (uint8_t)data[i]);  //perfect delay will replace with register stuff
	i++;
    }   
 }
    while (!i2c_transfer_complete(c->i2c));

	PT_END();
}

pt_state_t i2c_ctx_getdata(i2c_ctx_t *c, uint8_t *data, uint16_t size)
{
	PT_BEGIN(&c->leaf);
	 if (size != 0) {
        
		for (size_t i = 0; i < size; i++) {
			while (i2c_received_data(c->i2c) == 0);
			data[i] = i2c_get_data(c->i2c);
			for (uint32_t loop = 0; loop < 550; ++loop) {
                __asm__("nop");
                } 
			printf("get data = 0x%04X\r\n", data[i]);  //simply handy
		}
		
    } 

	PT_END();
}

pt_state_t i2c_ctx_stop(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);

	if (i2c_nack(c->i2c)) {
		i2c_clear_nack(c->i2c);
		i2c_send_stop(c->i2c);
	} else {
	    i2c_send_stop(c->i2c);
    }

	PT_END();
}


