#include "lora.hpp"

#include "definitions.hpp"
#include "spi.hpp"
#include "uart.hpp"


void init_lora(){
	spi_write8(RegOpMode_ADR, 0x00 | (1 << 3)); // sleep mode
	spi_write8(RegOpMode_ADR, 0x80 | (1 << 3)); // LoRa sleep mode
	spi_write8(RegOpMode_ADR, 0x81 | (1 << 3)); // LoRa standby mode

	spi_write8(RegMaxPayloadLength_ADR, 0x80);

	spi_write8(RegIrqFlagsMask_ADR, 0b10010111);

	// Set RegModemConfig1 to Default values
	spi_write8(RegModemConfig1_ADR, 0x72);
	// Set RegModemConfig2 to Default values
	spi_write8(RegModemConfig2_ADR, 0x70);
	// Set RegModemConfig2 to Default values
	spi_write8(RegModemConfig3_ADR, 0x00);

#if DEBUG_MODE
	send_debug("Init LORA Done!");
#endif
}

void lora_cont_recv(){
	spi_write8(RegFifoAddrPtr_ADR, 0x00);
	spi_write8(RegFifoRxBaseAddr_ADR, 0x00);
	spi_write8(RegFifoRxByteAddr_ADR, 0x00);
	spi_write8(RegOpMode_ADR, 0x85 | (1 << 3)); // selecting LoRa mode and RXCONT

	while(true){
		uint8_t irq = spi_read8(RegIrqFlags_ADR);
		if (irq & (1 << 6)){
			send_debug("Data Received!!");
			if (irq & (1 << 5))
				send_error("CRC Error!!");

			uint8_t rx_nb_bytes = spi_read8(RegRxNbBytes_ADR);
			//uint8_t rx_curr_adr = spi_read8(FifoRxCurrentAddr_ADR);
			//spi_write8(RegFifoAddrPtr_ADR, rx_curr_adr);
			spi_write8(RegFifoAddrPtr_ADR, 0x00);

			uint8_t recv_data[0xFF];
			//spi_read(RegFifo_ADR, rx_nb_bytes, recv_data);
			for(int i = 0; i < rx_nb_bytes; i++) recv_data[i] = spi_read8(RegFifo_ADR);
			spi_write8(RegFifoAddrPtr_ADR, 0x00);

			for(int i = 0; i < rx_nb_bytes; i++)
				send_char(recv_data[i]);

			/* DELETE LATER */
			/*spi_write8(RegFifoAddrPtr_ADR, 0x80);
			uint8_t recv_data[0x80];
			spi_read(RegFifo_ADR, 0x80, recv_data);
			spi_write8(RegFifoAddrPtr_ADR, 0x80); // NOTE: base RX addr
			for(int i = 0; i < 0x80; i++)
				send_char(recv_data[i]);*/


			send_data("\r\n");
			send_data("\r\n");

			spi_write8(RegIrqFlags_ADR, 0xFF);
		}
		/*else{
			send_int(irq);
			send_data(" - ");
			uint8_t op_mode = spi_read8(RegOpMode_ADR);
			send_int(op_mode);
			send_data("\r\n");

			wait_with_timer2(100); // 10ms wait
		}*/
	}
}

void lora_send(uint8_t sz, uint8_t *data){
	//send_debug("LoRa Send Start!!");
	spi_write8(RegOpMode_ADR, 0x81 | (1 << 3)); // selecting LoRa mode and Standby
	spi_write8(RegFifoAddrPtr_ADR, 0x00);
	spi_write8(RegFifoTxBaseAddr_ADR, 0x00);
	for(int i = 0; i < sz; i++) spi_write8(RegFifo_ADR, data[i]);
	//spi_write(RegFifo_ADR, sz, data);
	spi_write8(RegPayloadLength_ADR, sz);
	spi_write8(RegFifoAddrPtr_ADR, 0x00);
	spi_write8(RegOpMode_ADR, 0x83 | (1 << 3)); // selecting LoRa mode and TX

	while(true){
		uint8_t irq = spi_read8(RegIrqFlags_ADR);
		if(irq & (1 << 3)) break;

		/*send_int(irq);
		send_data(" - ");
		uint8_t op_mode = spi_read8(RegOpMode_ADR);
		send_int(op_mode);
		send_data("\r\n");

		wait_with_timer2(100); // 10ms wait*/
	}

	spi_write8(RegIrqFlags_ADR, 0xFF);
	//send_debug("LoRa Send Done!!");
}
