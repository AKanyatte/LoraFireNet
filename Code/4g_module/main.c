#include "usart.h"
#include "usart2.h"

int main()
{
	char* rx_command;
	//char mobile_num[] = "4039034943";
	char dw_num[] = "3065362241";
	char AT_command[80];
	uint8_t buffer[30] = {0};
	//uint8_t AT_is_OK = 0;
	
	clockInit();
	usart_config();
	usart2_config();
	serial_open();
	led_init();
	
	
	print_message("Testing UART Communication\r\n");
	

	sprintf(AT_command, "+++\r\n");
	msg_transmit((uint8_t*) AT_command, strlen(AT_command));
	sprintf(AT_command, "AT\r\n");
	msg_transmit((uint8_t*) AT_command, strlen(AT_command));
	print_message(AT_command);
	delay(1000);
	
	sprintf(AT_command, "ATE1\r\n");
	msg_transmit((uint8_t*) AT_command, strlen(AT_command));
	print_message(AT_command);
	delay(1000);
	
	msg_receive(buffer, sizeof(buffer));
	rx_command = (char*) buffer;
	print_message(rx_command);
	delay(1000);
	//CLI_Receive(rx_command, strlen(rx_command));

	sprintf(AT_command,"AT+CMGF=1\r\n");
	msg_transmit((uint8_t*) AT_command, strlen(AT_command));
	print_message(AT_command);
	delay(1000);
		
	sprintf(AT_command,"AT+CMGS=\"%s\"\r\n",dw_num);
	msg_transmit((uint8_t*) AT_command, strlen(AT_command));
	print_message(AT_command);
	delay(1000);
		
	sprintf(AT_command,"STM Test%c",0x1a);
	msg_transmit((uint8_t*) AT_command, strlen(AT_command));
		
	delay(2000);

}
	