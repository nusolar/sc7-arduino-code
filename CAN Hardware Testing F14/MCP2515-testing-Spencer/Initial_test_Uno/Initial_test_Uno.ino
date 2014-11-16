//Author Spencer Williams
//Creation Date 10/30/14
//Current Version: 0.90
/*
Revision history:
0.9 - created with basic functions (10/30/14)
*/


/*
Hardware setup:
SPI testing using Arduino Uno:
    1-8: disconnected
    9: Ground
    10-12: disconnected
    13: SCK
    14: MOSI
    15: MISO
    16: CS
    17: disconnected
    18: 5V
*/


/*
Testing Logs:
10/30
    Testing of SPI communications by reading the mode of the 2515 without initialization. Should default to config mode (4).
    Hardware setup used: SPI testing
    Results:

*/


/*
To do list:
Debug functions
move into separate library
Clean up code so that it meet code requirements
*/

#include <SPI.h>

const int SS_pin = 10; //slave select pin

void setup()
{
	Serial.begin(9600);
	pinMode(SS_pin, OUTPUT);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    //init_2515();
}

void loop()
{
	byte mode;
	mode = read_mode();
	Serial.println(mode);
	delay(100);

}

//Functions
byte read_address(byte address)
{
    //read address function
    //procedure: lower CS pin -> send read instruction -> send address -> read output -> raise CS pin
    //(unimplemented) ability to read multiple bytes at the same time by sending more clock signals
	byte message;

    digitalWrite(SS_pin, LOW);

    //read instruction
	SPI.transfer(0x03);

    //send address
	SPI.transfer(address);

    //read output
	message = SPI.transfer(0x00);

    digitalWrite(SS_pin, HIGH);
	return message;
}

byte read_Rx_buffer(byte address)
{
    //reads Rx buffer and clears the corresponding receive flag
    //procedure: lower CS pin -> send read instuction -> read output -> raise CS pin
	byte message;
    digitalWrite(SS_pin, LOW);
    //send custom read instruction
	SPI.transfer(address);

    //read output
	message = SPI.transfer(0x00);
    
    digitalWrite(SS_pin, HIGH);
	return message;
}

byte read_status_bit()
{
    //reads certain status bit that are often used
    //procedure: lower CS pin ->send read status comand -> read output -> raise CS pin
    //(unimplemented) ability to read multiple bits at the same time by sending more clock signals

	byte message;
    digitalWrite(SS_pin, LOW);
    //send read status command
	SPI.transfer(0xA0);

    //read output
	message = SPI.transfer(0x00);
    digitalWrite(SS_pin, HIGH);
	return message;
}

byte get_Rx_Status()
{
    //quickly gets status data
    //procedure: lower CS pin -> send command byte -> read output -> raise CS pin
    //(unimplemented) ability to read multiple bits at the same time by sending more clock signals
	byte message;
    digitalWrite(SS_pin, LOW);
    //command byte
	SPI.transfer(0xB0);

    //read byte
	message = SPI.transfer(0x00);

    digitalWrite(SS_pin, HIGH);
	return message;
}


void write_address(byte address, byte message)
{
    //Write to address function
    //procedure: lower CS pin -> send write command-> send address -> send byte -> raise CS pin
    //(unimplemented) ability to write multiple bits at the same time by sending more clock signals

    digitalWrite(SS_pin, LOW);
    //send write command
	SPI.transfer(0x02);

    //send address
	SPI.transfer(address);

    //send byte
	SPI.transfer(message);
    
    digitalWrite(SS_pin, HIGH);
}

void write_Tx_buffer(byte address, byte message)
{
    //writes to the Tx buffer
    //procedure: lower CS pin -> send command byte -> send message -> raise CS pin
    //Note: unsure of the validity of this procedure, more testing required
    digitalWrite(SS_pin, LOW);
    
    //send command byte
	SPI.transfer(address);

    //send message
	SPI.transfer(message);

    digitalWrite(SS_pin, HIGH);
}

void modify_bit(byte address, byte mask, byte data)
{
    //sets or clears a certain bit
    //procedure: lower CS pin-> send bit modify command -> send address of register -> send mask byte -> send data byte -> raise CS pin
    //note that only certain bits can be set
    //Use this function CAREFULLY. using this on a non-bit-modifiable register will set the mask to FFh, causing a byte-write

    digitalWrite(SS_pin, LOW);

    //send bit modify command
	SPI.transfer(0x05);

    //send register address
    SPI.transfer(address);

    //send mask byte
    SPI.transfer(mask);

    //send data byte
    SPI.transfer(data);

    digitalWrite(SS_pin, HIGH);
}

void init_2515(byte CNF1_config, byte CNF2_config, byte CNF3_config, byte RTS_config, byte filter_num, byte mask_num, byte config_mode)
{
    //used to initialize the MCP2515 (currently loopback will be the only one implemented)
    //loopback is used to internally test the 2515 + Due setup by internally moving TX straight to RX (Don't try to use transceivers)
    //note that the only time that the MCP2515 can be initialized is in this function
    //Registers that can be configured are CNF1, CNF2, CNF3, TXRTSCTRL, Filter registers, and Mask registers
    //Because of the amount of potential filter + mask setups, I have implemented this using cases. Add a case when you have a different filter or mask that you want to use
	/*
	Filter case list:
	0 - no filters

	Mask case list:
	0 - no masks
	*/



    //After power-up or a reset, the 2515 is in config mode

    //CNF1 init (address 2Ah)
	write_address(0x2A, CNF1_config);

    //CNF2 init (address 29h)
	write_address(0x29, CNF2_config);

    //CNF3 init (address 28h)
	write_address(0x28, CNF3_config);

    //TXRTSCTRL init (address 0Dh)
    write_address(0x0D, RTS_config); //attempting xxxx x000

    //Filter init 
    switch(filter_num)
    {
    	case 0:
    	//no filters case
    	break;
    }
    //Mask init (unused for testing)
    switch(mask_num)
    {
    	case 0:
    	//no masks case
    	break;
    }

    //set operational mode CANCTRl.REQOP
    set_config_mode(config_mode);
}

void reset(byte CNF1_config, byte CNF2_config, byte CNF3_config, byte RTS_config, byte filter_num, byte mask_num, byte config_mode)
{
    //used to reset the 2515 over software
    //note that the 2515 must be reconfigured after this happens
    //Sends reset signal

    digitalWrite(SS_pin, LOW);

    SPI.transfer(0xC0);//reset = 1100 0000

    digitalWrite(SS_pin, HIGH);
	//reinitializes 2515
	init_2515(CNF1_config, CNF2_config, CNF3_config, RTS_config, filter_num, mask_num, config_mode);

}

byte read_mode()
{
    //reads the current mode of the MCP2515 by looking at the CANSTAT.OPMODE bits
	byte message;
	byte mode;
     //gets entire CAN status register
	message = read_address(0x0E);
    mode = message;// >> 5;//gets bits 5-7
    return mode;
}

void set_config_mode(byte config_mode)
{
    /*sets config mode in the CANCTRL register (0Fh)
    000 = Normal Operation
    001 = Sleep Mode
    010 = Loopback mode
    011 = Listen-only mode
    100 = Configuration mode
    config_mode #s:
    0 = Normal
    1 = Sleep
    2 = Loopback
    3 = Listen-only
    4 = Configuration*/
    byte message, cleared_config, new_config;
    byte shifted_config, clear_mask;
    //Read in bits
    message = read_address(0x0F);

    //bit math to set bits 7-5 to the config_mode
    shifted_config = config_mode << 5;
    clear_mask = 0x1F;
    cleared_config = clear_mask & message;//creates a hex value of the form 000x xxxx
    new_config = cleared_config | shifted_config;
    
    //Write bits
    write_address(0x0F, new_config);
}
