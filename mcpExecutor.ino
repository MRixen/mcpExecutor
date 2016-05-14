#include <SPI.h>

const int CS_PIN_ADXL = 10;
const int CS_PIN_MCP2515 = 9;
const int START_PIN_IN = 7;
const int END_PIN_OUT = 8;

// CONTROL REGISTER
const byte CONTROL_REGISTER_BFPCTRL = 0x0C;
const byte CONTROL_REGISTER_TXRTSCTRL = 0x0D;
const byte CONTROL_REGISTER_CANSTAT = 0x0E;
const byte CONTROL_REGISTER_CANCTRL = 0x0F;
const byte CONTROL_REGISTER_TEC = 0x1C;
const byte CONTROL_REGISTER_REGISTER_REC = 0x1D;
const byte CONTROL_REGISTER_CNF3 = 0x28;
const byte CONTROL_REGISTER_CNF2 = 0x29;
const byte CONTROL_REGISTER_CNF1 = 0x2A;
const byte CONTROL_REGISTER_CANINTE = 0x2B;
const byte CONTROL_REGISTER_CANINTF = 0x2C;
const byte CONTROL_REGISTER_EFLG = 0x2D;
const byte CONTROL_REGISTER_TXB0CTRL = 0x30;
const byte CONTROL_REGISTER_TXB1CTRL = 0x40;
const byte CONTROL_REGISTER_TXB2CTRL = 0x50;
const byte CONTROL_REGISTER_RXB0CTRL = 0x60;
const byte CONTROL_REGISTER_RXB1CTRL = 0x70;

// REGISTER MCP2515
const byte REGISTER_TXB0SIDH = 0x31;
const byte REGISTER_TXB0SIDL = 0x32;
const byte REGISTER_TXB0DLC = 0x35;
const byte REGISTER_TXB0D0 = 0x36;
const byte REGISTER_TXB0D1 = 0x37;
const byte REGISTER_TXB0D2 = 0x38;
const byte REGISTER_TXB0D3 = 0x39;
const byte REGISTER_TXB0D4 = 0x3A;
const byte REGISTER_TXB0D5 = 0x3B;
const byte REGISTER_TXB0D6 = 0x3C;
const byte REGISTER_TXB0D7 = 0x3D;
const byte REGISTER_TXB0Dx[] = { REGISTER_TXB0D0, REGISTER_TXB0D1, REGISTER_TXB0D2, REGISTER_TXB0D3, REGISTER_TXB0D4, REGISTER_TXB0D5, REGISTER_TXB0D6, REGISTER_TXB0D7};
const byte REGISTER_TXB0SIDL_VALUE = 0x00;
const byte REGISTER_TXB0SIDH_VALUE = 0x01; // Identifier for the mcpExecutor
const byte CONTROL_REGISTER_CANSTAT_NORMAL_MODE = 0x00;
const byte CONTROL_REGISTER_CANSTAT_SLEEP_MODE = 0x20;
const byte CONTROL_REGISTER_CANSTAT_LOOPBACK_MODE = 0x40;
const byte CONTROL_REGISTER_CANSTAT_LISTEN_ONLY_MODE = 0x60;
const byte CONTROL_REGISTER_CANSTAT_CONFIGURATION_MODE = 0x80;
const byte CONTROL_REGISTER_CANINTE_INTE = 0x00;
const byte CONTROL_REGISTER_TXB0CTRL_VALUE = 0x00;

// Set values for bit timing
// Fosc = 8Mhz
// Tosc = 125ns
const byte CONTROL_REGISTER_VALUE_CNF1 = 0x03; // Baud rate prescaler calculated with application (Fosc = 8Mhz and CANspeed = 125kHz)
const byte CONTROL_REGISTER_VALUE_CNF2 = 0xB8; // BTLMODE = 1 (PHaseSegment 2 is configured with CNF 3) and PhaseSegment 1 = 8xTQ (7+1)
const byte CONTROL_REGISTER_VALUE_CNF3 = 0x05; // Set PhaseSegment 2 = 6xTQ (5+1)

// Set values for mode control
const byte CONTROL_REGISTER_CANCTRL_NORMAL_MODE = 0x00;
const byte CONTROL_REGISTER_CANCTRL_SLEEP_MODE = 0x20;
const byte CONTROL_REGISTER_CANCTRL_LOOPBACK_MODE = 0x40;
const byte CONTROL_REGISTER_CANCTRL_LISTEN_ONLY_MODE = 0x60;
const byte CONTROL_REGISTER_CANCTRL_CONFIGURATION_MODE = 0x80;

// SPI INSTRUCTIONS MCP2515
const byte SPI_INSTRUCTION_RESET = 0xC0;
const byte SPI_INSTRUCTION_READ = 0x03;
const byte SPI_INSTRUCTION_READ_RX_BUFFER0_SIDH = 0x90;
const byte SPI_INSTRUCTION_READ_RX_BUFFER0_D0 = 0x92;
const byte SPI_INSTRUCTION_WRITE = 0x02;
const byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_ID = 0x40;
const byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_DATA = 0x41;
const byte SPI_INSTRUCTION_RTS_BUFFER0 = 0x81;
const byte SPI_INSTRUCTION_RTS_BUFFER1 = 0x82;
const byte SPI_INSTRUCTION_RTS_BUFFER2 = 0x84;
const byte SPI_INSTRUCTION_READ_STATUS = 0xC0;
const byte SPI_INSTRUCTION_RX_STATUS = 0xC0;
const byte SPI_INSTRUCTION_BIT_MODIFY = 0xC0;


// REGISTER ADXL
const byte ACCEL_REG_POWER_CONTROL = 0x2D;  /* Address of the Power Control register                */
const byte ACCEL_REG_DATA_FORMAT = 0x31;    /* Address of the Data Format register                  */
const byte ACCEL_REG_X = 0x32; /* Address of the X Axis data register                  */
const byte ACCEL_REG_Y = 0x34; /* Address of the Y Axis data register                  */
const byte ACCEL_REG_Z = 0x36; /* Address of the Z Axis data register                  */
const byte ACCEL_SPI_RW_BIT = 0x80; /* Bit used in SPI transactions to indicate read/write  */
const byte ACCEL_SPI_MB_BIT = 0x40; /* Bit used to indicate multi-byte SPI transactions     */
const byte MESSAGE_SIZE_ADXL = 0x06;

byte ReadBuf[6]; // Read buffer of size 6 bytes (2 bytes * 3 axes)
byte retVal;

const int ADXL = 1;
const int MCP2515 = 2;
const int	NO_DEVICE = 3;


void setup()
{
	// Define I/Os
	pinMode(CS_PIN_ADXL, OUTPUT);
	pinMode(CS_PIN_MCP2515, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	pinMode(START_PIN_IN, INPUT);
	pinMode(END_PIN_OUT, OUTPUT);

	// Configure I/Os
	digitalWrite(CS_PIN_ADXL, HIGH);
	digitalWrite(CS_PIN_MCP2515, HIGH);
	digitalWrite(END_PIN_OUT, LOW);

	// Configure SPI
	SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
	SPI.begin();

	// Configure ADXL and MCP2515
	switchDevice(ADXL);
	initAdxl();
	switchDevice(MCP2515);
	//initMcp2515();
	delay(100);
	switchDevice(NO_DEVICE);

	// Give time to set up
	delay(100);
}

void loop()
{
	//if (digitalRead(START_PIN_IN) == 1)
	//{
		readAccel(); // Read sensor data
		//sendToMcp2515(); // Send sensor data to mcp2515
		//digitalWrite(END_PIN_OUT, HIGH);
		//waitFor(START_PIN_IN, 0); // Wait until master reset the signal -> Handshake
		//digitalWrite(END_PIN_OUT, LOW);
	//}
		//for (int i = 0; i < 6; i++)
		//{
	Serial.println("ReadBuffer: " + ReadBuf[0]);
		//}
		delay(1000);
}

void initAdxl() {
	writeToSpi(ACCEL_REG_DATA_FORMAT, 0x01, CS_PIN_ADXL);
	writeToSpi(ACCEL_REG_POWER_CONTROL, 0x08, CS_PIN_ADXL);
}

void initMcp2515() {

	switchDevice(MCP2515);

	// Reset chip to set in operation mode
	mcp2515_execute_reset_command();

	// Configure bit timing
	mcp2515_configureCanBus();

	// Configure interrupts
	mcp2515_configureInterrupts();

	// Configure bit masks and filters that we can receive everything 
	mcp2515_configureMasksFilters();

	// Set device to normal mode
	mcp2515_switchMode(CONTROL_REGISTER_CANSTAT_NORMAL_MODE, CONTROL_REGISTER_CANCTRL_NORMAL_MODE);

}

void mcp2515_execute_reset_command() {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage;

	writeSimpleCommandSpi(SPI_INSTRUCTION_RESET, CS_PIN_MCP2515);

	// Read the register value
	byte actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
	while (CONTROL_REGISTER_CANSTAT_CONFIGURATION_MODE != (CONTROL_REGISTER_CANSTAT_CONFIGURATION_MODE & actualMode))
	{
		actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
	}
}

void mcp2515_configureCanBus() {
	// Configure bit timing

	mcp2515_execute_write_command(CONTROL_REGISTER_CNF1, CONTROL_REGISTER_VALUE_CNF1, CS_PIN_MCP2515);

	mcp2515_execute_write_command(CONTROL_REGISTER_CNF2, CONTROL_REGISTER_VALUE_CNF2, CS_PIN_MCP2515);

	mcp2515_execute_write_command(CONTROL_REGISTER_CNF3, CONTROL_REGISTER_VALUE_CNF3, CS_PIN_MCP2515);
}

void mcp2515_configureInterrupts() {
	mcp2515_execute_write_command(CONTROL_REGISTER_CANINTE, CONTROL_REGISTER_CANINTE_INTE, CS_PIN_MCP2515);
}

void mcp2515_configureMasksFilters() {
	mcp2515_execute_write_command(CONTROL_REGISTER_TXB0CTRL, CONTROL_REGISTER_TXB0CTRL_VALUE, CS_PIN_MCP2515);
}

void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch){
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage[1];

	mcp2515_execute_write_command(CONTROL_REGISTER_CANCTRL, modeToSwitch, CS_PIN_MCP2515);

	// Read the register value
	byte actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
	while (modeToCheck != (modeToCheck & actualMode))
	{
		actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
	}
}

boolean waitFor(int cs_pin, int state) {
	while (digitalRead(cs_pin) == state)
	{
	}
	return true;
}

byte mcp2515_execute_read_command(byte registerToRead, int cs_pin)
{
	byte returnMessage;

	// Enable device
	digitalWrite(cs_pin, LOW);

	// Write spi instruction read  
	SPI.transfer(SPI_INSTRUCTION_READ);

	// Write the address of the register to read
	returnMessage = SPI.transfer(registerToRead);

	// Disable device
	digitalWrite(cs_pin, HIGH);

	return returnMessage;
}

void readAccel() {
	byte RegAddrBuf[] = { ACCEL_REG_X | ACCEL_SPI_RW_BIT | ACCEL_SPI_MB_BIT };

	switchDevice(ADXL);

	digitalWrite(CS_PIN_ADXL, LOW);
	SPI.transfer(RegAddrBuf[0]);
	for (int i = 0; i < 6; i++) {
		ReadBuf[i] = SPI.transfer(0x00); // Write 0 value to get data
		Serial.print("spi data: ");
		Serial.println(ReadBuf[i]);
	}
	digitalWrite(CS_PIN_ADXL, HIGH);
}

void sendToMcp2515() {

	switchDevice(MCP2515);

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(REGISTER_TXB0SIDL, REGISTER_TXB0SIDL_VALUE, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB0SIDH, REGISTER_TXB0SIDH_VALUE, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB0DLC, MESSAGE_SIZE_ADXL, CS_PIN_MCP2515);

	// Set data (message size of sensor -> 6 bytes) to tx buffer 0
	for (size_t i = 0; i < 6; i++)
	{
		mcp2515_execute_write_command(REGISTER_TXB0Dx[i], ReadBuf[i], CS_PIN_MCP2515);
	}

	// Send message
	mcp2515_execute_rts_command(0);
}

void writeToSpi(byte address, byte data, int cs_pin) {
	byte spiMessage[] = {address, data};

	digitalWrite(cs_pin, LOW);
	for (int i = 0; i < 2; i++) SPI.transfer(spiMessage[i]);
	digitalWrite(cs_pin, HIGH);
}

void mcp2515_execute_write_command(byte address, byte data, int cs_pin)
{	
	digitalWrite(cs_pin, LOW); // Enable device
	SPI.transfer(SPI_INSTRUCTION_WRITE); // Write spi instruction write  
	SPI.transfer(address);
	SPI.transfer(data);
	digitalWrite(cs_pin, HIGH); // Disable device
}

void mcp2515_execute_rts_command(int bufferId)
{
	byte spiMessage[1];

	switch (bufferId)
	{
	case 0:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER0;
		break;
	case 1:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER1;
		break;
	case 2:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER2;
		break;
	default:
		break;
	}
	writeSimpleCommandSpi(spiMessage[0], CS_PIN_MCP2515);
}

void writeSimpleCommandSpi(byte command, int cs_pin)
{
	digitalWrite(cs_pin, LOW);
	SPI.transfer(command);
	digitalWrite(cs_pin, HIGH);
}

void switchDevice(int device)  {
	switch (device)
	{
	case ADXL:
		pinMode(CS_PIN_MCP2515, INPUT);
		pinMode(CS_PIN_ADXL, OUTPUT);
		digitalWrite(CS_PIN_ADXL, HIGH);
		break;
	case MCP2515:
		pinMode(CS_PIN_ADXL, INPUT);
		pinMode(CS_PIN_MCP2515, OUTPUT);
		digitalWrite(CS_PIN_MCP2515, HIGH);
		break;
	case NO_DEVICE:
		pinMode(CS_PIN_ADXL, INPUT);
		pinMode(CS_PIN_MCP2515, INPUT);
		break;
	default:
		break;
	}
}