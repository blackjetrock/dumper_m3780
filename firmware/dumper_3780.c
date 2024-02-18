////////////////////////////////////////////////////////////////////////////////
//
//  Dumper for M3780 Microcontrollers
//
////////////////////////////////////////////////////////////////////////////////
//
// Uses Sean Riddle's dumper circuit, but with a RP Pico instead of the
// PIC. I ported Sean's code over to the Pico but couldn't get reliable
// reads, so I rewrote it to use a generic system of injecting instructions
// and used a more elaborate scheme to get the data out of the device.
// 
//
////////////////////////////////////////////////////////////////////////////////

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/structs/bus_ctrl.h"

#define DEBUG_STOP while(1) {}

// Some logic to analyse:
#include "hardware/structs/pwm.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

////////////////////////////////////////////////////////////////////////////////

#define DUMP_CODE      1
#define RAM_TEST_CODE  0

#if 0
// Gives about 2MHz clock
#define DUMP_ON_CLK   0
#define DELAY_CLK     2
#define DELAY_PORT    1
#define DELAY_VOLTS   1
#endif

int delay_clock_var = 2;

#if 1

#define DUMP_ON_CLK   0
#define DELAY_CLK     delay_clock_var
#define DELAY_PORT    1
#define DELAY_VOLTS   1
#endif

////////////////////////////////////////////////////////////////////////////////

#define CLOCK_DIV  30.0f

#define PORT_WIDTH 8

const int P_PORT5_0    = 0;
const int P_PORT5_1    = 1;
const int P_PORT5_2    = 2;
const int P_PORT5_3    = 3;
const int P_PORT5_4    = 4;
const int P_PORT5_5    = 5;
const int P_PORT5_6    = 6;
const int P_PORT5_7    = 7;

const int P_PORT4_0    = 8;
const int P_PORT4_1    = 9;
const int P_PORT4_2    = 10;
const int P_PORT4_3    = 11;
const int P_PORT4_4    = 12;
const int P_PORT4_5    = 13;
const int P_PORT4_6    = 14;
const int P_PORT4_7    = 15;

const int P_RESET      = 16;
const int P_STROBE     = 17;
const int P_XTAL2      = 18;

const int P_DRV_3V5    = 19;
const int P_DRV_7V0    = 20;
const int P_XTAL1      = 21;
const int P_EXT_INT    = 22;
const int P_PORT0_2    = 26;
const int P_PORT0_1    = 27;
const int P_PORT0_0    = 28;

typedef enum
  {
   TEST_PIN_FLOATING = 100,
   TEST_PIN_3V5,
   TEST_PIN_7V0,
  } TEST_PIN_VOLTAGE;
  

// Define some GPIO ports

const int port4[PORT_WIDTH] =
  {
   P_PORT4_0,
   P_PORT4_1,
   P_PORT4_2,
   P_PORT4_3,
   P_PORT4_4,
   P_PORT4_5,
   P_PORT4_6,
   P_PORT4_7,
  };

const int port5[PORT_WIDTH] =
  {
   P_PORT5_0,
   P_PORT5_1,
   P_PORT5_2,
   P_PORT5_3,
   P_PORT5_4,
   P_PORT5_5,
   P_PORT5_6,
   P_PORT5_7,
  };

// Opcodes (inverted)
#define DCI_OPCODE  (0x2A ^ 0xFF)
#define LM_OPCODE   (0x16 ^ 0xFF)
#define LR_OPCODE   (0x0E ^ 0xFF)
#define NOP_OPCODE  (0x2B ^ 0xFF)
#define ST_OPCODE   (0x17 ^ 0xFF)
#define LI_OPCODE   (0x20 ^ 0xFF)
#define INC_OPCODE  (0x1F ^ 0xFF)

////////////////////////////////////////////////////////////////////////////////
//
// Prototypes
//
////////////////////////////////////////////////////////////////////////////////

void serial_help(void);
void set_test_pin_voltage(TEST_PIN_VOLTAGE v);
int get_data_bus(void);
void detect_device(void);
void dump_rom(void);
void run_instructions(void);
int reset_and_detect(void);
void enable_rom(void);
void disable_rom(void);
void sweep_address(void);

volatile int perform_dump = 0;

////////////////////////////////////////////////////////////////////////////////
// Serial loop command structure
////////////////////////////////////////////////////////////////////////////////
  
typedef void (*FPTR)(void);

typedef struct
{
  char key;
  char *desc;
  FPTR fn;
} SERIAL_COMMAND;

int keypress = 0;
int parameter = 0;
int auto_increment_parameter = 0;
int auto_increment_address   = 0;

#define RAM_SIZE  (10*1024)

uint8_t ram_data[RAM_SIZE];
int dump_size = 0;

#define TEXT_PARAMETER_LEN 40

char text_parameter[TEXT_PARAMETER_LEN+1] = "";

int address = 0;

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

// Delay for less than a us
// This has to be tuned

void delay(int n)
{
  volatile int i;

  for(i=0; i<n; i++)
    {
    }
}

void set_gpio_input(int gpio_pin)
{
  gpio_init(gpio_pin);
  gpio_set_dir(gpio_pin, GPIO_IN);
  gpio_set_pulls(gpio_pin, 0, 0);

  printf("\nGPIO %d set as input", gpio_pin);
}

void set_gpio_output(int gpio_pin)
{
  gpio_init(gpio_pin);
  gpio_set_dir(gpio_pin, GPIO_OUT);
  gpio_put(gpio_pin, 0);
  printf("\nGPIO %d set as output", gpio_pin);
}

void set_port_input(const int port[PORT_WIDTH])
{
  for(int i=0; i<PORT_WIDTH; i++)
    {
      set_gpio_input(port[i]);
    }
}

void set_port_output(const int port[PORT_WIDTH])
{
  for(int i=0; i<PORT_WIDTH; i++)
    {
      set_gpio_output(port[i]);
    }
}

int read_port(const int port[PORT_WIDTH])
{
  int res = 0;

  delay(DELAY_PORT);
  
  for(int i=0; i<PORT_WIDTH; i++)
    {
      res |= (gpio_get(port[i]) << i);
    }

  return(res);
}

void write_port(const int port[PORT_WIDTH], int value)
{
  
  for(int i=0; i<PORT_WIDTH; i++)
    {
      if( value & (1<<i) )
	{
	  gpio_put(port[i], 1);
	}
      else
	{
	  gpio_put(port[i], 0);
	}
    }

  delay(DELAY_PORT);
}

void clock_low(void)
{
  gpio_put(P_XTAL2, 0);
  delay(DELAY_CLK);

#if DUMP_ON_CLK
  printf("\n   CLK LO DB:%02X", get_data_bus());
#endif
}



void clock_high(void)
{
  gpio_put(P_XTAL2, 1);

  delay(DELAY_CLK);

#if DUMP_ON_CLK
  printf("\n   CLK HI DB:%02X", get_data_bus());
#endif
}

void clock(void)
{
  clock_low();
  clock_high();
}

void double_clock(int n)
{
  for(int i=0; i<n; i++)
    {
      clock();
      clock();
    }
}


void set_data_bus(int value)
{
  write_port(port5, value);
}

int get_data_bus(void)
{
  return(read_port(port4));
}


void reset_low(void)
{
  gpio_put(P_RESET, 0);
}

void reset_high(void)
{
  gpio_put(P_RESET, 0);
}


void reset_device(void)
{
  printf("\nresetting device... ");
  reset_low();
  sleep_ms(300);
  reset_high();

  printf("  done.");
}


void display_ram_at(uint8_t *dat)
{
  int csum = 0;
  
  printf("\n");
  
  for(int z = 0; z<dump_size; z++)
    {
      int byte = 0;
      
      if( (z % 16) == 0)
	{
	  printf("\n%03X: ", z);
	}

      csum += *dat;
      
      printf("%02X ", *(dat++));
    }
  
  printf("\n");
  printf("\nCSUM:%08X\n", csum);

}

void display_ram(void)
{
  display_ram_at(&(ram_data[0]));
}


void cli_boot_mass(void)
{
  reset_usb_boot(0,0);
}

// Another digit pressed, update the parameter variable
void cli_digit(void)
{
  parameter *= 10;
  parameter += keypress-'0';
}

void cli_zero_parameter(void)
{
  parameter = 0;
}

void cli_set_address(void)
{
  address = parameter;
}

void cli_information(void)
{
  printf("\n");

  //  printf("\nNumber of reads  :%d", num_rd);
  //printf("\nNumber of writes :%d", num_wr);
  //  printf("\nNumber of loops: %d", loop_count);
  printf("\n");
}

void cli_text_param(void)
{
  int key;
  int done = 0;
  int i = 0;

  printf("\nType string and end with RETURN...\n");
  
  while( !done )
    {
      if( ((key = getchar_timeout_us(1000)) != PICO_ERROR_TIMEOUT))
	{
	  printf("%c", key);
	  
	  if( key == 13 )
	    {
	      done = 1;
	      continue;;
	    }

	  text_parameter[i++] = key;
	  
	  if( i >= (TEXT_PARAMETER_LEN-1) )
	    {
	      i--;
	    }
	}
    }
  
  text_parameter[i] = '\0';
}

void cli_set_test_3v5(void)
{
  set_test_pin_voltage(TEST_PIN_3V5);
}

void cli_set_test_7v0(void)
{
  set_test_pin_voltage(TEST_PIN_7V0);
}

void cli_set_test_float(void)
{
  set_test_pin_voltage(TEST_PIN_FLOATING);
}

////////////////////////////////////////////////////////////////////////////////

SERIAL_COMMAND serial_cmds[] =
  {
   {
    'h',
    "Serial command help",
    serial_help,
   },
   {
    '?',
    "Serial command help",
    serial_help,
   },
   {
    'I',
    "Information",
    cli_information,
   },
   {
    'u',
    "Dump ROM",
    dump_rom,
   },
   {
    'd',
    "Display RAM",
    display_ram,
   },
   {
    'D',
    "Detect device",
    detect_device,
   },
   {
    'A',
    "Set TEST pin to 3V5",
    cli_set_test_3v5,
   },
   {
    'B',
    "Set TEST pin to 7V0",
    cli_set_test_7v0,
   },
   {
    'F',
    "Set TEST pin to float",
    cli_set_test_float,
   },
   
   {
    'S',
    "Sweep address",
    sweep_address,
   },
   {
    'r',
    "Run instructions",
    run_instructions,
   },
   {
    'z',
    "Zero parameter",
    cli_zero_parameter,
   },
   {
    'A',
    "Set Address",
    cli_set_address,
   },
   {
    '0',
    "*Digit",
    cli_digit,
   },
   {
    '1',
    "*Digit",
    cli_digit,
   },
   {
    '2',
    "*Digit",
    cli_digit,
   },
   {
    '3',
    "*Digit",
    cli_digit,
   },
   {
    '4',
    "*Digit",
    cli_digit,
   },
   {
    '5',
    "*Digit",
    cli_digit,
   },
   {
    '6',
    "*Digit",
    cli_digit,
   },
   {
    '7',
    "*Digit",
    cli_digit,
   },
   {
    '8',
    "*Digit",
    cli_digit,
   },
   {
    '9',
    "*Digit",
    cli_digit,
   },
   {
    't',
    "Enter Text Parameter",
    cli_text_param,
   },
   {
    '!',
    "Boot to mass storage",
    cli_boot_mass,
   },
  };



void serial_help(void)
{
  printf("\n");
  
  for(int i=0; i<sizeof(serial_cmds)/sizeof(SERIAL_COMMAND);i++)
    {
      if( *(serial_cmds[i].desc) != '*' )
	{
	  printf("\n%c:   %s", serial_cmds[i].key, serial_cmds[i].desc);
	}
    }
  printf("\n0-9: Enter parameter digit");
}


void prompt(void)
{
  printf("\n\n(Text Parameter:'%s'", text_parameter);
  printf("\n(Parameter:%d (%04X) %c, Address:%d (%04X) %c) >",
	 parameter, parameter, auto_increment_parameter?'A':' ',
	 address,   address,   auto_increment_address?  'A':' ');
}


////////////////////////////////////////////////////////////////////////////////
//
// Serial CLI Handling
//
////////////////////////////////////////////////////////////////////////////////

int pcount = 0;
int periodic_read = 0;

void serial_loop()
{
  int  key;
  
  if( ((key = getchar_timeout_us(1000)) != PICO_ERROR_TIMEOUT))
    {
      for(int i=0; i<sizeof(serial_cmds)/sizeof(SERIAL_COMMAND);i++)
	{
	  if( serial_cmds[i].key == key )
	    {

	      keypress = key;
	      (*serial_cmds[i].fn)();
	      prompt();
	      break;
	    }
	}
    }
  else
    {
      // I have found that I need to send something if the serial USB times out
      // otherwise I get lockups on the serial communications.
      // So, if we get a timeout we send a spoace and backspace it. And
      // flush the stdio, but that didn't fix the problem but seems like a good idea.
      stdio_flush();
      printf(" \b");
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// TEST pin drive. Only one voltage can be selected at a time
// as we don't want the two 317ts to have their outputs connected
// together
//
////////////////////////////////////////////////////////////////////////////////

void set_test_pin_voltage(TEST_PIN_VOLTAGE v)
{
  switch(v)
    {
    case TEST_PIN_FLOATING:
      gpio_put(P_DRV_7V0, 1);
      gpio_put(P_DRV_3V5, 1);
      delay(DELAY_VOLTS);
      break;
      
    case TEST_PIN_3V5:
      gpio_put(P_DRV_7V0, 1);
      gpio_put(P_DRV_3V5, 0);
      delay(DELAY_VOLTS);
      break;
      
    case TEST_PIN_7V0:
      gpio_put(P_DRV_3V5, 1);
      gpio_put(P_DRV_7V0, 0);
      delay(DELAY_VOLTS);
      break;
      
    default:
      printf("\nAttempted to drive an unsupported volatge to TEST pin\n");
      break;
    }
}

void detect_device(void)
{
  // Set reset low
  reset_low();

  // take test pin to 7V0 to disable ROM
  cli_set_test_7v0();

  //////////////////////////////AAAA//////////////////////////////////////////////
  //
  // Set a value on the data bus

  printf("\nSetting data bus to 0xA3...");
  set_data_bus(0xA3);

  int dbval = 0;
  
  if( (dbval = get_data_bus()) == 0x5C )
    {
      printf("\nDetected device. Data bus value = %02X", dbval );
    }
  else
    {
      printf("\nDid not detect device");
      printf("\n  Expected 0x5C on bus, read 0x%02X instead", dbval);
    }

  //////////////////////////////BBBB//////////////////////////////////////////////

  set_data_bus(0xFF);

  set_test_pin_voltage(TEST_PIN_3V5);

  reset_low();

}

int reset_and_detect(void)
{
  int result = 0;
  
  // Set reset low
  reset_low();

  // take test pin to 7V0 to disable ROM
  cli_set_test_7v0();

  // Set a value on the data bus

  //  printf("\nSetting data bus to 0xA3...");
  set_data_bus(0xA3);
  
  int dbval = 0;
  
  if( (dbval = get_data_bus()) == 0x5C )
    {
      // All OK
      result = 1;

      // printf("\nDetected device. Data bus value = %02X", dbval );
    }
  else
    {
      // Not detected
      //       printf("\nDid not detect device");
      //printf("\n  Expected 0x5C on bus, read 0x%02X instead", dbval);
    }

  set_data_bus(0xFF);
    
  // Clock while in reset
  //printf("\nClocking in reset...");
  double_clock(256);

  return(result);
}

////////////////////////////////////////////////////////////////////////////////

void release_reset(void)
{
  gpio_put(P_RESET, 1);
} 

////////////////////////////////////////////////////////////////////////////////
// Run instructions in the MK3780 by cloking it and waiting for strobe to go low
// This is an opcode load signal. We place the opcode on the bus and then clock
// the required number of cycles for that instruction, adding data if required (multi
// byte instructions). Strobe will then go low again and we load the next instruction

// Device is detected and reset at start of run

////////////////////////////////////////////////////////////////////////////////

#define MAX_INST_LEN 20

int inst_len_i = 0;
int inst_len[MAX_INST_LEN];
int first_strobe = 1;
int clock_cycle = 0;
int last_strobe = 1;
int run_inst_done = 0;
int ram_index= 0;
int stb_dat_i = 0;
volatile int grabbed_i = 0;
int got       = 0;

#define MAX_GRABBED   (1024)
volatile int grabbed[MAX_GRABBED];

int all_same = 1;
int first = -1;

#if DUMP_CODE
int stb_dat[] =
  {
   NOP_OPCODE, NOP_OPCODE, NOP_OPCODE, NOP_OPCODE, NOP_OPCODE, 
   DCI_OPCODE, 0xFF, 0xFF, 0xFF, 0xFF,
   DCI_OPCODE, 0xFF, 0xFF, (0x01 ^ 0xFF), 0xFF,
   LR_OPCODE,  0xFF, 0xFF,
   LM_OPCODE,  0xFF,
  };

#define ADDRL stb_dat[13]
#define ADDRH stb_dat[11]

int rom_enable[] =
  {
   0, 0, 0, 0, 0,
   0, 0, 0, 0, 0,
   0, 0, 0, 0, 0,
   0, 0, 0,
   0, 1
  };

int grab_data[] =
  {
   -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1,
   -1, -1, -1,
   -1, 1
  };
#endif

#if RAM_TEST_CODE
int stb_dat[] =
  {
   NOP_OPCODE, NOP_OPCODE, NOP_OPCODE, NOP_OPCODE, NOP_OPCODE, 
   DCI_OPCODE, 0xFF, 0xFF, 0xFF, 0xFF,
   DCI_OPCODE, (0x07 ^ 0xFF), 0xFF, (0xFF ^ 0xFF), 0xFF,
   LM_OPCODE,  0xFF,
   INC_OPCODE, 
   DCI_OPCODE, (0x07 ^ 0xFF), 0xFF, (0xFF ^ 0xFF), 0xFF,
   ST_OPCODE,  0xFF,
   DCI_OPCODE, (0x07 ^ 0xFF), 0xFF, (0xFF ^ 0xFF), 0xFF,
   LM_OPCODE,  0xFF,
  };

#define ADDRL stb_dat[13]
#define ADDRH stb_dat[11]

int rom_enable[] =
  {
   0, 0, 0, 0, 0,
   0, 0, 0, 0, 0,
   0, 0, 0, 0, 0,
   0, 1,
   0,
   0, 0, 0, 0, 0,
   0, 0,
   0, 0, 0, 0, 0,
   0, 1,
  };

int grab_data[] =
  {
   -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1,
   -1, 1,
   -1,
   -1, -1, -1, -1, -1,
   -1, -1,
   -1, -1, -1, -1, -1,
   -1, 1,

  };
#endif

#define STB_DAT_SIZE  (sizeof(stb_dat)/sizeof(int))
int next_stb_data(void)
{
  if( stb_dat_i >= STB_DAT_SIZE )
    {
      stb_dat_i = STB_DAT_SIZE-10;
    }

  // Enable rom if required
  if( rom_enable[stb_dat_i] )
    {
      enable_rom();
    }
  else
    {
      disable_rom();
    }
  
  //printf("\n    %02X", stb_dat[stb_dat_i]);          
  return(stb_dat[stb_dat_i++]);
}

void determine_cycle_count(int strobe)
{
  if( (strobe == 0) && (last_strobe == 1) )
    {
      // Disable ROM
      //  disable_rom();

      
      // Get data for STB read also enables/disables ROM
      set_data_bus(next_stb_data());

      if( inst_len_i < MAX_INST_LEN )
	{
	inst_len[inst_len_i++] = clock_cycle;
	}

      clock_cycle = 0;
      got = 0;
    }
  else
    {
      clock_cycle++;
    }

  last_strobe = strobe;
}

typedef struct _TRACE_S
{
  int cycle;
  int db;
  char *tag;
} TRACE_S;

#define NUM_TRACE 50000
//TRACE_S trace[NUM_TRACE];
int trace_i = 0;

void trace_this(int cyc, int databus, char *tag)
{
  if( trace_i < NUM_TRACE )
    {
#if 0
      trace[trace_i].cycle = cyc;
      trace[trace_i].db = databus;
      trace[trace_i].tag = tag;
#endif
      trace_i++;
    }
  else
    {
      run_inst_done = 1;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

#define MAX_SAME_DATA (4*1024)

int same_data[MAX_SAME_DATA];

void sweep_address(void)
{
  // Put address into the DCI instruction and run until we get all of the same value
  // back. Save that and move to next address

  for(int addr=0; addr<MAX_SAME_DATA; addr++)
    {
      ADDRH = (((addr & 0xFF00)>>8) ^ 0xFF);
      ADDRL = (((addr & 0x00FF)>>0)  ^ 0xFF);

      printf("\n");
      for(int j=0; j<20; j++)
	{

	  printf("%02X ", stb_dat[j]);
	}
      printf("\n");
      
      if( (addr % 16) == 0 )
	{
	  printf("\n%04X: ", addr);
	}

      int done = 0;
      
      while(!done)
	{
	  run_instructions();
	  if( all_same )
	    {
	      same_data[addr] = first;
	      done = 1;
	    }
	}
    }

  printf("\nFinal data:\n");
  
  for( int gg=0; gg<MAX_SAME_DATA; gg++)
    {
      if( (gg % 16) == 0 )
	{
	  printf("\n%04X: ", gg);
	}
      
      printf("%02X ", same_data[gg]);
    }

  printf("\n");

}

void run_instructions(void)
{
  inst_len_i    = 0;
  first_strobe  = 1;
  clock_cycle   = 0;
  last_strobe   = 1;
  run_inst_done = 0;
  ram_index     = 0;
  stb_dat_i     = 0;
  got           = 0;
  grabbed_i     = 0;
  trace_i       = 0;
  
  int databyte  = 0;
  
#if 0
  if( !reset_and_detect() )
    {
      printf("\n Device not detected\n");
      return;
    }
#else

  reset_low();

  disable_rom();
  set_data_bus(0xFF);
  
  double_clock(256);
  
#endif
  
  release_reset();
  
  while (!run_inst_done)
    {
      
      switch(clock_cycle)
	{
	case 3:
	  
	  // Enable the rom so it can be read
	  //enable_rom();
	  
	  // Remove the instruction code
	  set_data_bus(0xFF);
	  break;
	  
	case 3000:
	  // Something wrong
	  run_inst_done = 1;
	  break;
	}
      
      // Clock this cycle high
      clock_high();
      
      //printf("\n%3d: %02X  CLK H", clock_cycle, get_data_bus());
      trace_this(clock_cycle, databyte = get_data_bus(), "CLK H");

      // Strobe may have gone low, that would be an instruction fetch
      if( gpio_get(P_STROBE) == 0 )
	{
	  if( first_strobe )
	    {
	      first_strobe = 1;
	      determine_cycle_count(0);
	      
	      //printf("\n%3d: %02X  STB", clock_cycle, get_data_bus());
	      trace_this(clock_cycle, get_data_bus(), "STB");
	    }
	  else
	    {
	      //printf("\n%3d: %02X  STB", clock_cycle, get_data_bus());
	      trace_this(clock_cycle, get_data_bus(), "STB");
	      set_data_bus(0xFF);	      
	    }
	}
      else
	{
	  determine_cycle_count(1);
	}
      
      clock_low();
      
      //printf("\n%3d: %02X  CLK L", clock_cycle, get_data_bus());
      trace_this(clock_cycle, get_data_bus(), "CLK L");
      
      if( gpio_get(P_STROBE) == 0 )
	{
	  //determine_cycle_count(0);
	  //printf("\n%3d: %02X  STB", clock_cycle, get_data_bus());
	  trace_this(clock_cycle, databyte = get_data_bus(), "STB");

	  // grab the data if required
	  if( grab_data[stb_dat_i] != -1 )
	    {
	      if( !got && (grab_data[stb_dat_i] == clock_cycle) )
		{
		  // Grab another value
		  if( grabbed_i < MAX_GRABBED )
		    {
		      grabbed[grabbed_i++] = databyte;
		    }
		  got = 1;
		}
	    }
	  
	}
      else
	{
	  //determine_cycle_count(1);
	}
    }

#if 0
  printf("\n");
  
  for(int ii=0; ii<trace_i; ii++)
    {
      printf("\n%3d: %02X  %s", trace[ii].cycle, trace[ii].db, trace[ii].tag);
    }
  
  printf("\n\nCycle Lengths\n");
  
  for(int i=0; i<MAX_INST_LEN; i++)
    {
      printf("\n%3d: %d", i, inst_len[i]);
    }
  
  printf("\n");
#endif
  
  // Dump grabbed data

  printf("\nData\n");

  first = grabbed[0];
  all_same = 1;
  
  for( int gg=0; gg<grabbed_i; gg++)
    {
      if( grabbed[gg]!= first )
	{
	  all_same = 0;
	  printf("gg=%04X", gg);
	}
      
      if( (gg % 16) == 0 )
	{
	  printf("\n%04X: ", gg);
	}
      
      printf("%02X ", grabbed[gg]);
    }

  printf("\nAll same: %s", all_same?"YES":"NO");
  printf("\n");
  
}


void enable_rom(void)
{
  set_test_pin_voltage(TEST_PIN_3V5);
}

void disable_rom(void)
{
  cli_set_test_7v0();
}


////////////////////////////////////////////////////////////////////////////////

void do_dump_core(void)
{
  
  // Set reset low
  reset_low();

  // take test pin to 7V0 to disable ROM
  cli_set_test_7v0();

  //////////////////////////////AAAA//////////////////////////////////////////////
  //
  // Set a value on the data bus

  //  printf("\nSetting data bus to 0xA3...");
  set_data_bus(0xA3);
  
  int dbval = 0;
  
  if( (dbval = get_data_bus()) == 0x5C )
    {
      printf("\nDetected device. Data bus value = %02X", dbval );
    }
  else
    {
      //      printf("\nDid not detect device");
      printf("\n  Expected 0x5C on bus, read 0x%02X instead", dbval);
    }

  //////////////////////////////BBBB//////////////////////////////////////////////

  set_data_bus(0xFF);
    
  // Clock while in reset
  //printf("\nClocking in reset...");
  double_clock(256);

  //////////////////////////////CCCC//////////////////////////////////////////////
  
  // Release reset
  //printf("\nReleasing reset...");
  gpio_put(P_RESET, 1);
  
  //printf("\n  done.");

  ///////////////////////////////DDDD/////////////////////////////////////////////
  
  // Wait for STROBE and align clock
  int done = 0;
  int halfoff = 0;

  //printf("\nWaiting for STROBE...");
  
  while( !done )
    {
      clock_low();
      
      if( gpio_get(P_STROBE) == 0 )
	{
	  done = 1;
	  halfoff = 0;
	}

      clock_high();
      
      if( gpio_get(P_STROBE) == 0 )
	{
	  done = 1;
	  halfoff = 0;
	}

      clock_low();
      
      if( gpio_get(P_STROBE) == 0 )
	{
	  done = 1;
	  halfoff = 1;
	}

      clock_high();
      
      if( gpio_get(P_STROBE) == 0 )
	{
	  done = 1;
	  halfoff = 1;
	}
    }
  
  //printf("\n  done.");
  
  if( halfoff )
    {
#if DEBUG_DUMP
      printf("\nAligning to half clock...");
#endif
      clock();
    }

  /////////////////////////////////EEEE///////////////////////////////////////////
  
  //printf("\nFirst instruction fetch...");

  // Waiting for strobe is compiled out in Sean's code

  double_clock(2);
  double_clock(2);
  set_data_bus(DCI_OPCODE);

  //////////////////////////////////FFFF//////////////////////////////////////////
  
  // Set address of DCI instruction
  //set_data_bus(0xFF);

  //double_clock(20);
#if 0
  double_clock(2);
#else
  done = 0;
  int strobe = 0;
  
  while( !done)
    {
      double_clock(1);
      
      if( (strobe = gpio_get(P_STROBE)) == 0 )
	{
	  done = 1;
	}

      //printf("\nStrobe = %d", strobe);
      double_clock(1);

      if( (strobe = gpio_get(P_STROBE)) == 0 )
	{
	  done = 1;
	}

    }
#endif

  //////////////////////////////////GGGG//////////////////////////////////////////
  
  set_data_bus(0xFF);
  double_clock(20);

  //////////////////////////////HHHH//////////////////////////////////////////////

  double_clock(2);
  set_data_bus(LM_OPCODE);
  double_clock(2);
    
  set_test_pin_voltage(TEST_PIN_3V5);

  //////////////////////////////////////IIIIIIIIIII///////////////////////////////
  
  //printf("\nReading ROM...");
  int x = 0;
  int y = 0;
  int z = 0;
  
  int xstart = 0;
  int bytes = 0;
  
  while (x<(48+xstart))
    {
      while( y<64 )
	{
	  // Don't mask the data bus
	  set_data_bus(0xFF);
	  
	  //------------------------------------------------------------------------------	  

	  clock( );

	  //------------------------------------------------------------------------------	  
	  y=y+1;
	  
	  double_clock(3);

	  //------------------------------------------------------------------------------
	  
	  clock_low();
	  
	  //printf("\nAAAA");
	  
	  int flag1=1;
						  
	  clock_high();

	  // test an extra setup delay
	  clock_high();
	  
	  //------------------------------------------------------------------------------
	  
	  if( x >= xstart)
	    {
#if 0
	      if( (bytes % 16) == 0 )
		{
		  printf("\n%04X", bytes);
		}
#endif
	      
	      int byte = get_data_bus();

	      //printf(" %02X", byte);
	      ram_data[bytes] = byte;
	      
	      bytes++;
	    }

	  //------------------------------------TTTTTT------------------------------------
	  
	  clock_low();

	  flag1=0;
	  clock_high();
	  clock_low();        
	  clock_high();

	  //------------------------------------------------------------------------------

	  //;disable ROM so we can force opcode during instruction fetch
	  set_test_pin_voltage(TEST_PIN_7V0);
	  //printf("\n    7V0");
	  clock_low();
	  clock_high();	  

	  if( x>=xstart )
	    {
	      z=z+1;
	    }
	  
	  //------------------------------------------------------------------------------

	  double_clock(1);

	  //------------------------------------------------------------------------------

	  clock_low();
	  clock_high();	  

	  flag1=1;
	  clock_low();
	  
	  //-------------------------------UUUU-------------------------------------------
	  //printf("\n    ASSERT D5");
	  
	  set_data_bus(LM_OPCODE);
	  
	  clock_high();

	  flag1=0;

	  double_clock(2);

	  set_test_pin_voltage(TEST_PIN_3V5);
#if DEBUG_DUMP
	  //printf("\n    3V5");
#endif
	}
      
      y=0;
      x=x+1;
    }

  set_test_pin_voltage(TEST_PIN_3V5);

  reset_low();

#if DEBUG_DUMP
  //printf("\n  done.");
#endif
  dump_size = bytes;

  int csum = 0;
  
  for(int y=0; y<bytes; y++)
    {
      csum += ram_data[y];
    }

  printf("    CSUM:%08X", csum);
}

void core1_main(void)
{
  while(1)
    {
      if( perform_dump )
	{
	  perform_dump = 0;
	  do_dump_core();
	}
    }
}

void do_dump2(void)
{
  for(delay_clock_var=1; delay_clock_var<50; delay_clock_var++)
    {
      printf("\nDelaylock var= %d", delay_clock_var);
      for(int xx=0; xx< 16; xx++)
	{
	  do_dump_core();
	}
    }
}


////////////////////////////////////////////////////////////////////////////////

void dump_rom(void)
{
  perform_dump = 1;
}


////////////////////////////////////////////////////////////////////////////////
//
//
//
////////////////////////////////////////////////////////////////////////////////

int main()
{
  int gpio_states;

  stdio_init_all();
  sleep_ms(2000);
  
  // Make sure voltage drives are off

  printf("\nTurning TEST pin drive off...");

  set_gpio_output(P_DRV_3V5);
  set_gpio_output(P_DRV_7V0);

  gpio_put(P_DRV_3V5, 1);
  gpio_put(P_DRV_7V0, 1);

  printf("\n");
  printf("\n***********************");
  printf("\n* 3780 Dumper         *");
  printf("\n***********************");
  printf("\n");
  
  // Set up GPIOs

  printf("\nSetting GPIOs...");
  set_port_input(port4);
  set_port_output(port5);

  
  set_gpio_output(P_RESET);
  set_gpio_output(P_XTAL2);
  set_gpio_input(P_STROBE);

  reset_low();
  
  ////////////////////////////////////////////////////////////////////////////////
  //
  // Overclock as needed
  //
  ////////////////////////////////////////////////////////////////////////////////
  
  //#define OVERCLOCK 135000
  //#define OVERCLOCK 200000
#define OVERCLOCK 270000
  //#define OVERCLOCK 360000
  
#if OVERCLOCK > 270000
  /* Above this speed needs increased voltage */
  vreg_set_voltage(VREG_VOLTAGE_1_20);
  sleep_ms(1000);
#endif
  
  /* Overclock */
  set_sys_clock_khz( OVERCLOCK, 1 );

  multicore_launch_core1(core1_main);
    
  while(1)
    {
      serial_loop();
    }
}
