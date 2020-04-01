/*
 *
 * Andrea Montefusco 2018
 *
 * Test program for DZT 6001: 80A, Single phase kWh meter, LCD, RS485/Modbus
 *
 * Home page: https://www.dutchmeters.com/index.php/product/dzt-6001/
 * User manual: http://dutchmeters.com/wp-content/uploads/2017/06/DZT-6001-manual.pdf
 * Register reference:  http://dutchmeters.com/wp-content/uploads/2017/04/DZT6001-Modbus.pdf
 *
 * Prerequisite: install libmodbus
 *
 * sudo apt-get install libmodbus-dev
 *
 * Compile and run with:
 *
 * gcc -Wall -I/usr/include/modbus test_dzt6001.c  -lmodbus  -o test_dzt6001 && ./test_dzt6001
 *
 * This program has been slightly modified from:
 *
 * https://electronics.stackexchange.com/questions/136646/libmodbus-trouble-with-reading-from-slave
 *
 *  
 * Alexander Petrov 2020 modified this program for dumping values from DTS238-7 energy meter 
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <cstring>
#include <stdbool.h>
#include <time.h>
#include <modbus.h>
#include <cassert>

#if LIBMODBUS_VERSION_HEX >= 0x0301000
 uint32_t old_rts, old_rtu;
 #define LIBMODBUS_NEW
#else
 timeval ts;
#endif


uint16_t tab_reg[256];


const char *format_float(const char *key, float value, float coef, bool add_comma = 1)
{
  static char buff[256];  
  float fv = coef * (float)value;

  int cb = sprintf(buff, "\"%s\":%.3f", key, fv);
  if (add_comma)
    strcat(&buff[cb], ",");

  return buff;
}


uint32_t u32_val(int ofs)
{
  return (tab_reg[ofs] << 16) + tab_reg[ofs + 1];
}
int32_t i32_val(int ofs)
{
  return (tab_reg[ofs] << 16) | tab_reg[ofs + 1];
}


int main()
{
  modbus_t *ctx = 0;

  fprintf(stderr, "program used modbus library version %s\n", LIBMODBUS_VERSION_STRING);

  //
  // create a libmodbus context for RTU
  // doesn't check if the serial is really there
  //
  ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);

  if (ctx == 0)
  {

    fprintf(stderr, "Unable to create the libmodbus context\n");
    return -1;
  }
  else
  {

    // enable debug
    modbus_set_debug(ctx, true);

    // initialize timeouts with default
#ifdef LIBMODBUS_NEW
    modbus_get_response_timeout(ctx, &old_rts, &old_rtu);
	// set the message and charcater timeout to 2 seconds
	modbus_set_response_timeout(ctx, 2, 0);
	modbus_set_byte_timeout(ctx, 2, 0);
#else
	modbus_get_response_timeout(ctx, &ts);
	ts.tv_sec = 2;
	ts.tv_usec = 0;
	// set the message and charcater timeout to 2 seconds
	modbus_set_response_timeout(ctx, &ts);
	modbus_set_byte_timeout(ctx, &ts);
#endif

  }

  // try to connet to the first DZT on the line
  // assume that line address is 1, the default
  // send nothing on the line, just set the address in the context
  if (modbus_set_slave(ctx, 1) == -1)
  {
    fprintf(stderr, "Didn't connect to slave/n");
    return -1;
  }

  // establish a Modbus connection
  // in a RS-485 context that means the serial interface is opened
  // but nothing is yet sent on the line
  if (modbus_connect(ctx) == -1)
  {

    fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
    return -1;
  }

  int nreg = 0;

  fprintf(stderr, "Connected\n");
  
  //
  // read all registers in DVT 6001
  // the function uses the Modbus function code 0x03 (read holding registers).
  //
  const char *fname = "/tmp/em_data.json";

  FILE *json = fopen(fname, "w");
  if (!json)
  {
    fprintf(stderr, "Cannot open file [%s], may not have privileges\n", fname);
    return -1;
  }

  assert(json);

  fputs("{", json);
  

  int start = 0;
  for (start = 0; start < 192; start += 128)
  {
    printf("#DBG: trying read from offset 0x%02x\n", start);
    nreg = modbus_read_registers(ctx, start, 0x20, tab_reg);

    if (nreg == -1)
    {

      fprintf(stderr, "Error reading registers: %s\n", modbus_strerror(errno));
      modbus_close(ctx);
      modbus_free(ctx);

      return -1;
    }
    else
    {
      int i;
      int ofs = 0;
      // dump all registers content

      fprintf(stderr, " Non-zero register dump:\n");
      for (i = 0; i < nreg; i++)
        if (tab_reg[i])
          printf("  reg #0x%02x: %d\n", i, tab_reg[i]);

      if (0x00 == start)
      {        
        fputs(format_float("total_energy",       u32_val(0), 0.01f), json);  
        fputs(format_float("reversing_energy",   u32_val(0x08), 0.01f), json); // 0x08-0x09
        fputs(format_float("forward_energy",     u32_val(0x0a), 0.01f), json); // 0x0a-0x0b 

        // ofs = 9;
        // fputs(format_float("kWh_A", tab_reg[ofs++], 0.01f), json);
        // fputs(format_float("kWh_B", tab_reg[ofs++], 0.01f), json);
        // fputs(format_float("kWh_C", tab_reg[ofs++], 0.01f), json);
        ofs = 0x11;
        fputs(format_float("frequency", tab_reg[ofs], 0.01f), json);
        uint16_t flags = tab_reg[0x15];
        const char* baud_rates[8] = { "unknown", "9600", "4800", "2400", "1200", "5???", "6???", "7???" };        
        fprintf(json, "\"baud_rate\":\"%s\",", baud_rates[flags & 7]); // low byte
        fprintf(json, "\"comm_addr\":%d,", flags >> 8); // high byte
      } 
      else
      if (0x80 == start) 
      { // part 3phase decoding
        fputs(format_float("voltage_A", tab_reg[ofs++], 0.1f), json);
        fputs(format_float("voltage_B", tab_reg[ofs++], 0.1f), json);
        fputs(format_float("voltage_C", tab_reg[ofs++], 0.1f), json);
        fputs(format_float("current_A", tab_reg[ofs++], 0.01f), json);
        fputs(format_float("current_B", tab_reg[ofs++], 0.01f), json);
        fputs(format_float("current_C", tab_reg[ofs++], 0.01f), json);
        
        // active power
        ofs = 6;
        fputs(format_float("power",   i32_val(ofs), 0.001f), json);
        ofs += 2;
        fputs(format_float("power_A", (int16_t)tab_reg[ofs++], 0.001f), json);
        fputs(format_float("power_B", (int16_t)tab_reg[ofs++], 0.001f), json);
        fputs(format_float("power_C", (int16_t)tab_reg[ofs++], 0.001f), json);
        // reactive power
        fputs(format_float("rpower",   i32_val(ofs), 0.001f), json);
        ofs += 2;
        fputs(format_float("rpower_A", (int16_t)tab_reg[ofs++], 0.001f), json);
        fputs(format_float("rpower_B", (int16_t)tab_reg[ofs++], 0.001f), json);
        fputs(format_float("rpower_C", (int16_t)tab_reg[ofs++], 0.001f), json);
        // apparent power
        fputs(format_float("ap_power",   u32_val(ofs), 0.001f), json);
        ofs += 2;
        fputs(format_float("ap_power_A", tab_reg[ofs++], 0.001f), json);
        fputs(format_float("ap_power_B", tab_reg[ofs++], 0.001f), json);
        fputs(format_float("ap_power_C", tab_reg[ofs++], 0.001f), json);

        ofs += 2; 
        ofs = 0x15;
        fputs(format_float("pfact",   tab_reg[ofs++], 0.001f), json);
        fputs(format_float("pfact_A", tab_reg[ofs++], 0.001f), json);
        fputs(format_float("pfact_B", tab_reg[ofs++], 0.001f), json);
        fputs(format_float("pfact_C", tab_reg[ofs++], 0.001f), json);

      }
    }
  } // for start - 3 loops
  // struct tm;
  char ts[32];
  time_t t;
  time(&t);
  strftime(ts, 32, "%Y-%m-%d %H:%M:%S", localtime(&t));
  fprintf(json, "\"ts\":\"%s\"", ts);
  fputs("}\n", json);
  fclose(json);
  puts("program complete!\n");

  modbus_close(ctx);
  modbus_free(ctx);
  return 0;
} // main function