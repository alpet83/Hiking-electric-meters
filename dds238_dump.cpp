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
 * Alexander Petrov 2020 modified this program for dumping values from DDS238-2 energy meter
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
#include <unistd.h>

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

int ask_meter(modbus_t *ctx)
{
  int nreg = 0;
  // struct tm;
  char ts[32];
  time_t t;
  time(&t);
  strftime(ts, 32, "%Y-%m-%d %H:%M:%S", localtime(&t));

  printf ("[%s]. #DBG: trying read registers\n", ts);

  nreg = modbus_read_registers(ctx, 0, 0x15, tab_reg);
  if (nreg == -1)
  {

    fprintf(stderr, "Error reading registers: %s\n", modbus_strerror(errno));
    return -1;
  }

  const char *fname = "/tmp/em_data_last.json";

  FILE *json = fopen(fname, "w");
  if (!json)
  {
    fprintf(stderr, "Cannot open file [%s], may not have privileges\n", fname);
    return -1;
  }

  assert(json);

  fputs("{", json);

  int i;
  int ofs = 0;
  // dump all registers content

  // fprintf(stderr, " Non-zero register dump:\n");
  for (i = 0; i < nreg; i++)
    if (tab_reg[i])
      printf("  reg #0x%02x: %d\n", i, tab_reg[i]);

  // all in kW*H
  fputs(format_float("total_energy",       u32_val(0), 0.01f), json);  
  fputs(format_float("reversing_energy",   u32_val(0x08), 0.01f), json); // 0x08-0x09
  fputs(format_float("forward_energy",     u32_val(0x0a), 0.01f), json); // 0x0a-0x0b 
  ofs = 0x0c;
  fputs(format_float("voltage", tab_reg[ofs++], 0.1f), json);  // 0x0c
  fputs(format_float("current", tab_reg[ofs++], 0.01f), json); // 0x0d

  fputs(format_float("active_power",            tab_reg[ofs++], 0.001f), json); // 0x0e
  fputs(format_float("reactive_power", (int16_t)tab_reg[ofs++], 0.001f), json); // 0x0f
  fputs(format_float("power_factor",            tab_reg[ofs++], 0.001f), json); // 0x0f

  fputs(format_float("frequency", tab_reg[0x11], 0.01f), json);

  fprintf(json, "\"ts\":\"%s\"", ts);
  fputs("}\n", json);
  fclose(json);

  // fast replacing
  unlink("/tmp/em_data.json");
  return rename(fname, "/tmp/em_data.json");
}

int main(int argc, char *argv[])
{
  modbus_t *ctx = 0;
  bool loop_mode = false;
  bool debug_mode = false;

  fprintf(stderr, "program used modbus library version %s\n", LIBMODBUS_VERSION_STRING);

  for (int n = 1; n < argc; n++)
  {
    const char *arg = argv[n];
    if (arg && strstr(arg, "-loop"))
      loop_mode = true;
    if (arg && strstr(arg, "-debug"))
      debug_mode = true;
  }

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
  fprintf(stderr, "Connected\n");

  // enable debug
  modbus_set_debug(ctx, debug_mode);

  // initialize timeouts with default
#ifdef LIBMODBUS_NEW
  modbus_get_response_timeout(ctx, &old_rts, &old_rtu);
  // set the message and charcater timeout to 5 seconds
  modbus_set_response_timeout(ctx, 1, 0);
  modbus_set_byte_timeout(ctx, 1, 0);
#else
  modbus_get_response_timeout(ctx, &ts);
  ts.tv_sec = 1;
  ts.tv_usec = 0;
  // set the message and charcater timeout to 1 seconds
  modbus_set_response_timeout(ctx, &ts);
  modbus_set_byte_timeout(ctx, &ts);
#endif

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

  int errors = 0;

  do
  { 
    sleep(1);   
    int res = ask_meter(ctx);
    
    if (res < 0) 
        errors ++;
    else  
        errors = 0; 
        
    printf("ack_meter returned %d, errors %d\n", res, errors);  
  } while (loop_mode && errors < 10);

  puts("program complete!\n");

  if (ctx)
  {
    modbus_close(ctx);
    modbus_free(ctx);
  }
  return 0;
} // main function