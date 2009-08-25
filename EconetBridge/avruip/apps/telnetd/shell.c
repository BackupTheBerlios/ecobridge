 /*
 * Copyright (c) 2003, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: shell.c,v 1.4 2009/08/25 19:07:40 markusher Exp $
 *
 */

#include "shell.h"

#include "adlc.h"

#include <string.h>

struct ptentry {
  char *commandstr;
  void (* pfunc)(char *str);
};

#define SHELL_PROMPT "> "

/*---------------------------------------------------------------------------*/
static void
parse(register char *str, struct ptentry *t)
{

  struct ptentry *p;

  for(p = t; p->commandstr != NULL; ++p) {
    if(strncmp(p->commandstr, str, strlen(p->commandstr)) == 0) {
      break;
    }
  }

  p->pfunc(str);


}
/*---------------------------------------------------------------------------*/
static void
inttostr(char *str, uint8_t i)
{
  str[0] = '0' + i / 100;
  if(str[0] == '0') {
    str[0] = ' ';
  }
  str[1] = '0' + (i / 10) % 10;
  if(str[0] == ' ' && str[1] == '0') {
    str[1] = ' ';
  }
  str[2] = '0' + i % 10;
  str[3] = ' ';
  str[4] = 0;
}

#define hexdigit(x)	(((x) < 10) ? (x) + '0' : (x) + 'a' - 10)

static void
hextostr(char *str, uint8_t i)
{
  str[0] = hexdigit(i >> 4);
  str[1] = hexdigit(i & 0xf);
}

/*---------------------------------------------------------------------------*/
static void
help(char *str)
{
//  shell_output("set clck [econet clock multiplier 1-4]", "");
  shell_output("\nset sttn [Econet station 1-254]", "");
  shell_output("\nset enet [Econet network 1-127]", "");
  shell_output("\nset aunn [AUN network 128-251]", "");
  shell_output("\nset ipad [Ethernet IP address  aaa.bbb.ccc.ddd]", "");
  shell_output("\nset snet [Ethernet subnet mask aaa.bbb.ccc.ddd]", "");
  shell_output("\nset gway [Ethernet gateway IP  aaa.bbb.ccc.ddd]", "");
  shell_output("\nset maca [Ethernet MAC address 0a:0b:0c:0d:0e:0f]\n\n", "");
  shell_output("\nset ecip [Econet IP address  aaa.bbb.ccc.ddd]", "");
  shell_output("\nset ecsb [Econet subnet mask aaa.bbb.ccc.ddd]", "");
  shell_output("\nset ewan [Econet WAN gateway IP  aaa.bbb.ccc.ddd]", "");

  shell_output("stats   - show network statistics", "");
  shell_output("config  - show configuration", "");
  shell_output("help, ? - show help", "");
  shell_output("exit    - exit shell", "");
}
/*---------------------------------------------------------------------------*/
static void
setvalue(char *str)
{

  char *cmd;
  char *value;

  strlcpy(cmd, str, 4);

  switch (*cmd) {
/*    case 'clck':
      break;
*/
    case 'sttn':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.Station = cleanvaluebyte(value);
    break;
    case 'enet':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.Econet_Network = cleanvaluebyte(value);
    break;
    case 'aunn':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.Ethernet_Network = cleanvaluebyte(value);
    break;
    case 'ipad':
	strlcpy(value, str+10, 2);
	eeGlobals.IPAddr = cleanvalueip(value);
    break;
    case 'snet':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.Subnet = cleanvalueip(value);
    break;
    case 'gway':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.Gateway = cleanvalueip(value);
    break;
    case 'maca':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.MAC = cleanvaluemac(value);
    break;
    case 'ecip':
	strlcpy(value, str+10, 2);
	eeGlobals.IPAddr = cleanvalueip(value);
    break;
    case 'ecsb':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.Subnet = cleanvalueip(value);
    break;
    case 'ewan':
	strlcpy(value, str+10, strlen(str));
	eeGlobals.Gateway = cleanvalueip(value);
    break;
    default:
      shell_output("Unknown command: ", cmd);
    }

}
/*---------------------------------------------------------------------------*/
static uint8_t cleanvalue(char *value)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static uint32_t cleanvalueip(char *value)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static uint8_t cleanvaluemac(char *value)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static void
stats(char *str)
{
/*
  char outstring[4];
  inttostr(&outstring,0);

  shell_output("BEE Statistics", "\n");
  shell_output("Econet", "");
  shell_output("======", "\n");
  shell_output("Frames in        : ", outstring);
  shell_output("Short scouts     : ", outstring);
  shell_output("Rx Broadcasts    : ", outstring);
  shell_output("Tx Attempts      : ", outstring);
  shell_output("Tx Collisions    : ", outstring);
  shell_output("Tx Not Listening : " , outstring);
  shell_output("Tx Net Error     : " , outstring);
  shell_output("Tx Line Jammed   : " , outstring);
  shell_output("\n\nhelp, ? - show help", "");
  shell_output("exit    - exit shell", "");
*/
}/*---------------------------------------------------------------------------*/
static void
config(char *str)
{
  char outstring[5];
  char MAC_addr[13];

  uint8_t i;
  for (i = 0; i < 6; i++) {
    hextostr(MAC_addr + (i * 2), eeGlobals.MAC[i]);
  }
  MAC_addr[12] = 0;


  inttostr(outstring,eeGlobals.Econet_Network);
  shell_output("\nEconet\n======\n\nNetwork\t\t: ", outstring);

  inttostr(outstring,eeGlobals.Station);
  shell_output("Station\t\t: ", outstring);
/*
  inttostr(outstring,eeGlobals.ClockMultiplier);
  shell_output("Clock x\t\t: ", outstring);
*/
  shell_output("\nEthernet\n========\n\n", "");
  shell_output("MAC Address\t: ", MAC_addr);
  shell_output("IP Address\t: ", outstring);
  shell_output("Subnet\t\t: ", outstring);
  shell_output("Gateway\t\t: ", outstring);

  shell_output("\nEconet WAN\n========\n\n", "");
  shell_output("IP Address\t: ", outstring);
  shell_output("Subnet\t\t: ", outstring);
  shell_output("Gateway\t\t: ", outstring);

  inttostr(outstring,eeGlobals.Ethernet_Network);
  shell_output("AUN Network\t: ", outstring);

}/*---------------------------------------------------------------------------*/
static void
unknown(char *str)
{
  if(strlen(str) > 0) {
    shell_output("Unknown command: ", str);
  }
}
/*---------------------------------------------------------------------------*/
static struct ptentry parsetab[] =
  {{"stats", stats},
   {"config", config},
   {"set", help},
/*   {"set clck", setvalue}, */
   {"set sttn", setvalue},
   {"set enet", setvalue},
   {"set ethn", setvalue},
   {"set ipad", setvalue},
   {"set snet", setvalue},
   {"set gway", setvalue},
   {"set maca", setvalue},
   {"set ecip", setvalue},
   {"set ecsb", setvalue},
   {"set ewan", setvalue},
   {"exit", shell_quit},
   {"?", help},

   /* Default action */
   {NULL, unknown}};
/*---------------------------------------------------------------------------*/
void
shell_init(void)
{
}
/*---------------------------------------------------------------------------*/
void
shell_start(void)
{
  shell_output("BEE Gateway", "");
  shell_output("'?' for help", "");
  shell_prompt(SHELL_PROMPT);
}
/*---------------------------------------------------------------------------*/
void
shell_input(char *cmd)
{
  parse(cmd, parsetab);
  shell_prompt(SHELL_PROMPT);
}
/*---------------------------------------------------------------------------*/
