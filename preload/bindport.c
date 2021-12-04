/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
*/

/*
   LD_PRELOAD library to restrict ephemeral port range for a process (when using bind with 0 port argument). 
   Compile on Linux with:
   gcc -nostartfiles -fpic -shared bindport.c -o bindport.so -ldl -D_GNU_SOURCE


   BIND_PORT_RANGE="10000-12000" LD_PRELOAD=./bindport.so test.o

   Based on: https://github.com/X4BNet/ld_preload_shim/blob/main/bind.c by Daniel Ryde

*/



#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <string.h>
int (*real_bind)(int, const struct sockaddr *, socklen_t);
int (*real_connect)(int, const struct sockaddr *, socklen_t);

char *port_range_env;
unsigned long int bind_addr_saddr;
unsigned long int inaddr_any_saddr;
struct sockaddr_in local_sockaddr_in[] = { 0 };
int startport = 0;
int endport = 0;

void _init (void)
{
        const char *err;

        real_bind = dlsym (RTLD_NEXT, "bind");
        if ((err = dlerror ()) != NULL) {
                fprintf (stderr, "dlsym (bind): %s\n", err);
        }

        inaddr_any_saddr = htonl (INADDR_ANY);
        if (port_range_env = getenv ("BIND_PORT_RANGE")) {
//
   const char s[2] = ":";
   char *saveptr1, *saveptr2, *start, *end, *endptr, *enptr1, *pr;
   pr = strdup(port_range_env);
   start = strtok_r(pr, "-", &end);
   // ports 1000 -> 60000
//printf("first: %s, end: %s s: %d  e: %d\n", start, end, strlen(start), strlen(end));
  if (strlen(start) > 5 || strlen(start) < 4 || strlen(end) < 4 || strlen(end) > 5 ) {
     printf("invalid BIND_PORT_RANGE (no -, too long, too short) \n");
   } else {
    int sp, ep;
    sp=strtol(start, &endptr, 10);
    ep=strtol(end, &endptr, 10);
    if ( sp > 0 && sp < 65536 && ep > 0 && ep < 65535 && ep > sp) {
      //printf("valid port range %u-%u original: %s \n", sp, ep, port_range_env);
      //printf("PRELOAD: restricting ephemeral bind port range \n");
      startport = sp;
      endport = ep;
    } else {
      //printf("invalid port range %u-%u \n", sp, ep);
    }

   }
   


          //
        }
}

int bind (int fd, const struct sockaddr *sk, socklen_t sl)
{
        static struct sockaddr_in *lsk_in;

        lsk_in = (struct sockaddr_in *)sk;
        int binderr = -1;
        //printf("PRELOAD bind: fd: %d port: %d \n", fd, ntohs (lsk_in->sin_port));

        if ( (lsk_in->sin_family == AF_INET || lsk_in->sin_family == AF_INET6 )
                && (lsk_in->sin_port == 0)
                && startport != 0) 
        {
                lsk_in->sin_port = htons( startport );
                //printf("port is 0");
        }

        binderr = real_bind (fd, sk, sl);
        while ( binderr < 0) {
          startport++;
          if (startport > endport) {
            //printf("bind failed and out of ports, returning EADDRINUSE error \n");
            errno = EADDRINUSE;
            return -1;
          }
          //printf("bind failed, trying again, port: %d errno %d \n",startport , errno);
          lsk_in->sin_port = htons( startport );
          binderr = real_bind (fd, sk, sl);
        }
        
        return binderr;
}


