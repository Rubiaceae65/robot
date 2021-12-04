#!/usr/bin/env bash

set -eux
gcc -nostartfiles -fpic -shared bindport.c -o bindport.so -ldl -D_GNU_SOURCE

gcc  test.c -o test.o 
gcc echoserver.c -o echoserver.o 


#BIND_PORT_RANGE="21000-22000" LD_PRELOAD=./bindport.so ./test.o
BIND_PORT_RANGE="1000-23000" LD_PRELOAD=./bindport.so ./test.o

#BIND_ADDR="127.0.0.1" LD_PRELOAD=./bindport.so ./echoserver.o


