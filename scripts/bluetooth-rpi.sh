#!/usr/bin/env bash


bluetoothctl -- agent on
bluetoothctl -- default agent
bluetoothctl -- scan on

#joystick
bluetoothctl -- pair B8:5A:F7:C3:48:64
bluetoothctl -- connect B8:5A:F7:C3:48:64

#gps
bluetoothctl -- pair 00:0D:B5:61:26:D3
bluetoothctl -- connect 00:0D:B5:61:26:D3


