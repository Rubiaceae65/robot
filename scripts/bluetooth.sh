#!/usr/bin/env bash

if ! pgrep -x "dbus-daemon" > /dev/null
then
    # export DBUS_SESSION_BUS_ADDRESS=$(dbus-daemon --config-file=/usr/share/dbus-1/system.conf --print-address | cut -d, -f1)

    # or:
    dbus-daemon --config-file=/usr/share/dbus-1/system.conf
    # and put in Dockerfile:
    # ENV DBUS_SESSION_BUS_ADDRESS="unix:path=/var/run/dbus/system_bus_socket"
else
    echo "dbus-daemon already running"
fi

#/usr/libexec/bluetooth/bluetoothd --debug &
/usr/sbin/bluetoothd -d

#reset BCM chip (making sure get access even after container restart)
/opt/vc/bin/vcmailbox 0x38041 8 8 128 0  > /dev/null
sleep 1
/opt/vc/bin/vcmailbox 0x38041 8 8 128 1  > /dev/null 
sleep 1

#load firmware to BCM chip and attach to hci0
#hciattach /dev/ttyAMA0 bcm43xx 115200 noflow
hciattach /dev/ttyAMA0 bcm43xx 921600 noflow


#create hci0 device
hciconfig hci0 up


