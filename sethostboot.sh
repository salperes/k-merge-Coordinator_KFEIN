#!/bin/bash

MAC=`hostname`
if [ $MAC == "raspberrypi" ]
 then
        MAC='coord-'
        MAC=$MAC`cat /sys/class/net/eth0/address | tr -d : | tr '[:lower:]' '[:upper:]'`
        echo $MAC

        hostname $MAC
        echo $MAC > /etc/hostname

        sed 's/raspberrypi/'$MAC'/' /etc/hosts > /etc/hosts_
        mv /etc/hosts_ /etc/hosts

	#echo 'dtoverlay=gpio-no-irq' >> /boot/config.txt
	sed 's/dtparam=audio=on/dtparam=audio=off/' /boot/config.txt > /boot/config.txt_
	mv /boot/config.txt_ /boot/config.txt

        reboot
        exit

else
        exit
fi

