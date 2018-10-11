#!/bin/bash

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev | grep ttyUSB) # 查找/sys/xxx/ttyUSB*/dev
do
    #echo "sysdevpath: ${sysdevpath}"
    syspath="${sysdevpath%/dev}" # 去掉结尾的dev
    echo "syspath   : ${syspath}"
    eval "$(udevadm info -q property --export -p $syspath)" # 查询设备属性
    [[ -z "$ID_SERIAL" ]] && continue
    echo -e "ID_SERIAL: ${ID_SERIAL}"
    echo -e "${ID_VENDOR_ID}:${ID_MODEL_ID} ${DEVNAME}"
    echo
done