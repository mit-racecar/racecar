#!/bin/sh

set -e

sudo cp pwm_sysfs.conf /etc/init/pwm_sysfs.conf
sudo chown root:root /etc/init/pwm_sysfs.conf
sudo chmod 644 /etc/init/pwm_sysfs.conf

sudo cp pwm_sysfs.sh /usr/local/bin/pwm_sysfs.sh
sudo chown root:root /usr/local/bin/pwm_sysfs.sh
sudo chmod 744 /usr/local/bin/pwm_sysfs.sh

sudo chmod a+rw /sys/class/pwm/pwmchip0/pwm0/polarity
