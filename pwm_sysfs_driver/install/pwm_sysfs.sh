#! /bin/sh
#

case "$1" in
  start)
	echo "PWM SYSFS Start"
	echo 163 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio163/direction
	echo 0 > /sys/class/pwm/pwmchip0/export
	sleep 1
	chmod 666 /sys/class/pwm/pwmchip0/pwm0/enable
	chmod 666 /sys/class/pwm/pwmchip0/pwm0/polarity
	chmod 666 /sys/class/pwm/pwmchip0/pwm0/period
	chmod 666 /sys/class/pwm/pwmchip0/pwm0/duty
	echo 163 > /sys/class/gpio/unexport
	;;

  stop)
	echo "PWM SYSFS Stop"
	echo out > /sys/class/gpio/gpio163/direction
	#echo 163 > /sys/class/gpio/unexport
	;;
esac


case "$1" in
  start)
	echo "PWM SYSFS Start"
	echo 165 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio165/direction
	echo 2 > /sys/class/pwm/pwmchip0/export
	sleep 1
	chmod 666 /sys/class/pwm/pwmchip0/pwm2/enable
	chmod 666 /sys/class/pwm/pwmchip0/pwm2/polarity
	chmod 666 /sys/class/pwm/pwmchip0/pwm2/period
	chmod 666 /sys/class/pwm/pwmchip0/pwm2/duty
	echo 165 > /sys/class/gpio/unexport
	;;

  stop)
	echo "PWM SYSFS Stop"
	echo out > /sys/class/gpio/gpio165direction
	#echo 165 > /sys/class/gpio/unexport
	;;
esac


exit 0
