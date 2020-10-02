echo 'KERNEL=="i2c-[0-1]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules
udevadm control --reload-rules
usermod -a -G i2c $USER
