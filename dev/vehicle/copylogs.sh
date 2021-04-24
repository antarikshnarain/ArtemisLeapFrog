set -e
sudo mkdir -p /mnt/usb
sudo mount /dev/sda1 /mnt/usb
sudo mkdir -p /mnt/usb/logs/
sudo cp -r ~/.ros/log/* /mnt/usb/logs/
sudo umount /dev/sda1