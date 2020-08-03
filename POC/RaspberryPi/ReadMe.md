# Raspberry Pi

## Additional packages
```bash
sudo apt-get install wiringpi
```

## Compiling
```bash
g++ -o output files.cpp -lwiringPi
```

## Connecting to Rasberry Pi

### Headless

1. PC to Pi connection
    1. Connect PI directly to the ethernet port.
    2. find ethernet ip, generally it is 10.0.42.1.
    3. Find PI on the local
    ```bash
    nmap -sn 10.0.42.1/24
    ```
    4. Connect to PI
    ```bash
    ssh pi@10.0.42.x
    ```

## Setup I2C communication
Refer [Electronics Wing|Raspi-I2C](https://www.electronicwings.com/raspberry-pi/raspberry-pi-i2c)

## SSH without password
1. On the host machine
```bash
# Select No if key already exist.
ssh-keygen -t rsa
ssh pi@x.x.x.x mkdir -p .ssh
cat .ssh/id_rsa.pub | ssh pi@x.x.x.x 'cat >> .ssh/authorized_keys'
```
2. Login to PI
```bash
ssh pi@x.x.x.x
```