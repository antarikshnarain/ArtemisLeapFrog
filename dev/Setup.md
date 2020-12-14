# Setup Instructions

## Setting up Raspberry Pi

### Installation media

1. Install Raspberry Imager.
2. Download Navio Image.
3. Flash Raspbian OS Lite.
4. Flash Navio Image.
5. Mount SD card on your desktop and create an empty file named ssh in boot partition.

```bash
touch ssh
```

### Configure Pi

1. Search Pi on network

    ```bash
    nmap -sn <base-ip-address>/24
    ## OR
    ssh pi@raspberrypi.local
    ## OR
    ssh pi@navio.local
    ```

2. Using *raspi-config* rename hostname and connect to wifi
    - Hostnames:
        1. leapfrog-root : master controller
        2. navio : pixhawk mimic controller
    - Wifi:
        - SSID: SERC Leapfrog
        - Pswd: *Refer the box* -> Hint: project hashname and timeline

3. Setup auto-login: On your local and Pi's

    ```bash
    ssh-keygen
    ssh-copy-id <username>@<hostip>
    ```

## Tips

1. Add SSH target to VSCode for better user experience.