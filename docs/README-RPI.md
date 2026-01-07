# Raspberry Pi Setup

## Enable SSH on Raspberry Pi 5

The first edition of Ubuntu available for the RPi 5 is 23.10. On new
installation, password-based SSH is disabled by a cloud IoT policy.

To enable password-based SSH login, run the command:

```bash
sudo rm -f /etc/ssh/sshd_config.d/*
```

## Enable overscan compensation in the bootloader

Overscan is when the edges of the desktop appear outside the edges of your
display.

If your monitor has overscan issues, you can enable overscan compensation
in the bootloader by running the command:

```bash
sudo sed -i 's/^disable_overscan=1/#disable_overscan=1/' /boot/firmware/config.txt
```

## Prepare system for display capabilities

A script is provided to perform the following tasks:

- Create a 16GB swapfile and add it to `/etc/fstab`
- Disable apt from asking for user input
- Prevent the system from waiting on a network connection on startup

```bash
cd oasis_tooling/scripts
./setup_rpi.sh
```

Make sure to add the following line to .bashrc to disable apt from asking for
user input:

```bash
export NEEDRESTART_MODE="a"
```

## Install Gnome and Wayland

A script is provided to install Gnome and Wayland on the Raspberry Pi:

- Installs Gnome, Gnome Terminal and Gnome System Monitor
- Installs extensions from `oasis_visualization/extensions` folder
- Configures Gnome for an always-on display
- Configures Gnome to use the dark theme
- Disables systemd system suspend

```bash
cd oasis_tooling/scripts
./install_gnome.sh
```

This script takes approximately 30 minutes to execute on a microSDXC card with
a UHS Speed Class 3 (U3) and Video Speed Class 30 (V30) rating.
