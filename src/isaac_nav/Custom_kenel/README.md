# Nvidia-Jetson-Kernel: Nvidia Jetson Orin board 64 GB jetpack  36.4.3

This repository gives advice on how to compile and flash the kernel/image for Nvidia Jetson boards. 

## General Process
In general, the process includes the following steps: 

1. Get the folder structure and Source code from Nvidia (If are you using the script option move to step 3)
2. Extract and place parts at the right location
3. Configure the kernel
4. Compile kernel, image, modules etc. 
5. Set the board in recovery mode and flash


## Installation process

This instruction assumes thaht the build folder is directly located in the user home directory and is named nvidia
```
~/nvidia/
```
The host system is assumed to be running on a *Ubuntu22.04 LTS*.


You need the following packages:
```
sudo apt update
sudo apt install build-essential bc git wget curl \
  python3 python3-pip libncurses5-dev libssl-dev flex bison \
  kmod cpio
```


1. Get source code and folder structure
a) Create a folder for the kernel build
```
mkdir nvidia
cd ~/nvidia 
```

b) Download from nvidia website: https://developer.nvidia.com/embedded/jetson-linux
 
```
wget -O bsp.tbz2 -L https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/Jetson_Linux_r36.4.3_aarch64.tbz2
wget -O rootfs.tbz2 -L https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/Tegra_Linux_Sample-Root-Filesystem_r36.4.3_aarch64.tbz2
```

c) Unpack at right locations
```
tar -xvpf bsp.tbz2
sudo tar -xvpf rootfs.tbz2 -C Linux_for_Tegra/rootfs/

```
d) Install the toolchain 

```
mkdir $HOME/l4t-gcc
cd $HOME/l4t-gcc
wget -O toolcahin.tbz2 -L https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v3.0/toolchain/aarch64--glibc--stable-2022.08-1.tar.bz2
tar xf toolcahin.tbz2
```

3. Configure the kernel
```
export ARCH=arm64
export CROSS_COMPILE=$HOME/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
sudo apt install gcc-aarch64-linux-gnu
export JETSON_VERSION=36.4.3 
```


a) Prepare the kernel for the configuration 
``` 
cd ~/nvidia/Linux_for_Tegra
sudo ./apply_binaries.sh  
sudo ./tools/l4t_flash_prerequisites.sh

# Let's modify the kernel before we flash the board.

cd ~/nvidia/Linux_for_Tegra/source
./source_sync.sh -k -t jetson_$JETSON_VERSION

cd ~/nvidia/Linux_for_Tegra/source/kernel
sudo vim ~/nvidia/Linux_for_Tegra/source/kernel/Makefile
  29gg
  ... defconfig
        $(MAKE) \
                ARCH=arm64 \
                -C $(kernel_source_dir) $(O_OPT) \
                LOCALVERSION=$(version) \
                nconfig
  ... Image
```

b) Build the kernel 

``` 
cd ~/nvidia/Linux_for_Tegra/source
./generic_rt_build.sh "enable"
make -C kernel
```
Menuconfig Terminal UI opens: y for yes, m for module, n for no
STRG + / --> Search (Real name in the conf file differ from the shown names and paths. Both are shown in the search results)
Modules selected:
- wireguard  
- Device Drivers > Network device support > USB Network Adapters: QMI WWAN , CDVC MBIM, USB CDC NCM, USB CDC ECM,
- Device Drivers > USB Support > USB Serial Converter support: USB driver for GSM and CDMA modems 

The modules should be named like this in the config file (differing from the description in menuconfig):
  # CONFIG_USB_NET_QMI_WWAN=y
  # CONFIG_USB_NET_CDC_MBIM=y
  # CONFIG_USB_SERIAL_OPTION=y
  # CONFIG_USB_USBNET=y
  # CONFIG_USB_NET_CDC_EEM=y
  # CONFIG_USB_NET_CDC_NCM=y
  # CONFIG_NET_CLS_BPF=m
  # CONFIG_BLK_DEV_THROTTLING=y
  # CONFIG_NET_CLS_CGROUP=m
  # CONFIG_CGROUP_NET_PRIO=y
  # CONFIG_IP_SET=m
  # CONFIG_IP_VS_NFCT=y
  # CONFIG_IP_VS_PROTO_TCP=y
  # CONFIG_IP_VS_PROTO_UDP=y
  # CONFIG_IP_VS_RR=m
  # CONFIG_CRYPTO_SEQIV=m
  # CONFIG_XFRM_USER=y
  # CONFIG_INET_ESP=m
  # CONFIG_NETFILTER_XT_TARGET_TPROXY=m
  # CONFIG_NETFILTER_XT_MATCH_MARK=m
  # CONFIG_NETFILTER_XT_MATCH_SOCKET=m
  # CONFIG_CGROUPS=y
  # CONFIG_CPUSETS=y
  # CONFIG_CGROUP_CPUACCT=y
  # CONFIG_CGROUP_SCHED=y
  # CONFIG_NAMESPACES=y
  # CONFIG_CGROUP_BPF=y

The configuration has been tested by setting these modules as included (y/*), modules did not work in a test, however should also work (m)

c) run the following commands to install the kernel and in-three-modules
```
export INSTALL_MOD_PATH=~/nvidia/Linux_for_Tegra/rootfs/
sudo -E make install -C kernel
cp kernel/kernel-jammy-src/arch/arm64/boot/Image \
  ~/nvidia/Linux_for_Tegra/kernel/Image
```

4. Build the NVIDIA Out-of-tree modules
build the modules external of the jetson 

```
cd ~/nvidia/Linux_for_Tegra/source
export IGNORE_PREEMPT_RT_PRESENCE=1
export CROSS_COMPILE=$HOME/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export KERNEL_HEADERS=$PWD/kernel/kernel-jammy-src
make modules
```
b) then Intall the modules and copy them to the right folder
```
export INSTALL_MOD_PATH=~/nvidia/Linux_for_Tegra/rootfs/
sudo -E make modules_install

# update the initramfs 

$ cd ~/nvidia/Linux_for_Tegra
$ sudo ./tools/l4t_update_initrd.sh

```


5. Build the DTBs

a) build the dtbs
```
cd ~/nvidia/Linux_for_Tegra/source
export CROSS_COMPILE=<toolchain-path>/bin/aarch64-buildroot-linux-gnu-
export KERNEL_HEADERS=$PWD/kernel/kernel-jammy-src
make dtbs

```

b) copy them to the right path 

```
cp kernel-devicetree/generic-dts/dtbs/* ~/nvidia/Linux_for_Tegra/kernel/dtb/

```


6. Set the board to recovery mode and flash

a) Setting Jetson Xavier to Recovery Mode
1. Turn off and remove power 
2. connect the power supply and press the middle button along the power button for 5 seconds 
3. Check on the host PC, if Jetson is in recovery mode: lsusb should list a NVIDIA Jetson board 

c) run the nvidia flash script. This should autodetect the jetson and flash it in approx. 15 minutes max. it will delete all the files from the board 
```
# to verify that all the modules are installed and ready to flash

cd ~/nvidia/Linux_for_Tegra
sudo ./tools/l4t_update_initrd.sh

# then run the following command to flash the board 

sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1   -c tools/kernel_flash/flash_l4t_external.xml -p "-c bootloader/generic/cfg/flash_t234_qspi.xml"    --showlogs --network usb0 --erase-all --keep jetson-agx-orin-devkit nvme0n1p1

```
nvme0n1p1 selects the m.2 ssd for the flash

## First steps on Jetson

Follow the steps to choose the user and device name and set up the internet connection 

```
sudo apt update
sudo apt upgrade -y
sudo apt install openssh-server screen tmux htop bwm-ng cuda nvidia-jetpack nano -y
```


Remove unnecessary packages
```
sudo apt remove --purge libreoffice*
sudo apt remove --purge thunderbird*
sudo apt clean
sudo apt autoremove
```

Disable GUI on boot
```
sudo systemctl set-default multi-user.target
```
instead of graphical.target

Start GUI: 
```
sudo systemctl start gdm3.service
```

Jetson stats:
```
sudo apt install python3-pip
sudo -H pip install -U jetson-stats
jtop
```


## Test on the device

Attach a RM500Q Modem with usb extension board. If after bootup, the following devices show up under /dev, the kernel modules have been successfully included:
```
cdc-wdm*
ttyUSB*
ttyUSB*
ttyUSB*
ttyUSB*
```
One ttyUSB is for Quectel flashing, one for GPS output and two for AT commands.
In modem manager, the modem should be listed. If shown details of it with, e.g., 'mmcli -m 0', the ports should include cdc-wdm0 (qmi) and tty ports
---
# Troubleshooting Guide

## 1. Kubernetes CPU / QoS Errors

If Kubernetes is failing with CPU or QoS-related errors, follow these steps.

### Step 1: Verify CPU is enabled in cgroups

Check whether `cpu` appears in the cgroup controllers:

```bash
cat /sys/fs/cgroup/cgroup.controllers
cat /sys/fs/cgroup/cgroup.subtree_control

# kubepods
cat /sys/fs/cgroup/kubepods.slice/cgroup.controllers
cat /sys/fs/cgroup/kubepods.slice/cgroup.subtree_control

# burstable
cat /sys/fs/cgroup/kubepods.slice/kubepods-burstable.slice/cgroup.controllers
cat /sys/fs/cgroup/kubepods.slice/kubepods-burstable.slice/cgroup.subtree_control
```

Expected result:  
All outputs should include `cpu`.

---

### Step 2: Enable CPU controller (if missing)

If `cpu` does not appear, enable it by running:

```bash
echo "+cpu" | sudo tee /sys/fs/cgroup/system.slice/cgroup.subtree_control
echo "+cpu" | sudo tee /sys/fs/cgroup/kubepods.slice/cgroup.subtree_control
```

After enabling, run the verification commands again to confirm `cpu` is listed.

---

## 2. 5G Modem Not Detected (or Only Shows 3G)

If the 5G modem is not detected or only connects as 3G, verify that the correct custom kernel is running.

### Step 1: Check the running kernel version

```bash
uname -r
```

Expected output:

```
5.15.148-rt-tegra
```

If a different version appears, configure the correct kernel.

---

### Step 2: Configure the RT kernel

Open:

```
/boot/extlinux/extlinux.conf
```

Add the following entry:

```
LABEL rt
      MENU LABEL RT kernel (5.15.148-rt-tegra)
      LINUX /boot/Image-5.15.148-rt-tegra
      INITRD /boot/initrd.img-5.15.148-rt-tegra
      FDT /boot/dtb/kernel_tegra234-p3737-0000+p3701-0000-nv.dtb
      APPEND ${cbootargs} root=/dev/nvme0n1p1 rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 console=ttyAMA0,115200 firmware_class.path=/etc/firmware fbcon=map:0 nospectre_bhb
      OVERLAYS /boot/jetson-io-hdr40-user-custom.dtbo
```

Then set:

```
DEFAULT rt
```

---

### Step 3: Reboot

```bash
sudo reboot
```

After rebooting, verify again:

```bash
uname -r
```

It should now show:

```
5.15.148-rt-tegra
```

Then test the 5G modem again.