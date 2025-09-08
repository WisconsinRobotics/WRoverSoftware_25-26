# WRoverSoftware 2025-2026

This repository contains the software for Wisconsin Robotics' rover, developed for the 2026 University Rover Challenge (URC). Below are the steps to set up the development environment.

---

## Table of Contents
1. [System Requirements](#system-requirements)
2. [Installation Guide](#installation-guide)
    - [Using VirtualBox](#using-virtualbox)
    - [Using UTM (MacOS Alternative)](#using-utm-macos-alternative)
3. [Installing ROS 2 Jazzy](#installing-ros-2-jazzy)
4. [Troubleshooting](#troubleshooting)

---

## System Requirements

- **Processor Architecture**: x86 or ARM64
- **Operating System**: Ubuntu 24.04.3 (Daily Build)
- **Virtualization Software**: Oracle VirtualBox (preferred) or UTM (MacOS alternative)

---

## Installation Guide

### Using VirtualBox

1. **Download Ubuntu**:  
   Download the Ubuntu 24.04.3 daily build ISO from [here](https://cdimage.ubuntu.com/daily-live/20240421/). Choose the appropriate image for your processor architecture (x86 or ARM64).

2. **Install VirtualBox**:  
   Download and install Oracle VirtualBox from the official [website](https://www.virtualbox.org/).

3. **Create a New Virtual Machine**:
   - Open VirtualBox and click the **New** button.
   - In the "Name and Operating System" section:
     - Enter a name for your virtual machine.
     - Select the location of the Ubuntu ISO image.
   - Uncheck the "Skip Unattended Installation" option.

4. **Set Up the Virtual Machine**:
   - **Username and Password**: Configure these in the "Unattended Guest OS Installation" section.
   - **Virtual Hardware**:
     - Allocate half of your available RAM to the virtual machine.
     - Allocate half of your CPU cores to the virtual machine.
   - **Virtual Hard Disk**: Set the disk size to 64 GB or more.

5. **Start the Virtual Machine**:  
   Follow the on-screen instructions to complete the Ubuntu installation.

---

### Using UTM (MacOS Alternative)

If VirtualBox does not work on MacOS, follow these steps:

1. **Install UTM**:  
   Download and install UTM from the official [website](https://mac.getutm.app/).

2. **Create a New Virtual Machine**:
   - Open UTM and click **Create New Virtual Machine**.
   - Select **Virtualize** as the virtualization method.
   - Choose **Linux** as the target operating system.
   - Specify the path to the Ubuntu ISO image.

3. **Configure the Virtual Machine**:
   - **CPU, RAM, and Disk Size**: Use the same values as specified for VirtualBox (half of your system's resources and 64 GB disk space).

4. **Start the Virtual Machine**:  
   Follow the on-screen instructions to complete the Ubuntu installation.

---

## Installing ROS 2 Jazzy

To install ROS 2 Jazzy, follow instructions [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)