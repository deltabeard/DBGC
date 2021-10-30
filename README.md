# DBGC: Deltabeard's Game Boy Cartridge
A reprogrammable Game Boy cartridge that utilises the Raspberry Pi RP2040.

<img alt='Front preview of PCB' src='/hw/front.png' width='25%'> <img alt='Back preview of PCB' src='/hw/back.png' width='25%'>

## Supported Features
- Support for No MBC, MBC1 and MBC3 (no support for RTC yet) games.
- Support for battery-backed SRAM (but saving to FRAM currently doesn't work).
- Can only play the one game programmed in at compile time.

## Proposed Features

- Supports the Game Boy (DMG), Game Boy Pocket (MGB), Game Boy Color (CGB), and the Game Boy Advance (AGB) in Game Boy Color mode only.
- Real Time Clock (RTC) with battery backup.
- FRAM 32KiB for save support.
- NOR Flash 16MiB with support upto 8MiB ROM size.
- Support multiple games on NOR Flash.
- USB-C firmware upgrade support.
- USB-C flash storage explorer to store and retrieve save files and ROMs.
- USB-C networking support.

### Flash Storage
The flash storage will be partitioned to allow the user to either support large ROM sizes or increased number of ROMs stored. By setting the maximum ROM size to a smaller value, there will be more space to store other ROMs on the *Storage* partition. An example partition scheme is shown below.

| Partition | Size | Purpose |
|-----------|  --: |---------|
| Firmware | 256KiB | Stores firmware for the RP2040 |
| Storage | Remainder | Wear-levelling filesystem holding ROMs and save files |

### USB-C Networking

By utilising RP2040 and TinyUSB features, the USB-C port can act as a networking device, which when attached to a host, will allow the RP2040 to connect to the Internet. Future features will allow supported Game Boy games to make connections to the internet.

## Notes
BOM is optimised for purchase from JLCPCB with SMT assembly. The cost of producing a single cart of this revision is approximately £15. This version of the PCB has major errors that can be seen in the [errata.md](https://github.com/deltabeard/DBGC/blob/master/errata.md) document. Do not manufacture PCB version 0.3 or below; instead wait for the next version or the one on the master branch. 

# License

The majority of the software within this project are licensed under the 
terms of the ISC License. Please see the header of each source file for 
license terms. In brief, the copyright and permission notices must not be
modified.

The PCB schematic and layout are licensed under the 
Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) 
license. In brief, you may modify and share but only under the same license 
to allow others to continue modifying and sharing.

The distribution of this project in source and binary form, regardless for 
free or not, with proprietary software, including commercial games, is 
unauthorised.

This project includes
[Libbet and the Magic Floor](https://github.com/pinobatch/libbet) and
[2048-gb](https://github.com/Sanqui/2048-gb), which are both 
licensed under the terms of the zlib license, and are included within this
project in an unmodified binary form for testing purposes.

# Showcase

- [Pokemon Polished Crystal](https://github.com/Rangi42/polishedcrystal). An 
  major fan-made upgrade to Pokemon Crystal. Uses MBC3 with battery backed 
  SRAM and Real Time Clock. The version tested had CGB double-speed disabled,
  which caused minor graphical glitches within the game. DBGC does not 
  currently support double-speed, so using it to play Polished Crystal will 
  not result in the intended experience. Non-volatile saving does not 
  currently work (save data is erased on power off).

![Pokemon Polished Crystal](https://user-images.githubusercontent.com/3747104/139542854-f9939c75-38e1-47c1-935c-c93735fade4c.jpg)

- Mega Man 1. This commercial game uses MBC1 ROM banking.

![megaman1](https://user-images.githubusercontent.com/3747104/139542857-6119248a-b5ca-431a-a083-31c3c4363988.jpg)

- [2048-gb](https://github.com/Sanqui/2048-gb). Open source homebrew game. 
  Uses no ROM banking.

![2048](https://user-images.githubusercontent.com/3747104/139542860-f92e0137-292f-451e-84d8-506cd876ac5d.jpg)
