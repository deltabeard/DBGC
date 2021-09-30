# DBGC: Deltabeard's Game Boy Cartridge
A reprogrammable Game Boy cartridge that utilises the Raspberry Pi RP2040.

<img alt='Front preview of PCB' src='/hw/front.png' width='25%'> <img alt='Back preview of PCB' src='/hw/back.png' width='25%'>

## Proposed Features
- Supports the Game Boy (DMG), Game Boy Pocket (MGB), Game Boy Color (CGB), and the Game Boy Advance (AGB) in Game Boy Color mode only.
- Real Time Clock (RTC) with battery backup.
- FRAM 32KiB for immediate saving.
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
BOM is optimised for purchase from JLCPCB with SMT assembly.

# License

This pre-release project is proprietary, and not for resale.
