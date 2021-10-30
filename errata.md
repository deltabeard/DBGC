# Errata

Problems that cannot be solved via software updates.

## PCB dated "09/21"

### Major
- USB differential signals are connected the wrong way round.
  The fix is to cross the signals at the resistors using magnet wire (or alternative).
- The IO2 and IO3 connections to the NOR Flash are connected incorrectly. Possible fixes:
  - Use magnet wire to cross the IO2 and IO3 signals. v0.2 and above requires this fix.
  - Use single-IO or dual-IO flash access, which use IO0 or IO0 and IO1, respectively.
  
### Minor
- The CR2032 cell holder is too far in the PCB, making it difficult to remove the cell from the holder.
- The RTC IC is too close to the top edge of the PCB, making it difficult to place within a GB cartridge enclosure.
- The LED light does not work.
