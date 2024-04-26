*from 2023*
# Shutdown Procedure

This is the procedure for packing up from a product demo

## Shutdown the ROV

1. In the Second Terminal Window use the `ctrl`+`c` keyboard shortcut to Keyboard Interrupt ros.

1. type in

```console
sudo shutdown now
```

1. you will be disconnected as the rov shuts down

1. In the second terminal window use the keyboard shortcut `ctrl`+`d` to close the window

1. watch for the activity (green) light on the rov pi to be off for 10 seconds

1. Flip the `OUTPUT SW` to `OFF` on the 48V PSU

1. Flip the `INPUT SW` to `O` on the 48V PSU

1. unplug the SBS50 from the 48v PSU

1. unplug the 48V PSU Power Cable (IEC-C19) from the Power Input Socket (IEC-C20) on the 48V PSU

1. unplug the 48V PSU Power Cable (IEC-C19 / NEMA 5-15P) from the Extension Cord Outlet (NEMA 5-15R)

1. Coil the cable inside the PSU box

1. unplug the ROV LAN from the deck box

1. Disconnect the Pneumatic Line from the Compressor Hookup

1. disconnect the tether from the ROV

1. pack the tether and rov into their transport containers

## Shutdown the DECK BOX

1. Close the RQT window with the X on the top right

1. in the first terminal window use the keyboard shortcut `ctrl`+`c` to Keyboard Interrupt ros

1. in the first terminal window use the keyboard shortcut `ctrl`+`d` to close the window

1. in the third terminal window type the following to shutdown the DECK

```console
sudo shutdown now
```

1. wait for the fans to stop spinning on the deck box pc

1. switch the `POWER IN` switch to `OFF`

1. unplug the Deck power Cable (IEC-C13) from the power entry socket (IEC-C14).

1. unplug the xbox controller and coil its cable and secure it with the velcro tie

1. unplug the WAN ethernet cable if necessary

1. unplug the monitor USB-C from the `MONITOR PSU`

1. unscrew the RP-SMA Wi-FI antennas from their positions and place them flat on top of the lexan

1. turn off the keyboard and place it on top of the lexan

1. verify nothing will be damaged by closing the deck box

1. close and latch the deck box

1. close and latch the PSU
