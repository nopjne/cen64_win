Project Pentothal is a CEN64 modification to enable dumping and comparing of frame state while also enabling stable input recording and replay.
It is primarily aimed to speed up testing of Sodium64 (a Nintendo64 based SNES emulator) on a known good emulator CEN64.

The following command line parameters have been added to CEN64 to facilitate the desired testing functionality:
```
      "  -dumpref                   : Dump frames at 88 frame interval (recompile to change interval).\n"
      "  -compref                   : Compare frames with frames in .\\ref dir.\n"
      "  -buttontrace               : Record player input and save to a file driven by cart name.\n"
      "  -buttonreplay              : Replay player input from \\ref driven by cart name.\n"
      "  -windowoffset X Y          : Specify CEN64 window top left location coordinates.\n"
```

A script called Pentothal.py is present to enable starting of multiple CEN64 instances, based on smc/sfc roms present in the working directory.

See it in action:
[![See it in action](https://img.youtube.com/vi/b2kO7u1A6HI/0.jpg)](https://www.youtube.com/watch?v=b2kO7u1A6HI)

Sodium64: https://github.com/Hydr8gon/sodium64
CEN64: https://github.com/n64dev/cen64


