# Datatrak Mk.II Locator emulator

This is a very-unfinished emulator for the Datatrak Mk.II Locator navigation receiver.

It currently emulates the main 68000 processor, with 256K of RAM and ROM, and the dual UART.


## Instructions for use

Building:

```bash
git clone --recurse-submodules https://github.com/philpem/datatrak-emulator datatrak
cd datatrak
make
```

Running:

  - Start the emulator first — it will listen for connections on ports 10000 and 10001:
    - `./emutrak`
  - Connect to the serial ports in separate terminals:
    - `nc localhost 10000`  (UART A — emulator waits for this before booting the CPU)
    - `nc localhost 10001`  (UART B — optional)

Both `nc` (netcat) and `telnet` work.

## Contributing

Please fork the repository, make your changes on a branch, and open a pull request.

## Licence

GPLv3.

The 68000 emulator "Musashi" (by Karl Stenerud, in the "musashi" subrepo) is licenced separately. The licence terms for it are included in "musashi/readme.txt".

