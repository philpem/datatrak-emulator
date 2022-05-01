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

  - Open two terminals to display the output from the serial ports:
    - `stty -icanon && ncat -k -l 8888`
    - `stty -icanon && ncat -k -l 8889`
  - Start the emulator
    - `./emutrak`

## Contributing

Please fork the repository, make your changes on a branch, and open a pull request.

## Licence

GPLv3.

The 68000 emulator "Musashi" (by Karl Stenerud, in the "musashi" subrepo) is licenced separately. The licence terms for it are included in "musashi/readme.txt".

