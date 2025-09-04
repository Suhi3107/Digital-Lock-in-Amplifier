# Digital Lock-in Amplifier (STM32)

**STM32-based digital lock-in amplifier** for wavelength-modulation spectroscopy (WMS) used in TDLAS gas sensing.

## Highlights
- Real-time I/Q demodulation implemented on STM32H7 (DMA + NCO mixing).
- Supports 1f / 2f harmonic extraction, multi-harmonic processing and on-board filtering.
- Intended for embedded gas analyzers.

## Repository contents
- `main.c`, peripheral drivers and core demodulation loop
- `README.md` (this)
- `LICENSE` (GPL-3.0)

## Quick start (development)
1. Toolchain: STM32CubeIDE
2. Build: open project in STM32CubeIDE.
3. Flash: use ST-Link (or your programmer) to flash the binary to the STM32H7 board.
4. Serial output: board prints demodulated 2f/1f values over UART at 115200 baud.

## Example usage
- Connect photodetector analog output → ADC pin.
- Configure modulation frequency (see `main.c` constants).
- Read streaming 2f amplitude on UART; calibrate using known gas sample.

## Files to add (recommended)
- `docs/` : wiring diagram, block diagram (PNG), timing diagram.
- `examples/` : sample CSV traces & plotting script (for validation).
- `firmware/` : build instructions, pre-built .bin/.hex for evaluation.

## License
GPL-3.0. See `LICENSE`.

## Contact
GitHub: https://github.com/Suhi3107 —
