# BB15 Reset And Power API

`BB15Pinout` describes two independent reset paths:

- `host.boardReset` controls the host GPIO connected to the BB15 board RESET
  input. It is active-low and resets the complete BB15 board.
- `akidaReset` controls the active-low Akida-only reset signal. The standard
  Nicla pinouts route it through expander P4.

The standard pinouts also use expander P0 for external-flash boot selection,
P3 for Akida sleep, and P2 for the completion interrupt route.

`powerDown()` and `powerUp()` control the board reset path. `powerDown()`
holds the complete BB15 board in reset; `powerUp()` releases it. A subsequent
`begin()` and `BB15Runner::begin()` are required before inference.

`holdAkidaInReset()` and `releaseAkidaReset()` control only the Akida reset
path. They do not reset the complete BB15 board.
