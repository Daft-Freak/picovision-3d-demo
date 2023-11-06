# PicoVision 3D Demo

3D demo of a PicoVision... on a PicoVision, or since this is using the 32blit SDK, another device (may require adjusting max triangles and/or tile size).


On an RP2040-based device uses both cores and both interpolators (on each core) for more speed.


## Building

```
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=/path/to/32blit-sdk/pico.toolchain -DPICO_BOARD=pico_w -DPICO_ADDON=pimoroni_picovision
ninja -C build
```
... or however you usually build 32blit SDK projects for your favourite device.