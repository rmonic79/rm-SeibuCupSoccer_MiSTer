# Authors and Credits

## rmSeibuCupSoccer_MiSTer core

**Author**: Umberto Parisi ([rmonic79](https://github.com/rmonic79))

The original RTL for the Seibu Cup Soccer core — everything under
`rtl/SeibuCup/` and the project wrapper `SeibuCup.sv` — is copyright
Umberto Parisi and distributed under **GNU GPL v3 or later**.

That includes the **SEI300 / COP3 coprocessor** (`SeibuCup_cop3.sv`), which is
not a port of anything: MAME marks Seibu Cup Soccer
`MACHINE_UNEMULATED_PROTECTION | MACHINE_NOT_WORKING`, so the chip's behaviour
was derived from the game's own program ROM, from its microcode table, and —
for the trigonometric tables — from the lookup ROMs carried by the bootleg
board. Several of MAME's comments on the chip turned out to be wrong and were
corrected in the process; the findings are written up in `docs/COP_cupsoc.md`
and in the README.

Also original to this core:

| | |
|---|---|
| `crt_adjust_sys.sv`, `crt_vsize.sv` | CRT geometry on the analog output (H-Size, H-Position, V-Shift, V-Size with PVM and Cabinet modes) |
| `pause_overlay.sv`, `pause_text.sv` | VBlank-synchronised pause overlay with logo and supporters scroll |
| `SeibuCup_sprite_renderer.sv` | SEI252 sprite renderer |
| `SeibuCup_tile_layer.sv`, `SeibuCup_text_renderer.sv` | BG / MG / FG tilemaps and text layer |
| `SeibuCup_main_top.sv`, `SeibuCup_maincpu_map.sv` | bus, memory map, DMA arbitration |
| `SeibuCup_audio_z80.sv` | Z80 audio subsystem and mixer glue |
| `sdram_bridge.sv`, `tile_rom_arbiter.sv`, `rom_cache.sv` | memory paths |
| `tools/` | lint, boot simulation and the equivalence testbenches |

`ddram_sprite.sv` is adapted from `ddram_4port.sv` (Sorgelig / BoogieWings)
and keeps its original copyright.

## Third-party components

This core builds on top of excellent open-source projects. All third-party
sources retain their original copyright and license. The core as a whole
is distributed under **GNU GPL v3 or later** to stay compatible with the
most restrictive upstream (JTFRAME / JTCORES).

| Component | Author | Project | License |
|-----------|--------|---------|---------|
| **FX68K** — cycle-accurate M68000 core | Jorge Cwik ([ijor](https://github.com/ijor)) | [ijor/fx68k](https://github.com/ijor/fx68k) | LGPL-2.1 |
| **T80** — Z80 core | Daniel Wallner, MikeJ | [MiSTer-devel/T80](https://github.com/MiSTer-devel/T80) | BSD / GPL |
| **JTFRAME / JTCORES** — framework, filters, tilemap, etc. | Jose Tejada ([@jotego](https://github.com/jotego)) | [jotego/jtcores](https://github.com/jotego/jtcores) | GPL-3 |
| **JTOPL** — YM3812 OPL2 FM synthesizer | Jose Tejada | [jotego/jtopl](https://github.com/jotego/jtopl) | GPL-3 |
| **JT6295** — OKI M6295 ADPCM sample player | Jose Tejada | [jotego/jt6295](https://github.com/jotego/jt6295) | GPL-3 |
| **sdram.sv** — SDRAM controller | Sorgelig ([sorgelig](https://github.com/sorgelig)) | [MiSTer-devel](https://github.com/MiSTer-devel) | GPL-3 |
| **sys/ framework** — MiSTer HPS/IO, OSD, video scaler, audio | Sorgelig / MiSTer-devel | [MiSTer-devel/Main_MiSTer](https://github.com/MiSTer-devel/Main_MiSTer) | GPL-3 |

The JTOPL used here carries five upstream rhythm-channel fixes that were
missing from the copy this core started with; without them the percussion
does not play.

## Reference

- **Seibu Cup Soccer** — Seibu Kaihatsu, 1992, on the Seibu `legionna.cpp`
  hardware family: Seibu CRTC, SEI252 sprite chip, **SEI300 / COP3**
  coprocessor, COPX-D1 lookup ROM (which, as it turns out, nothing reads).
  This FPGA core is a reimplementation from the program ROM, from MAME source
  (`seibu/legionna.cpp`, `seibu/legionna_v.cpp`, `seibu/seibu_crtc.cpp`,
  `seibu/seibucop.cpp`) and from observation of real hardware behaviour.
  ROMs are **not** included and must be provided by the user.
- **MAME project** — reference for memory maps, timing and driver behaviour,
  including the parts this core had to disagree with.
  [mamedev/mame](https://github.com/mamedev/mame)
- **The bootleg board** (`seicupbl.cpp`) — replaces the COP with discrete
  logic and therefore carries the chip's lookup tables in ROM. Used as ground
  truth to verify the sine, cosine, division and hypotenuse/arctangent tables.
- The owner of the original PCB who tested the coprocessor work on hardware.

## Special thanks

- **Andrea Bogazzi** ([@asturur](https://github.com/asturur)) for the work on
  the CRT adjust module.
