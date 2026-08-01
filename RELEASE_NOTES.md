# superOS-303 v1.0.0-beta (Combined Edition)

## Read this first: be prepared to lose all your patterns

**Updating to v1.0.0-beta WILL erase your patterns. The pattern storage
layout moved, and on first boot the unit starts factory-fresh: all superOS
patterns, tracks, and settings are gone.** Back up your patterns with the web
editor (SysEx download) before updating and restore them afterwards, or
accept the loss before you send the update. D650C-mode patterns and any
installed mask ROM are stored separately and are not affected.

**Pattern length change:** since the 0.9.8/0.9.9 line, the maximum pattern
length is now **32 steps per section** (it was 64). Enter A/B mode
(ACCENT + SLIDE) to play a slot as one 64-step pattern. This trade is what guarantees
that every slot, fully loaded (all 3 variations, polyphony, step probability
on every step, all 8 tracks) always fits in storage alongside the new
features.

## Installing

This release is the **no-USB-C build** (DIN MIDI only) and installs through
the **original OS-303 bootloader**; no bootloader update and no ISP
programmer needed.

- **`.syx`**: the one-file MIDI update. Hold TAP (WRITE/NEXT) at power-on
  (four LEDs light solid), then press **Update OS...** in the web editor and
  pick the file; the editor sends it automatically. Any SysEx tool throttled
  to ~50 ms between messages (SysEx Librarian, MIDI-OX) works too. Units
  already on superOS 0.9.9+ can also enter the bootloader with SysEx
  `F0 7D 4A F7`. The flash service is bundled; there is no second file to
  install, and the file never writes the bootloader region.
- **`.hex`**: full image (firmware + flash service + bootloader) for ISP
  programming of a blank CPU.
- **D650C mode** ships without the mask ROM. Load your own dump from the web
  editor ("Load ROM dump": a raw 2048-byte .bin or a nibble-format .syx);
  superOS mode works fully without it.

## New since v0.9.9c

- **A/B mode**: every pattern slot is two 32-step sections. ACCENT + SLIDE
  enters a sticky A/B mode in which every pattern you select (and every chain
  member) plays A then B as one 64-step pattern; a single section button
  leaves it. Both LEDs lit with the playing half blinking.
- **A/B + chain memory**: hold a pattern key + FUNCTION in A/B mode to save
  the mode on that pattern; selecting it later (or powering on with it) turns
  A/B mode back on. Chains can be stored on their first pattern together with
  the A/B mode and re-arm automatically when that pattern is selected,
  surviving power-off. Hold a pattern key + FUNCTION with A/B mode off to
  clear the pattern's saved A/B memory and stored chain in one gesture.
- **A/B section editing**: the FN + PITCH step editor always opens on section
  A instead of following playback; PITCH MODE (or TIME MODE) + CLEAR flips
  the edited section, with the sub-mode LED solid for A and blinking for B.
  In PITCH/TIME write modes while running, PITCH MODE + CLEAR pins live edits
  to one section the same way.
- **Chain browsing**: with a chain playing and A/B mode off, a single section
  button browses the other bank without interrupting playback.
- **Metronome tap-write**: the original 303's TAP time-write, measured
  cycle-exact against the real mask ROM, extended into guided one-pass takes
  with overdub across linked pairs and whole chains.
- **New length editor**: FUNCTION opens on the page holding the last step;
  double/halve the pattern, spill into the B section, per-section triplet
  timing.
- **Factory reset**: in the FUNCTION + CLEAR menu, hold C# for 2 seconds
  (fast-blink warning) to erase all patterns and settings on both firmwares
  and reboot factory-fresh. An uploaded mask ROM survives.
- **In-editor OS updates**: the web editor validates and sends firmware
  updates itself; no external SysEx tool.
- **Single-file updates**: the SPM flash service is bundled into every update
  `.syx` and the release `.hex`.
- **Storage guarantee**: the 32-step layout is proven (host-tested) to fit
  the worst case of every feature armed on every slot, with the update
  reserve intact.
- Fixes: tap-write no longer wipes chained or linked-pair patterns with saved
  content; overdub ties never erase saved notes; matrix ghost guard for
  CLEAR + pattern-key combos; step-probability SysEx wire fix; pattern-key
  taps on a playing linked pair target section A.

## Coming from v0.9.8

Everything in the 0.9.9 line is included: step probability per characteristic
(accent, slide, down, up with per-step levels), the combined D650C emulator
build with panel firmware switching, USB-C MIDI (enumerates as
"SuperOS-303"), per-pattern scales with presets, three variations per slot
with variation-3 polyphony, keyboard play mode, the original-303 metronome
voicing, watchdog boot guard, and firmware version reporting to the web
editor.
