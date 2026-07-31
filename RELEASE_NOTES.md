# superOS-303 v1.0.0-beta (Combined Edition)

## Read this first: this update factory-resets your patterns

**Installing v1.0.0-beta moves the pattern storage layout. On first boot the
unit starts factory-fresh: all superOS patterns, tracks, and settings are
erased.** Back up your patterns with the web editor (SysEx download) before
updating and restore them afterwards. D650C-mode patterns and any installed
mask ROM are stored separately and are not affected.

**Pattern length change:** since the 0.9.8/0.9.9 line, the maximum pattern
length is now **32 steps per section** (it was 64). Link sections A + B
(ACCENT + SLIDE) to get one 64-step pattern. This trade is what guarantees
that every slot, fully loaded (all 3 variations, polyphony, step probability
on every step, all 8 tracks) always fits in storage alongside the new
features.

## Installing

- **`.syx`**: the one-file MIDI update. Easiest path: the web editor's
  **Update OS...** button picks the file, reboots the unit into the
  bootloader, and sends it automatically. Manual path: hold TAP (WRITE/NEXT)
  at power-on (or send `F0 7D 4A F7`), then send the file with SysEx
  Librarian / MIDI-OX throttled to ~50 ms between messages. The flash service
  is bundled; there is no second file to install.
- **`.hex`**: full image (firmware + flash service + bootloader) for ISP
  programming of a blank CPU.

## New since v0.9.9c

- **A/B sections**: every pattern slot is two 32-step sections; link them for
  one 64-step pattern. Both LEDs lit with the playing half blinking.
- **Chain memory**: chains can be stored on their first pattern and re-arm
  automatically when that pattern is selected, surviving power-off.
- **Chain-wide A/B mode**: with a chain playing, ACCENT + SLIDE makes every
  member play A then B; a single section button browses the other bank
  without interrupting playback.
- **Metronome tap-write**: the original 303's TAP time-write, measured
  cycle-exact against the real mask ROM, extended into guided one-pass takes
  with overdub across linked pairs and whole chains.
- **New length editor**: FUNCTION opens on the page holding the last step;
  double/halve the pattern, spill into the B section, per-section triplet
  timing.
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
