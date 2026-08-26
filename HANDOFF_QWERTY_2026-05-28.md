# Handoff: zitaotech_sofle keymap migration

Date: 2026-05-28
Repo: `D:\zmk键盘固件\zmk_config_zitaotech_sofle`
Source Word file: `D:\zmk键盘固件\firmware\keymao.docx`

## Latest Status Addendum: 2026-05-29

This handoff file is historical. The repository has moved beyond the state described below. For current behavior, read `README.md` first.

Current branch state:

- Current remote head observed during continuation: `d0ad957 [Draw] func-G`.
- User firmware keymap entry point is `config/zitaotech_sofle.keymap`.
- Current drawer entry point is `keymap-drawer/zitaotech_sofle.yaml`.
- README keymap images are `docs/keymap-images/*.png`; they are regenerated in a compact dark keymap-editor style based on the old Word document screenshots, not directly from the looser drawer SVG layout.
- Local downloaded firmware artifact `firmware.zip` is ignored by `.gitignore`.
- The four active layers are `QWERTY`, `FUNC`, `NUM`, and `MOUSE`; the empty `RESERVE` layer has been removed.
- `MOUSE_layer` remains layer id `3`; `zip_temp_layer 3 600` must continue to point at it.

Important corrections to the older notes below:

- The older `&hd1 ENTER MB3` note is superseded. Current root keymap uses `&hd1 MB3 ENTER`.
- Current `hd1` is defined with `bindings = <&mkp>, <&kp>`, so the middle thumb key taps `ENTER` and holds mouse middle button `MB3`.
- The older `Sft+D` / `&kp LS(D)` note is superseded. Current `FUNC` layer uses `&kp LS(LC(D))`, shown in drawer as `Sft+Ctl+D`, at QWERTY `G` position.
- The older scroll-listener migration to positions `59/62` is superseded. Current scroll mode is position `61`, gated by highest active layer `MOUSE_layer` on central-side code. Position `60` no longer participates in scroll mode.
- Right-hand TrackPoint must not query central keymap state in the peripheral driver. It reports pointer motion; `config/boards/arm/zitaotech_sofle/custom_driver_left/trackpoint_scroll_processor.c` gates TrackPoint scroll on the central half.
- `config/boards/arm/zitaotech_sofle/zitaotech_sofle.keymap` is a board-level default file and is not currently synchronized with the root keymap. Do not treat exact equality between the two keymap files as a hard requirement unless the user explicitly asks to synchronize them.

## Current Goal

User wants this ZMK config to keep the current repository's special pointing-device behavior, but migrate key positions from the Word document layer by layer.

We have only modified and discussed the default `QWERTY` layer so far. Other layers still need review and confirmation one layer at a time.

## User Constraints

Do not break or remove these special mechanisms:

- TrackPoint / BB trackball movement temporarily enters mouse layer via `zip_temp_layer 3 600`; A320 trackpad stays on the current layer because it scrolls by default.
- TrackPoint / trackpad / trackball pointing behavior and arrow mode must keep working.
- Middle 8 keys in the physical center area must not be changed to match the Word document:
  - two `C_MUTE` keys
  - two mouse click keys around home row center
  - `PG_DN` / `PG_UP`
  - bracket hold-tap keys `&mt LBRC LBKT` / `&mt RBRC RBKT`
- Encoder / knob `sensor-bindings` must not be changed to match Word.
- Changes should proceed layer by layer, with user confirmation before each layer is edited.

## Word Document Extraction

The Word file contains only images, no extractable text.

Extracted image files are under:

`D:\zmk键盘固件\firmware\keymao_extracted`

Important images:

- `image1.png`: default `QWERTY` layer
- `image2.png`: `FUNC` layer
- `image3.png`: `MOUSE` layer
- `image4.png`: `RESERVE` / numeric layer

LibreOffice / `soffice` was not found locally, so visual DOCX render QA could not be run. The images were extracted directly from the `.docx`.

## Work Completed

### Default QWERTY layer migrated

Originally modified both keymap copies:

- `config/zitaotech_sofle.keymap`
- `config/boards/arm/zitaotech_sofle/zitaotech_sofle.keymap`

Current continuation note: after `func-G`, the root user keymap `config/zitaotech_sofle.keymap` is the authoritative firmware keymap. The board-level default keymap is not currently synchronized with it.

The default layer now follows the Word `QWERTY` image for ordinary keys while preserving the protected center keys and encoder bindings.

Key changes include:

- Right top row outside key changed from `ENTER` to `BSLH`.
- Right home-row outside key changed from `BSLH` to `SQT`.
- Bottom-right outside key changed from `EQUAL` to `DEL`.
- Thumb cluster changed to match Word:
  - left thumb layer/space key: `&lt 1 SPACE`
  - left center thumb: `&kp ENTER`
  - right center thumb: `&hd1 MB3 ENTER`
  - right thumb layer/space key: `&lt 1 SPACE`
  - right modifiers: `RALT`, `RGUI`, `RCTRL`

### Added behavior definitions

Added:

- `#define NUM 2`
- `TdCapToLay`: tap dance for Caps key
  - single tap: `&kp CAPS`
  - double tap: `&to NUM`
- `hd1`: hold-tap used by Word image's Enter / middle mouse button thumb key
  - defined as a hold-tap with `bindings = <&mkp>, <&kp>`
  - used as `&hd1 MB3 ENTER`
  - current behavior: tap `ENTER`, hold mouse middle button `MB3`

### FUNC-G continuation

After the original TrackPoint fix, the user made a `func-G` update:

- `FUNC` layer at QWERTY `G` position changed from `&kp LS(D)` to `&kp LS(LC(D))`.
- Drawer changed from `Sft+D` to `Sft+Ctl+D`.
- The same update corrected the `hd1` tap/hold order described above.

### Preserved pointing scroll behavior after thumb key migration

Current scroll-mode contract:

- Physical position `61` is the scroll-mode key.
- Physical position `60` no longer participates in scroll mode.
- `MOUSE_layer` remains layer id `3`.
- Left-side central drivers can query `zmk_keymap_highest_layer_active()`.
- Right-side TrackPoint is a split peripheral and must not query central keymap state in its own driver.
- TrackPoint scroll gating now lives in `config/boards/arm/zitaotech_sofle/custom_driver_left/trackpoint_scroll_processor.c`.

This preserves the intended behavior:

- Tap `LT 1 SPACE`: sends Space.
- Hold `LT 1 SPACE`: enters layer 1.
- In `MOUSE_layer`, holding physical position `61` enables scroll mode.
- Caps/CapsLock remains keyboard-layer behavior only and must not force scroll mode.

## Dunhao / Backslash Issue

User reported:

- Normal laptop keyboard and another keyboard produce Chinese dunhao `、` from the backslash key in Chinese input mode.
- This ZMK keyboard only produces backslash `\`.

Current change:

- The Word image places the dunhao/backslash key below Backspace.
- It is currently set to standard ZMK `&kp BSLH`.

Known possible next test:

- If flashing this version still produces `\` instead of `、`, try changing only that key from `&kp BSLH` to `&kp NUBS` / `NON_US_BSLH`.
- ZMK docs list both:
  - `BSLH`: standard US backslash / pipe key
  - `NUBS`: non-US backslash / pipe key

Do not change this immediately unless user confirms after testing or asks to try the alternative.

## Verification Done

Static checks completed:

- keymap-drawer shows 66 keys per active layer.
- `zip_temp_layer 3 600` remains for TrackPoint / BB trackball and is intentionally absent from the A320 trackpad listener.
- Right TrackPoint no longer calls `zmk_keymap_highest_layer_active()` in the peripheral driver.
- TrackPoint scroll listener position is `61`; A320 trackpad now scrolls by default in every layer.

Not done:

- No local firmware build was run because `west`, `cmake`, and `ninja` were not found in this environment.

## Current Modified Files

Expected modified files from the original migration and later continuations:

- `config/zitaotech_sofle.keymap`
- `config/boards/arm/zitaotech_sofle/zitaotech_sofle.keymap`
- `config/boards/shields/right_trackpoint/custom_driver_right/trackpoint_0x15.c`
- `config/boards/shields/left_bbtrackpad/custom_driver_left/a320.c`
- `config/boards/shields/left_bbtrackball/custom_driver_left/bbtrackball_input_handler.c`

Do not revert these unless the user explicitly requests it.

## Important Caveats For Next Agent

- There are two keymap files, but the root user keymap is currently authoritative: `config/zitaotech_sofle.keymap`.
- The board-level default keymap is not currently synchronized with the root keymap. Do not blindly overwrite one with the other.
- Treat physical positions used by drivers as behavior contracts:
  - position `32`: left trackpad press, right click on FUNC and left click on QWERTY/NUM/MOUSE
  - position `34`: A320 arrow mode
  - position `36`: A320 half-speed precision pointer mode
  - position `61`: TrackPoint scroll mode
- If future layer changes move the intended scroll keys again, update the three driver listeners accordingly.
- User wants per-layer confirmation before editing.

## Suggested Prompt For Next Window

Continue work in `D:\zmk键盘固件\zmk_config_zitaotech_sofle`.

Read `HANDOFF_QWERTY_2026-05-28.md` first. The default QWERTY layer has already been migrated from `D:\zmk键盘固件\firmware\keymao.docx` while preserving protected center keys, encoder bindings, and pointing-device scroll behavior. Do not revert those changes.

Next task: continue layer-by-layer keymap migration from the Word images. Start by reviewing the next layer image, compare it against the current repository layer, list proposed changes, and ask for confirmation before editing. Preserve all special pointing-device behavior, temporary mouse layer behavior, physical-position listeners, and the user-specified middle 8 protected keys / encoders.
