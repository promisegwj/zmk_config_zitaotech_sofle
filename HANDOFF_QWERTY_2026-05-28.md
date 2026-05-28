# Handoff: zitaotech_sofle keymap migration

Date: 2026-05-28
Repo: `D:\zmk键盘固件\zmk_config_zitaotech_sofle`
Source Word file: `D:\zmk键盘固件\firmware\keymao.docx`

## Current Goal

User wants this ZMK config to keep the current repository's special pointing-device behavior, but migrate key positions from the Word document layer by layer.

We have only modified and discussed the default `QWERTY` layer so far. Other layers still need review and confirmation one layer at a time.

## User Constraints

Do not break or remove these special mechanisms:

- Pointing device movement temporarily enters mouse layer via `zip_temp_layer 3 600`.
- TrackPoint / trackpad / trackball scroll mode and arrow mode must keep working.
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

Modified both keymap copies:

- `config/zitaotech_sofle.keymap`
- `config/boards/arm/zitaotech_sofle/zitaotech_sofle.keymap`

The default layer now follows the Word `QWERTY` image for ordinary keys while preserving the protected center keys and encoder bindings.

Key changes include:

- Right top row outside key changed from `ENTER` to `BSLH`.
- Right home-row outside key changed from `BSLH` to `SQT`.
- Bottom-right outside key changed from `EQUAL` to `DEL`.
- Thumb cluster changed to match Word:
  - left thumb layer/space key: `&lt 1 SPACE`
  - left center thumb: `&kp ENTER`
  - right center thumb: `&hd1 ENTER MB3`
  - right thumb layer/space key: `&lt 1 SPACE`
  - right modifiers: `RALT`, `RGUI`, `RCTRL`

### Added behavior definitions

Added:

- `#define NUM 4`
- `TdCapToLay`: tap dance for Caps key
  - single tap: `&kp CAPS`
  - double tap: `&to NUM`
- `hd1`: hold-tap used by Word image's Enter / middle mouse button thumb key
  - defined as a hold-tap with `bindings = <&kp &mkp>`
  - used as `&hd1 ENTER MB3`

### Preserved pointing scroll behavior after thumb key migration

Important detail: the repository's scroll mode drivers used to listen to physical positions `60/61`, because those were the old `&mo 1` keys.

After migrating to Word layout, the layer/space thumb keys are on positions `59/62`. To avoid accidentally binding scroll mode to `ENTER` or `MB3`, these drivers were updated to listen to `59/62`:

- `config/boards/shields/right_trackpoint/custom_driver_right/trackpoint_0x15.c`
- `config/boards/shields/left_bbtrackpad/custom_driver_left/a320.c`
- `config/boards/shields/left_bbtrackball/custom_driver_left/bbtrackball_input_handler.c`

This preserves the intended behavior:

- Tap `LT 1 SPACE`: sends Space.
- Hold `LT 1 SPACE`: enters layer 1.
- While held, the pointing-device driver also sees that physical key as scroll mode.

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

- Default layer binding count is 66 in both keymap files.
- `zip_temp_layer 3 600` remained in place.
- No old driver listeners for `position 60` / `position 61` remain in the searched files.
- New scroll listener positions are `59` / `62`.

Not done:

- No local firmware build was run because `west`, `cmake`, and `ninja` were not found in this environment.

## Current Modified Files

Expected modified files:

- `config/zitaotech_sofle.keymap`
- `config/boards/arm/zitaotech_sofle/zitaotech_sofle.keymap`
- `config/boards/shields/right_trackpoint/custom_driver_right/trackpoint_0x15.c`
- `config/boards/shields/left_bbtrackpad/custom_driver_left/a320.c`
- `config/boards/shields/left_bbtrackball/custom_driver_left/bbtrackball_input_handler.c`

Do not revert these unless the user explicitly requests it.

## Important Caveats For Next Agent

- There are two keymap files. Keep default-layer changes synchronized in both.
- The two keymap files already differed in non-default layers before this handoff. Do not blindly overwrite one with the other.
- Treat physical positions used by drivers as behavior contracts:
  - position `34`: arrow mode
  - position `36`: slow / precision mode
  - positions `59/62`: scroll mode after this change
- If future layer changes move the intended scroll keys again, update the three driver listeners accordingly.
- User wants per-layer confirmation before editing.

## Suggested Prompt For Next Window

Continue work in `D:\zmk键盘固件\zmk_config_zitaotech_sofle`.

Read `HANDOFF_QWERTY_2026-05-28.md` first. The default QWERTY layer has already been migrated from `D:\zmk键盘固件\firmware\keymao.docx` while preserving protected center keys, encoder bindings, and pointing-device scroll behavior. Do not revert those changes.

Next task: continue layer-by-layer keymap migration from the Word images. Start by reviewing the next layer image, compare it against the current repository layer, list proposed changes, and ask for confirmation before editing. Preserve all special pointing-device behavior, temporary mouse layer behavior, physical-position listeners, and the user-specified middle 8 protected keys / encoders.
