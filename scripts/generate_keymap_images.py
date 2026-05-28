from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
YAML_PATH = ROOT / "keymap-drawer" / "zitaotech_sofle.yaml"
OUT_DIR = ROOT / "docs" / "keymap-images"

IMAGE_W = 1800
IMAGE_H = 900
SS = 2

BG = "#111820"
KEY = "#20313f"
KEY_HELD = "#24394a"
KEY_TRANS = "#182531"
KEY_BORDER = "#223746"
TEXT = "#e7edf3"
MUTED = "#aab6c2"
ACCENT = "#42b983"
BLUE = "#4fb3ff"

FONT_REGULAR = Path("C:/Windows/Fonts/segoeui.ttf")
FONT_BOLD = Path("C:/Windows/Fonts/segoeuib.ttf")


# Base geometry from keymap-drawer's zitaotech_sofle physical layout. The
# outer two columns are lifted by 11 raw units below so they sit about half a
# key lower than the highest columns, instead of a full row lower.
RAW_POSITIONS = [
    (28, 67, 52, 52, 0),
    (84, 67, 52, 52, 0),
    (140, 35, 52, 52, 0),
    (196, 28, 52, 52, 0),
    (252, 35, 52, 52, 0),
    (308, 42, 52, 52, 0),
    (381, 35, 38, 38, 0),
    (487, 35, 38, 38, 0),
    (560, 42, 52, 52, 0),
    (616, 35, 52, 52, 0),
    (672, 28, 52, 52, 0),
    (728, 35, 52, 52, 0),
    (784, 67, 52, 52, 0),
    (840, 67, 52, 52, 0),
    (28, 123, 52, 52, 0),
    (84, 123, 52, 52, 0),
    (140, 91, 52, 52, 0),
    (196, 84, 52, 52, 0),
    (252, 91, 52, 52, 0),
    (308, 98, 52, 52, 0),
    (560, 98, 52, 52, 0),
    (616, 91, 52, 52, 0),
    (672, 84, 52, 52, 0),
    (728, 91, 52, 52, 0),
    (784, 123, 52, 52, 0),
    (840, 123, 52, 52, 0),
    (28, 179, 52, 52, 0),
    (84, 179, 52, 52, 0),
    (140, 147, 52, 52, 0),
    (196, 140, 52, 52, 0),
    (252, 147, 52, 52, 0),
    (308, 154, 52, 52, 0),
    (381, 91, 38, 38, 0),
    (487, 91, 38, 38, 0),
    (560, 154, 52, 52, 0),
    (616, 147, 52, 52, 0),
    (672, 140, 52, 52, 0),
    (728, 147, 52, 52, 0),
    (784, 179, 52, 52, 0),
    (840, 179, 52, 52, 0),
    (28, 235, 52, 52, 0),
    (84, 235, 52, 52, 0),
    (140, 203, 52, 52, 0),
    (196, 196, 52, 52, 0),
    (252, 203, 52, 52, 0),
    (308, 210, 52, 52, 0),
    (360, 160, 38, 24, 0),
    (402, 160, 38, 24, 0),
    (466, 160, 38, 24, 0),
    (508, 160, 38, 24, 0),
    (560, 210, 52, 52, 0),
    (616, 203, 52, 52, 0),
    (672, 196, 52, 52, 0),
    (728, 203, 52, 52, 0),
    (784, 235, 52, 52, 0),
    (840, 235, 52, 52, 0),
    (140, 259, 52, 52, 0),
    (196, 252, 52, 52, 0),
    (252, 259, 52, 52, 0),
    (309, 283, 52, 52, 11.93),
    (374, 308, 52, 52, 23.869),
    (483, 313, 52, 52, -23.869),
    (550, 293, 52, 52, -11.93),
    (616, 259, 52, 52, 0),
    (672, 252, 52, 52, 0),
    (728, 259, 52, 52, 0),
]

OUTER_KEYPOS = {
    0,
    1,
    12,
    13,
    14,
    15,
    24,
    25,
    26,
    27,
    38,
    39,
    40,
    41,
    54,
    55,
}


def load_font(path: Path, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(path), size=size)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return load_font(FONT_BOLD if bold else FONT_REGULAR, size * SS)


def normalize_label(value: Any) -> tuple[str, str | None, str | None]:
    if isinstance(value, dict):
        tap = str(value.get("t", "") or "")
        hold = value.get("h")
        key_type = value.get("type")
        return clean_label(tap), clean_label(hold) if hold is not None else None, key_type

    return clean_label(value), None, None


def clean_label(value: Any) -> str:
    text = str(value or "").strip()
    if not text:
        return ""

    replacements = {
        "&TdCapToLay": "CAPS",
        "&bootloader": "BOOT",
        "&mkp LCLK": "LCLK",
        "&mkp RCLK": "RCLK",
        "&mkp MB1": "MB1",
        "&mkp MB2": "MB2",
        "&mkp MB3": "MB3",
        "&msc SCRL_UP": "SCRL\nUP",
        "&msc SCRL_DOWN": "SCRL\nDN",
        "&bl BL_DEC": "BL\nDEC",
        "&bl BL_INC": "BL\nINC",
        "BT CLR": "BT\nCLR",
        "OUT TOG": "OUT\nTOG",
        "KP NUM": "KP\nNUM",
        "KP SLASH": "KP\n/",
        "KP ASTERISK": "KP\n*",
        "KP MINUS": "KP\n-",
        "KP PLUS": "KP\n+",
        "KP EQUAL": "KP\n=",
        "KP COMMA": "KP\n,",
        "KP DOT": "KP\n.",
        "KP ENTER": "KP\nENTER",
        "PG DN": "PG\nDN",
        "PG UP": "PG\nUP",
    }
    if text in replacements:
        return replacements[text]

    if text.startswith("&kp "):
        text = text[4:]
    if text.startswith("&"):
        text = text[1:]
    text = text.replace("_", " ")

    if text.startswith("KP N"):
        return "KP\n" + text[4:]
    if text in {"", "none", "trans", "null"} or text.startswith("鈻"):
        return ""

    return text


def text_size(draw: ImageDraw.ImageDraw, text: str, fnt: ImageFont.FreeTypeFont) -> tuple[int, int]:
    if not text:
        return 0, 0
    box = draw.multiline_textbbox((0, 0), text, font=fnt, spacing=2 * SS, align="center")
    return box[2] - box[0], box[3] - box[1]


def fitted_font(draw: ImageDraw.ImageDraw, text: str, max_w: int, max_h: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    for size in range(24, 8, -1):
        fnt = font(size, bold=bold)
        w, h = text_size(draw, text, fnt)
        if w <= max_w and h <= max_h:
            return fnt
    return font(9, bold=bold)


def draw_centered_text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    max_w: int,
    max_h: int,
    fill: str,
    bold: bool = False,
) -> None:
    if not text:
        return
    fnt = fitted_font(draw, text, max_w, max_h, bold=bold)
    w, h = text_size(draw, text, fnt)
    draw.multiline_text(
        (xy[0] - w / 2, xy[1] - h / 2),
        text,
        font=fnt,
        fill=fill,
        spacing=2 * SS,
        align="center",
    )


def draw_key_on(draw: ImageDraw.ImageDraw, cx: int, cy: int, w: int, h: int, tap: str, hold: str | None, key_type: str | None) -> None:
    fill = KEY_TRANS if key_type == "trans" else KEY_HELD if key_type == "held" else KEY
    radius = max(7 * SS, min(w, h) // 9)
    x0 = cx - w // 2
    y0 = cy - h // 2
    x1 = cx + w // 2
    y1 = cy + h // 2
    draw.rounded_rectangle((x0, y0, x1, y1), radius=radius, fill=fill, outline=KEY_BORDER, width=1 * SS)

    if hold:
        hold_font = fitted_font(draw, hold, int(w * 0.78), int(h * 0.28), bold=False)
        draw.multiline_text(
            (x0 + 6 * SS, y0 + 5 * SS),
            hold,
            font=hold_font,
            fill=BLUE,
            spacing=1 * SS,
            align="left",
        )
        tap_y = cy + int(h * 0.12)
        tap_h = int(h * 0.55)
    else:
        tap_y = cy
        tap_h = int(h * 0.72)

    draw_centered_text(draw, (cx, tap_y), tap, int(w * 0.86), tap_h, TEXT, bold=False)


def draw_rotated_key(
    base: Image.Image,
    cx: int,
    cy: int,
    w: int,
    h: int,
    angle: float,
    tap: str,
    hold: str | None,
    key_type: str | None,
) -> None:
    pad = 28 * SS
    patch = Image.new("RGBA", (w + pad * 2, h + pad * 2), (0, 0, 0, 0))
    pdraw = ImageDraw.Draw(patch)
    draw_key_on(pdraw, patch.width // 2, patch.height // 2, w, h, tap, hold, key_type)
    rotated = patch.rotate(angle, expand=True, resample=Image.Resampling.BICUBIC)
    base.alpha_composite(rotated, (cx - rotated.width // 2, cy - rotated.height // 2))


def px(raw_x: float, raw_y: float, keypos: int) -> tuple[int, int]:
    scale = 1.55 * SS
    x_origin = 320 * SS
    y_origin = 170 * SS
    y_adjust = -11 if keypos in OUTER_KEYPOS else 0
    return int(x_origin + (raw_x - 28) * scale), int(y_origin + (raw_y + y_adjust - 28) * scale)


def draw_layer(name: str, layer_index: int, keys: list[Any]) -> Image.Image:
    img = Image.new("RGBA", (IMAGE_W * SS, IMAGE_H * SS), BG)
    draw = ImageDraw.Draw(img)

    draw.text((26 * SS, 34 * SS), "zmk_config_zitaotech_sofle / zitaotech_sofle", font=font(16), fill=TEXT)

    draw.rounded_rectangle((26 * SS, 68 * SS, 89 * SS, 109 * SS), radius=6 * SS, fill=ACCENT)
    draw.text((34 * SS, 82 * SS), "LAYERS", font=font(12, bold=True), fill="white")
    for i in range(4):
        y = 120 + i * 48
        fill = ACCENT if i == layer_index else "#24313f"
        draw.rounded_rectangle((26 * SS, y * SS, 70 * SS, (y + 44) * SS), radius=5 * SS, fill=fill)
        draw.text((42 * SS, (y + 12) * SS), str(i), font=font(20, bold=True), fill="white")

    draw.text((320 * SS, 90 * SS), "default_layer" if name == "QWERTY" else name.lower() + "_layer", font=font(28, bold=True), fill=TEXT)
    draw.text((515 * SS, 96 * SS), name, font=font(16), fill=TEXT)

    for keypos, item in enumerate(keys):
        raw_x, raw_y, raw_w, raw_h, angle = RAW_POSITIONS[keypos]
        cx, cy = px(raw_x, raw_y, keypos)
        w = int(raw_w * 1.55 * SS)
        h = int(raw_h * 1.55 * SS)
        tap, hold, key_type = normalize_label(item)
        if angle:
            draw_rotated_key(img, cx, cy, w, h, angle, tap, hold, key_type)
        else:
            draw_key_on(draw, cx, cy, w, h, tap, hold, key_type)

    draw.text((320 * SS, 740 * SS), "Sensor bindings", font=font(18), fill=TEXT)
    draw.rounded_rectangle((320 * SS, 771 * SS, 450 * SS, 796 * SS), radius=3 * SS, fill="#273343")
    draw.rounded_rectangle((510 * SS, 771 * SS, 640 * SS, 796 * SS), radius=3 * SS, fill="#273343")
    draw.text((348 * SS, 778 * SS), "encoder_left", font=font(12), fill=TEXT)
    draw.text((536 * SS, 778 * SS), "encoder_right", font=font(12), fill=TEXT)
    draw.ellipse((342 * SS, 808 * SS, 424 * SS, 890 * SS), fill="#203849")
    draw.ellipse((532 * SS, 808 * SS, 614 * SS, 890 * SS), fill="#203849")
    draw_centered_text(draw, (383 * SS, 849 * SS), "VOL_UP\nVOL_DN", 76 * SS, 52 * SS, TEXT)
    draw_centered_text(draw, (573 * SS, 849 * SS), "LEFT\nRIGHT", 76 * SS, 52 * SS, TEXT)

    draw.text((1520 * SS, 838 * SS), f"{name} layer - {len(keys)} keys", font=font(12), fill=MUTED)

    return img.resize((IMAGE_W, IMAGE_H), Image.Resampling.LANCZOS).convert("RGB")


def main() -> None:
    data = yaml.safe_load(YAML_PATH.read_text(encoding="utf-8"))
    layers: dict[str, list[Any]] = data["layers"]
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    names = ["QWERTY", "FUNC", "NUM", "MOUSE"]
    filenames = ["qwerty.png", "func.png", "num.png", "mouse.png"]
    for index, (name, filename) in enumerate(zip(names, filenames)):
        keys = layers[name]
        if len(keys) != 66:
            raise ValueError(f"{name} has {len(keys)} keys, expected 66")
        draw_layer(name, index, keys).save(OUT_DIR / filename, optimize=True)
        print(f"wrote {OUT_DIR / filename}")


if __name__ == "__main__":
    main()
