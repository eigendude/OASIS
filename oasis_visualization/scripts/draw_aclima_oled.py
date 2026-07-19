#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from pathlib import Path

from PIL import Image


WIDTH = 128
HEIGHT = 32
BLOCK = 2
TEXT = "ACLIMA"

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT = SCRIPT_DIR.parent / "media" / "oled" / "aclima_shadow_128x32.png"

# Thick 8x11 source glyphs.
#
# Strokes are defined directly as two logical cells wide. Do not apply
# morphology or automatic thickening; it distorts the visible shadow.
GLYPHS = {
    "A": (
        "00111100",
        "01111110",
        "11000011",
        "11000011",
        "11000011",
        "11111111",
        "11111111",
        "11000011",
        "11000011",
        "11000011",
        "11000011",
    ),
    "C": (
        "00111111",
        "01111111",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "01111111",
        "00111111",
    ),
    "L": (
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11000000",
        "11111111",
        "11111111",
    ),
    "I": (
        "11111111",
        "11111111",
        "00011000",
        "00011000",
        "00011000",
        "00011000",
        "00011000",
        "00011000",
        "00011000",
        "11111111",
        "11111111",
    ),
    "M": (
        "11000011",
        "11100111",
        "11111111",
        "11011011",
        "11011011",
        "11000011",
        "11000011",
        "11000011",
        "11000011",
        "11000011",
        "11000011",
    ),
}

LETTER_SPACING = 3

# One logical cell equals BLOCK physical pixels.
SHADOW_DX = 1
SHADOW_DY = 1


def build_text_mask(text: str) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    cursor_x = 0

    for char in text:
        if char not in GLYPHS:
            raise ValueError(f"Unsupported character: {char}")

        glyph = GLYPHS[char]

        for y, row in enumerate(glyph):
            for x, value in enumerate(row):
                if value == "1":
                    cells.add((cursor_x + x, y))

        cursor_x += len(glyph[0]) + LETTER_SPACING

    return cells


def build_visible_shadow(
    letter_cells: set[tuple[int, int]],
) -> set[tuple[int, int]]:
    shadow_cells = {(x + SHADOW_DX, y + SHADOW_DY) for x, y in letter_cells}

    # The source letters are treated as opaque black shapes. Only the portion
    # of the shifted white shadow extending beyond those shapes is displayed.
    return shadow_cells - letter_cells


def render() -> Image.Image:
    letter_cells = build_text_mask(TEXT)
    visible_cells = build_visible_shadow(letter_cells)

    if not visible_cells:
        raise ValueError("The shadow offset produced no visible pixels")

    min_x = min(x for x, _ in visible_cells)
    max_x = max(x for x, _ in visible_cells)
    min_y = min(y for _, y in visible_cells)
    max_y = max(y for _, y in visible_cells)

    logical_width = max_x - min_x + 1
    logical_height = max_y - min_y + 1

    pixel_width = logical_width * BLOCK
    pixel_height = logical_height * BLOCK

    if pixel_width > WIDTH or pixel_height > HEIGHT:
        raise ValueError(
            f"Artwork is {pixel_width}x{pixel_height}, "
            f"which does not fit in {WIDTH}x{HEIGHT}"
        )

    offset_x = (WIDTH - pixel_width) // 2 - min_x * BLOCK
    offset_y = (HEIGHT - pixel_height) // 2 - min_y * BLOCK

    image = Image.new("1", (WIDTH, HEIGHT), 0)
    pixels = image.load()
    if pixels is None:
        raise RuntimeError("Pillow did not provide pixel access for the image")

    for logical_x, logical_y in visible_cells:
        start_x = offset_x + logical_x * BLOCK
        start_y = offset_y + logical_y * BLOCK

        for dy in range(BLOCK):
            for dx in range(BLOCK):
                pixels[start_x + dx, start_y + dy] = 1

    return image


def main() -> None:
    image = render()

    OUTPUT.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    image.save(
        OUTPUT,
        format="PNG",
        optimize=True,
        compress_level=9,
    )

    print(f"Wrote {OUTPUT} " f"({WIDTH}x{HEIGHT}, {OUTPUT.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
