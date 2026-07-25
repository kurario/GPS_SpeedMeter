#!/usr/bin/env python3
"""Generate the compact anti-aliased VLW font used by the speed display."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


GLYPHS = "-0123456789"


def be32(value: int) -> bytes:
    return struct.pack(">I", value & 0xFFFFFFFF)


def generate(font_path: Path, output_path: Path, size: int) -> None:
    font = ImageFont.truetype(str(font_path), size)
    glyphs: list[tuple[int, int, int, int, int, int, bytes]] = []

    for character in GLYPHS:
        left, top, right, bottom = font.getbbox(character, anchor="ls")
        width = right - left
        height = bottom - top
        advance = round(font.getlength(character))
        baseline_to_top = -top

        bitmap = Image.new("L", (width, height), 0)
        draw = ImageDraw.Draw(bitmap)
        draw.text((-left, -top), character, font=font, fill=255, anchor="ls")
        glyphs.append(
            (
                ord(character),
                height,
                width,
                advance,
                baseline_to_top,
                left,
                bitmap.tobytes(),
            )
        )

    ascent = max(glyph[4] for glyph in glyphs)
    descent = max(glyph[1] - glyph[4] for glyph in glyphs)
    header = b"".join(
        be32(value)
        for value in (len(glyphs), 11, ascent + descent, 0, ascent, descent)
    )
    metrics = b"".join(
        b"".join(be32(value) for value in (*glyph[:6], 0))
        for glyph in glyphs
    )
    bitmaps = b"".join(glyph[6] for glyph in glyphs)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(header + metrics + bitmaps)
    print(
        f"Wrote {output_path} ({output_path.stat().st_size} bytes, "
        f"{ascent}px ascent, {descent}px descent)"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--font",
        type=Path,
        default=Path("/Library/Fonts/Roboto-Bold.ttf"),
        help="Roboto Bold TrueType source",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("main/speed_font.vlw"),
        help="output VLW file",
    )
    parser.add_argument("--size", type=int, default=160)
    args = parser.parse_args()
    generate(args.font, args.output, args.size)


if __name__ == "__main__":
    main()
