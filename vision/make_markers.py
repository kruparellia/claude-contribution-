#!/usr/bin/env python3
"""
make_markers.py
---------------
Generate print-ready ArUco markers for the arm + workspace.

Outputs a PDF (preferred — page dimensions are in mm, so printer scaling
can't change the marker size) plus a PNG copy with DPI metadata as backup.

Usage:
    python3 vision/make_markers.py                 # default: IDs 0..3, 25 mm
    python3 vision/make_markers.py --size 30       # 30 mm physical size
    python3 vision/make_markers.py --ids 0 1 2 7   # specific marker IDs
    python3 vision/make_markers.py --ids 10 --size 50 --label "BASE HEADING"

Each PDF page contains one marker at the requested physical size, a 25 mm
verification ruler beside it, and an ID label.  Print at 100 % scale and
check the ruler against a real ruler before cutting.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas as rl_canvas
from reportlab.lib.utils import ImageReader

DICT = cv2.aruco.DICT_4X4_50          # 4x4 bits, 50 IDs — robust at low res
DPI  = 600                            # high enough that the marker stays crisp

def make_marker_image(marker_id: int, size_mm: float) -> Image.Image:
    """Render the marker bitmap with the required 1-cell white quiet zone."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
    side_px    = int(round(size_mm / 25.4 * DPI))
    marker     = cv2.aruco.generateImageMarker(aruco_dict, marker_id, side_px)
    border_px  = side_px // 4              # ≥1 marker-cell wide, as ArUco requires
    bordered   = cv2.copyMakeBorder(
        marker, border_px, border_px, border_px, border_px,
        cv2.BORDER_CONSTANT, value=255,
    )
    return Image.fromarray(bordered)

def draw_ruler(c: rl_canvas.Canvas, x_mm: float, y_mm: float, length_mm: float = 25):
    """Draw a 25 mm reference ruler with mm ticks."""
    c.setLineWidth(0.3)
    c.line(x_mm * mm, y_mm * mm, (x_mm + length_mm) * mm, y_mm * mm)
    for i in range(length_mm + 1):
        tick = 1.5 if i % 5 == 0 else 0.8
        c.line((x_mm + i) * mm, y_mm * mm, (x_mm + i) * mm, (y_mm + tick) * mm)
    c.setFont("Helvetica", 7)
    c.drawString(x_mm * mm, (y_mm - 3) * mm, f"verify: {length_mm} mm")

def write_pdf(marker_id: int, size_mm: float, out_path: Path, label: str | None = None) -> None:
    c = rl_canvas.Canvas(str(out_path), pagesize=A4)
    img = make_marker_image(marker_id, size_mm)

    # Position marker top-left on the page with a 20 mm margin.
    margin_mm   = 20
    quiet_mm    = size_mm / 4                       # quiet zone width either side
    total_mm    = size_mm + 2 * quiet_mm            # printed bitmap covers this
    page_h_mm   = A4[1] / mm
    x_mm        = margin_mm
    y_top_mm    = page_h_mm - margin_mm
    y_bot_mm    = y_top_mm - total_mm

    c.drawImage(
        ImageReader(img),
        x_mm * mm, y_bot_mm * mm,
        width=total_mm * mm, height=total_mm * mm,
    )

    # Label below.
    c.setFont("Helvetica", 10)
    spec = f"ArUco DICT_4X4_50  ID {marker_id}  {size_mm:.0f} mm"
    if label:
        spec = f"{label}  —  {spec}"
    c.drawString(x_mm * mm, (y_bot_mm - 6) * mm, spec)

    # Reference ruler under the label so it prints alongside the marker.
    draw_ruler(c, x_mm, y_bot_mm - 14, length_mm=25)

    # Crop guide — outer edge of the quiet zone. Cut on this line.
    c.setDash(2, 2)
    c.setLineWidth(0.2)
    c.rect(x_mm * mm, y_bot_mm * mm, total_mm * mm, total_mm * mm)
    c.setDash()

    c.showPage()
    c.save()

def write_png(marker_id: int, size_mm: float, out_path: Path) -> None:
    img = make_marker_image(marker_id, size_mm)
    img.save(out_path, dpi=(DPI, DPI))      # DPI metadata so naive PNG prints don't scale

def write_sheet(ids: list[int], size_mm: float, out_path: Path,
                label: str | None = None) -> None:
    """Tile every requested marker on ONE A4 page in a grid — handy for small
    markers (e.g. 20 mm claw-tip tags) so you print and cut them together."""
    c = rl_canvas.Canvas(str(out_path), pagesize=A4)
    page_w_mm, page_h_mm = A4[0] / mm, A4[1] / mm
    margin_mm  = 15
    quiet_mm   = size_mm / 4
    total_mm   = size_mm + 2 * quiet_mm          # printed bitmap incl. quiet zone
    gap_mm     = 10                              # space between tiles for the ID label
    cell_mm    = total_mm + gap_mm
    cols       = max(1, int((page_w_mm - 2 * margin_mm) // cell_mm))

    for i, mid in enumerate(ids):
        row, col = divmod(i, cols)
        x_mm = margin_mm + col * cell_mm
        y_top_mm = page_h_mm - margin_mm - row * (cell_mm + 6)   # +6 for label row
        y_bot_mm = y_top_mm - total_mm
        c.drawImage(ImageReader(make_marker_image(mid, size_mm)),
                    x_mm * mm, y_bot_mm * mm,
                    width=total_mm * mm, height=total_mm * mm)
        c.setFont("Helvetica", 7)
        c.drawString(x_mm * mm, (y_bot_mm - 3) * mm,
                     f"ID {mid}  {size_mm:.0f}mm")
        c.setDash(2, 2); c.setLineWidth(0.2)
        c.rect(x_mm * mm, y_bot_mm * mm, total_mm * mm, total_mm * mm)
        c.setDash()

    # One verification ruler at the bottom of the page.
    draw_ruler(c, margin_mm, margin_mm, length_mm=25)
    c.setFont("Helvetica", 9)
    spec = f"ArUco DICT_4X4_50  {size_mm:.0f} mm  IDs {ids}"
    if label:
        spec = f"{label}  —  {spec}"
    c.drawString(margin_mm * mm, (margin_mm + 6) * mm, spec)
    c.showPage()
    c.save()

def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--ids",  type=int, nargs="+", default=[0, 1, 2, 3])
    p.add_argument("--size", type=float, default=25.0, help="marker side length in mm")
    p.add_argument("--label", type=str, default=None,
                   help="optional role text printed beside the spec, e.g. 'BASE HEADING'")
    p.add_argument("--sheet", action="store_true",
                   help="tile all IDs on ONE page (good for small tip markers)")
    args = p.parse_args()

    out_dir = Path(__file__).parent / "markers"
    out_dir.mkdir(exist_ok=True)

    if args.sheet:
        stem = f"aruco_sheet_{int(args.size)}mm_ids" + "-".join(str(i) for i in args.ids)
        sheet_path = out_dir / f"{stem}.pdf"
        write_sheet(args.ids, args.size, sheet_path, label=args.label)
        print(f"  wrote {sheet_path}")
        print()
        print("Print at 100 % / 'Actual Size'. Verify the bottom ruler reads 25 mm.")
        return

    for mid in args.ids:
        stem    = f"aruco_id{mid:02d}_{int(args.size)}mm"
        pdf_path = out_dir / f"{stem}.pdf"
        png_path = out_dir / f"{stem}.png"
        write_pdf(mid, args.size, pdf_path, label=args.label)
        write_png(mid, args.size, png_path)
        print(f"  wrote {pdf_path}")
        print(f"  wrote {png_path}")

    print()
    print("Print the PDF at 100 % / 'Actual Size' (not 'Fit to Page').")
    print("Measure the printed reference ruler with a real ruler — if the")
    print("printed ruler reads 25 mm against your real ruler, the marker")
    print("is also at the correct physical size.")

if __name__ == "__main__":
    main()
