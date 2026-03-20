from __future__ import annotations

import argparse
from pathlib import Path

import fitz
import numpy as np
from rapidocr_onnxruntime import RapidOCR


def ocr_page(page: fitz.Page, engine: RapidOCR, dpi: int) -> list[str]:
    pix = page.get_pixmap(dpi=dpi, alpha=False)
    image = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)
    result, _ = engine(image)
    if not result:
        return []
    return [item[1].strip() for item in result if len(item) > 1 and item[1].strip()]


def build_markdown(pdf_path: Path, dpi: int) -> str:
    document = fitz.open(pdf_path)
    engine = RapidOCR()
    sections: list[str] = [f"# {pdf_path.stem}", ""]

    for index, page in enumerate(document, start=1):
        lines = ocr_page(page, engine, dpi)
        sections.append(f"## 第{index}页")
        sections.append("")
        if lines:
            sections.extend(lines)
        else:
            sections.append("[本页未识别到可用文字]")
        sections.append("")

    return "\n".join(sections).strip() + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="OCR a scanned PDF into Markdown")
    parser.add_argument("pdf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--dpi", type=int, default=200)
    args = parser.parse_args()

    markdown = build_markdown(args.pdf, args.dpi)
    args.output.write_text(markdown, encoding="utf-8")


if __name__ == "__main__":
    main()
