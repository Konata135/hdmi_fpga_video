"""Dump a PDF's text to a UTF-8 file so it can be searched with rg.
Usage: python tools/pdf_to_text.py <in.pdf> <out.txt>
Written for the vendor and board manuals in doc/参考-官方/ (the console on this
machine is GBK and mangles Chinese, so the text goes to a file rather than to
stdout).  Pin tables and the like are usually plain text in these PDFs; when a
table is an image instead, render the page and read that:
    pdftoppm -png -f 18 -l 19 <in.pdf> page
"""
import sys
from pypdf import PdfReader


def main():
    pdf, out = sys.argv[1], sys.argv[2]
    reader = PdfReader(pdf)
    with open(out, "w", encoding="utf-8") as f:
        f.write(f"pages: {len(reader.pages)}\n")
        for i, page in enumerate(reader.pages):
            f.write(f"\n===== page {i + 1} =====\n")
            try:
                f.write((page.extract_text() or "") + "\n")
            except Exception as e:                 # a page that will not extract
                f.write(f"(extract failed: {e})\n")
    print("written", out)


if __name__ == "__main__":
    main()
