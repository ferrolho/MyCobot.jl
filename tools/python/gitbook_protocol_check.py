#!/usr/bin/env python3
"""Cross-check Elephant's protocol documentation for the myCobot 280 for Arduino.

Downloads the English and Chinese GitBook pages of the communication protocol, compares
them table by table, checks each frame's length byte, and maps each command code to the
names in pymycobot. Prints a report and, with --markdown, the command table for the docs.

Usage:
    python3 gitbook_protocol_check.py [--pymycobot PATH] [--markdown]
"""
import argparse
import html
import re
import sys
import urllib.request
from pathlib import Path

PAGE = "3-FunctionsAndApplications/6.developmentGuide/CommunicationProtocolPackage/18-communication.html"
BOOKS = {
    "en": "https://docs.elephantrobotics.com/docs/mycobot_280_ar_en/",
    "cn": "https://docs.elephantrobotics.com/docs/mycobot_280_ar_cn/",
}


def fetch(url):
    with urllib.request.urlopen(url, timeout=30) as r:
        return r.read().decode("utf8", errors="replace")


def text(fragment):
    return re.sub(r"\s+", " ", html.unescape(re.sub(r"<[^>]+>", " ", fragment))).strip()


def tables(page):
    """Return [(heading, rows)] for each table, rows = [[cell, ...], ...]."""
    body = re.search(r'<section class="normal markdown-section">(.*?)</section>', page, re.S).group(1)
    out = []
    for m in re.finditer(r"<table>(.*?)</table>", body, re.S):
        headings = re.findall(r"<h\d[^>]*>(.*?)</h\d>", body[: m.start()], re.S)
        rows = [[text(c) for c in re.findall(r"<t[dh][^>]*>(.*?)</t[dh]>", r, re.S)]
                for r in re.findall(r"<tr[^>]*>(.*?)</tr>", m.group(1), re.S)]
        out.append((text(headings[-1]) if headings else "", rows))
    return out


def pymycobot_codes(path):
    src = (Path(path) / "common.py").read_text()
    block = src[src.index("class ProtocolCode"):]
    block = block[: block.index("\nclass ", 10)]
    codes = {}
    for name, val in re.findall(r"^\s+([A-Z][A-Z0-9_]+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*$", block, re.M):
        codes.setdefault(int(val, 0), []).append(name)
    return codes


def frames(tabs):
    """Frame tables (rows 'Data[i]'), as dicts with code, length byte and data fields."""
    out = []
    for i, (heading, rows) in enumerate(tabs):
        data = [r for r in rows if r and r[0].startswith("Data[") and len(r) > 2]
        if len(data) < 5:
            continue
        reply = "return" in " ".join(r[1].lower() for r in data[:4]) or "返回" in " ".join(r[1] for r in data[:4])
        hexv = lambda s: int(s[2:], 16) if re.fullmatch(r"0[Xx][0-9A-Fa-f]+", s.replace(" ", "")) else None
        out.append(dict(table=i, heading=heading, reply=reply, length=hexv(data[2][2]), code=hexv(data[3][2]),
                        rows=data, fields=[r[2] for r in data[4:-1]]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pymycobot", default=str(Path(__file__).resolve().parents[3] / "pymycobot" / "pymycobot"))
    ap.add_argument("--markdown", action="store_true")
    a = ap.parse_args()

    tabs = {lang: tables(fetch(url + PAGE)) for lang, url in BOOKS.items()}
    en, cn = tabs["en"], tabs["cn"]
    problems = []
    if len(en) != len(cn):
        problems.append(f"table count: EN {len(en)}, CN {len(cn)}")
    nums = lambda s: sorted(re.findall(r"-?\d+(?:\.\d+)?", s))
    hexes = lambda s: re.findall(r"0X[0-9A-F]+", s.upper())
    for i, ((h, ra), (_, rb)) in enumerate(zip(en, cn)):
        if len(ra) != len(rb):
            problems.append(f"table {i} ({h}): EN {len(ra)} rows, CN {len(rb)} rows")
        for x, y in zip(ra, rb):
            for k in range(min(len(x), len(y))):
                if nums(x[k]) != nums(y[k]) or hexes(x[k]) != hexes(y[k]):
                    problems.append(f"table {i} ({h}) col {k}: EN '{x[k]}' vs CN '{y[k]}'")

    fr_en, fr_cn = frames(en), frames(cn)
    for f in fr_en:
        expected = len(f["rows"]) - 3  # rows after FE FE LEN: CMD, data..., FA
        if f["length"] is not None and f["length"] != expected:
            problems.append(f"table {f['table']} ({f['heading']}): length byte 0x{f['length']:02X}, frame has {expected}")

    codes = pymycobot_codes(a.pymycobot)
    unknown = sorted({f["code"] for f in fr_en if f["code"] is not None and f["code"] not in codes})
    shared = {c: n for c, n in codes.items() if len(n) > 1 and c in {f["code"] for f in fr_en}}

    print(f"EN tables {len(en)}, CN tables {len(cn)}, frames {len(fr_en)}, codes {len({f['code'] for f in fr_en})}")
    print(f"EN/CN differences and length errors: {len(problems)}")
    for p in problems:
        print("  " + p)
    print(f"codes not in pymycobot: {[hex(c) for c in unknown]}")
    print("codes with more than one pymycobot name:")
    for c, n in sorted(shared.items()):
        print(f"  0x{c:02X}: {', '.join(n)}")

    if a.markdown:
        by_code = {}
        cn_title = {f["table"]: f["heading"] for f in fr_cn}
        for f in fr_en:
            e = by_code.setdefault(f["code"], dict(en=f["heading"], cn=cn_title.get(f["table"], ""), req=None, rep=None))
            if f["reply"]:
                e["rep"] = f["fields"]
            elif e["req"] is None:
                e["req"] = f["fields"]
            else:  # a second request table is the reply in a few places
                e["rep"] = f["fields"]
        print("\n| Code | pymycobot | Elephant (EN / CN) | Request data | Reply data |")
        print("| --- | --- | --- | --- | --- |")
        def fmt(xs):
            # Join each high/low byte pair into one 16-bit field, written as name(h,l).
            xs = [re.sub(r"\s+", "", x) for x in xs]
            out, i = [], 0
            while i < len(xs):
                m = re.fullmatch(r"(.+?)_?high", xs[i], re.I)
                if m and i + 1 < len(xs) and re.fullmatch(re.escape(m.group(1)) + r"_?low", xs[i + 1], re.I):
                    out.append(m.group(1).rstrip("_") + "(h,l)")
                    i += 2
                else:
                    out.append(xs[i])
                    i += 1
            return "—" if not out else ", ".join(f"`{x}`" for x in out)
        for c in sorted(by_code):
            e = by_code[c]
            print(f"| `0x{c:02X}` | {' / '.join(codes.get(c, ['?']))} | {e['en']} / {e['cn']} | {fmt(e['req'])} | {fmt(e['rep']) if e['rep'] is not None else ''} |")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
