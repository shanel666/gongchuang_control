# -*- coding: utf-8 -*-
"""Minimal PDF text extractor (FlateDecode + ToUnicode CMap). No external deps."""
import re, sys, zlib

def load_objects(data):
    objs = {}
    for m in re.finditer(rb'(\d+)\s+(\d+)\s+obj', data):
        num = int(m.group(1))
        start = m.end()
        end = data.find(b'endobj', start)
        if end < 0:
            continue
        body = data[start:end]
        sm = re.search(rb'stream\r?\n', body)
        dictb = body[:sm.start()] if sm else body
        stream = None
        if sm:
            s = sm.end()
            e = body.rfind(b'endstream')
            if e > s:
                stream = body[s:e]
                stream = stream.rstrip(b'\r\n')
        objs[num] = (dictb, stream)
    return objs

def decode(obj):
    d, s = obj
    if s is None:
        return None
    if b'FlateDecode' in d:
        try:
            return zlib.decompress(s)
        except Exception:
            try:
                return zlib.decompressobj().decompress(s)
            except Exception:
                return None
    return s

def parse_tounicode(cmap):
    mp = {}
    txt = cmap.decode('latin-1')
    for blk in re.findall(r'beginbfchar(.*?)endbfchar', txt, re.S):
        for a, b in re.findall(r'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>', blk):
            src = int(a, 16)
            dst = ''.join(chr(int(b[i:i+4], 16)) for i in range(0, len(b), 4))
            mp[src] = dst
    for blk in re.findall(r'beginbfrange(.*?)endbfrange', txt, re.S):
        for a, b, c in re.findall(r'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>', blk):
            lo, hi, st = int(a, 16), int(b, 16), int(c, 16)
            for i in range(lo, min(hi, lo + 512) + 1):
                mp[i] = chr(st + (i - lo))
    return mp

def tokenize_content(cs):
    """Yield (op, args) for text ops, tracking text position."""
    out = []
    i = 0
    n = len(cs)
    stack = []
    cur = b''
    while i < n:
        c = cs[i:i+1]
        if c == b'(':
            # literal string with escapes
            j = i + 1
            depth = 1
            buf = []
            while j < n:
                ch = cs[j:j+1]
                if ch == b'\\':
                    nx = cs[j+1:j+2]
                    esc = {b'n': b'\n', b'r': b'\r', b't': b'\t', b'b': b'\b', b'f': b'\f',
                           b'(': b'(', b')': b')', b'\\': b'\\'}
                    if nx in esc:
                        buf.append(esc[nx]); j += 2; continue
                    if nx.isdigit():
                        o = cs[j+1:j+4]
                        m = re.match(rb'[0-7]{1,3}', o)
                        buf.append(bytes([int(m.group(0), 8) & 0xFF])); j += 1 + len(m.group(0)); continue
                    j += 2; continue
                if ch == b'(':
                    depth += 1
                elif ch == b')':
                    depth -= 1
                    if depth == 0:
                        j += 1
                        break
                buf.append(ch)
                j += 1
            stack.append(('str', b''.join(buf)))
            i = j
            continue
        if c == b'<':
            if cs[i:i+2] == b'<<':
                # dict, skip to matching >>
                j = i + 2
                depth = 1
                while j < n and depth:
                    if cs[j:j+2] == b'<<':
                        depth += 1; j += 2
                    elif cs[j:j+2] == b'>>':
                        depth -= 1; j += 2
                    else:
                        j += 1
                i = j
                continue
            j = cs.find(b'>', i + 1)
            hx = cs[i+1:j]
            hx = re.sub(rb'[^0-9A-Fa-f]', b'', hx)
            if len(hx) % 2:
                hx += b'0'
            stack.append(('hex', bytes.fromhex(hx.decode())))
            i = j + 1
            continue
        if c == b'[':
            stack.append(('arr_start', None)); i += 1; continue
        if c == b']':
            arr = []
            while stack and stack[-1][0] != 'arr_start':
                arr.append(stack.pop())
            if stack:
                stack.pop()
            arr.reverse()
            stack.append(('arr', arr))
            i += 1
            continue
        # token or number
        j = i
        while j < n and cs[j:j+1] not in b' \t\r\n()<>[]/':
            j += 1
        if j == i:
            i += 1
        else:
            tok = cs[i:j]
            if re.fullmatch(rb'[-+]?\d*\.?\d+', tok):
                stack.append(('num', float(tok)))
            else:
                op = tok.decode('latin-1')
                out.append((op, list(stack)))
                stack = []
            i = j
    return out

def unicode_from_bytes(b, cmap, two_byte):
    # Prefer GBK/GB18030 decode (many Chinese PDFs use GBK-EUC-H without ToUnicode)
    try:
        gbk = b.decode('gb18030')
    except Exception:
        gbk = None
    if gbk is not None and any('\u4e00' <= ch <= '\u9fff' for ch in gbk):
        return gbk
    if two_byte:
        s = ''
        for i in range(0, len(b) - 1, 2):
            code = (b[i] << 8) | b[i+1]
            s += cmap.get(code, '')
        return s
    s = ''
    for ch in b:
        s += cmap.get(ch, chr(ch) if 32 <= ch < 127 else '')
    return s

def extract(path):
    data = open(path, 'rb').read()
    objs = load_objects(data)
    # collect all tounicode maps merged
    merged = {}
    font_has_tounicode = set()
    for num, (d, s) in objs.items():
        if b'/ToUnicode' in d:
            m = re.search(rb'/ToUnicode\s+(\d+)\s+0\s+R', d)
            if m:
                ref = int(m.group(1))
                if ref in objs:
                    cm = decode(objs[ref])
                    if cm:
                        merged.update(parse_tounicode(cm))
                        font_has_tounicode.add(num)
    # pages
    pages = []
    for num, (d, s) in objs.items():
        if re.search(rb'/Type\s*/Page[^s]', d):
            pages.append(num)
    pages.sort()
    result = []
    for pno in pages:
        d, _ = objs[pno]
        cm = re.search(rb'/Contents\s+(\d+)\s+0\s+R', d)
        cs = b''
        if cm:
            cs = decode(objs.get(int(cm.group(1)), (b'', None))) or b''
        else:
            cm2 = re.search(rb'/Contents\s*\[(.*?)\]', d, re.S)
            if cm2:
                for r in re.findall(rb'(\d+)\s+0\s+R', cm2.group(1)):
                    cs += (decode(objs.get(int(r), (b'', None))) or b'') + b'\n'
        toks = tokenize_content(cs)
        two_byte = bool(merged)
        items = []
        line = ''
        tx = ty = 0.0
        last_ty = None
        for op, args in toks:
            if op in ('Tj', 'TJ'):
                if not args:
                    continue
                a = args[-1]
                parts = []
                if a[0] == 'str':
                    parts = [a[1]]
                elif a[0] == 'hex':
                    parts = [a[1]]
                elif a[0] == 'arr':
                    for t in a[1]:
                        if t[0] in ('str', 'hex'):
                            parts.append(t[1])
                txt = ''.join(unicode_from_bytes(p, merged, two_byte) for p in parts)
                if last_ty is not None and abs(ty - last_ty) > 2.0 and line.strip():
                    items.append(line)
                    line = ''
                last_ty = ty
                line += txt
            elif op == 'Tm' and len(args) >= 6:
                nums = [v for k, v in args if k == 'num']
                if len(nums) >= 6:
                    if line.strip():
                        items.append(line)
                        line = ''
                    tx, ty = nums[-2], nums[-1]
                    last_ty = ty
            elif op == 'Td' and len(args) >= 2:
                nums = [v for k, v in args if k == 'num']
                if len(nums) >= 2:
                    if abs(nums[-1]) > 2.0 and line.strip():
                        items.append(line)
                        line = ''
                    tx += nums[-2]; ty += nums[-1]
                    last_ty = ty
            elif op in ('T*', "'", '"'):
                if line.strip():
                    items.append(line)
                    line = ''
        if line.strip():
            items.append(line)
        result.append((pno, items))
    return result

if __name__ == '__main__':
    for pno, items in extract(sys.argv[1]):
        print('===== PAGE obj %d =====' % pno)
        for it in items:
            print(it)
