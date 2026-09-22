#!/usr/bin/env python3
"""Compile DSL text into the RAP / PCL design-book docx files.

DSL line grammar (block = consecutive non-empty lines):
  '# ' / '## ' / '### '      headings 1/2/3
  'FIG: file | caption'      embed figure from figs/ with Caption paragraph
  'EQ: text'                 monospace equation line
  'NOTE: text'               bold 'Design decision:' paragraph
  '- '                       bullet list item
  '1. ' (any digit.)         numbered list item
  '| a | b |'                table row (first row of block = header)
  'INCLUDE name'             include another DSL file (tools_rap dir)
  otherwise                  paragraph (lines joined with space)
"""
import sys, os, re
import docx
from docx.shared import Pt, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH

HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, 'figs')

def load_blocks(path):
    """Line-class aware blocking: directive lines (#,##,###,FIG:,EQ:,NOTE:) are
    single-line blocks; runs of same-class lines (plain paragraph, '- ' bullet,
    '1. ' numbered, '|' table row) form one block each."""
    import re as _re
    blocks, cur, curcls = [], [], None
    def flush():
        nonlocal cur, curcls
        if cur:
            blocks.append('\n'.join(cur))
        cur, curcls = [], None
    def cls_of(line):
        if line.startswith('# ') or line.startswith('## ') or line.startswith('### '): return 'h'
        if line.startswith('FIG: '): return 'fig'
        if line.startswith('EQ: '): return 'eq'
        if line.startswith('NOTE: '): return 'note'
        if line.startswith('- '): return 'b'
        if _re.match(r'^\d+\. ', line): return 'n'
        if line.startswith('|'): return 't'
        return 'p'
    for raw in open(path, encoding='utf-8'):
        line = raw.rstrip('\n')
        if line.startswith('INCLUDE '):
            flush(); blocks.append(('INCLUDE', line.split(None, 1)[1])); continue
        if not line.strip():
            flush(); continue
        c = cls_of(line)
        if c in ('h', 'fig', 'eq', 'note') or c != curcls:
            flush()
        cur.append(line); curcls = c
    flush()
    return blocks

def expand(blocks):
    out = []
    for b in blocks:
        if isinstance(b, tuple) and b[0] == 'INCLUDE':
            out.extend(expand(load_blocks(os.path.join(HERE, b[1]))))
        else:
            out.append(b)
    return out

def title_page(d, title, subtitle, docnum, meta_rows):
    d.add_paragraph(title, style='Title')
    p = d.add_paragraph(subtitle); p.runs[0].italic = True
    d.add_paragraph()
    t = d.add_table(rows=0, cols=2); t.style = 'Table Grid'
    for k, v in meta_rows:
        c = t.add_row().cells; c[0].text = k; c[1].text = v
    d.add_paragraph()
    d.add_paragraph(
        "Number labelling convention: [validated] = bit-exact simulated or measured result "
        "from the GPM v1.8 / GPV artifact set; [derived] = closed-form result of an equation "
        "stated in this book; [budget] = design target or resource estimate pending "
        "implementation; [simulated] = figure produced by the seeded numpy models shipped "
        "with this book's generator. No figure or number in this book claims measured "
        "hardware data.", style='Intense Quote')
    d.add_page_break()

def compile_dsl(blocks, d, figs_dir=None):
    FIGD = figs_dir or FIGS
    nfig = ntab = 0
    i = 0
    while i < len(blocks):
        b = blocks[i]; i += 1
        if b.startswith('# '):
            d.add_heading(b[2:], level=1)
        elif b.startswith('## '):
            d.add_heading(b[3:], level=2)
        elif b.startswith('### '):
            d.add_heading(b[4:], level=3)
        elif b.startswith('FIG: '):
            spec = b[5:]
            fn, cap = spec.split('|', 1)
            nfig += 1
            d.add_picture(os.path.join(FIGD, fn.strip()), width=Inches(6.3))
            d.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
            cp = d.add_paragraph(f'Figure {nfig}. {cap.strip()}', style='Caption')
        elif b.startswith('EQ: '):
            p = d.add_paragraph(); r = p.add_run(b[4:])
            r.font.name = 'Consolas'; r.font.size = Pt(9.5)
            p.paragraph_format.left_indent = Pt(18)
        elif b.startswith('NOTE: '):
            p = d.add_paragraph(); r = p.add_run('Design decision: '); r.bold = True
            p.add_run(b[6:])
        elif b.startswith('- '):
            d.add_paragraph(b[2:], style='List Bullet')
        elif re.match(r'^\d+\. ', b):
            d.add_paragraph(re.sub(r'^\d+\. ', '', b), style='List Number')
        elif b.startswith('|'):
            rows = [ [c.strip() for c in ln.strip().strip('|').split('|')]
                     for ln in b.split('\n') ]
            ntab += 1
            t = d.add_table(rows=1, cols=len(rows[0])); t.style = 'Table Grid'
            for c, h in zip(t.rows[0].cells, rows[0]):
                c.text = h
                for par in c.paragraphs:
                    for run in par.runs: run.bold = True
            for row in rows[1:]:
                cells = t.add_row().cells
                for c, v in zip(cells, row): c.text = v
            d.add_paragraph(f'Table {ntab}.', style='Caption')
        else:
            d.add_paragraph(' '.join(ln.strip() for ln in b.split('\n')))
    return nfig, ntab

def build(dsl_path, out_path, title, subtitle, docnum, meta, figs_dir=None):
    d = docx.Document()
    st = d.styles['Normal']; st.font.name = 'Calibri'; st.font.size = Pt(10.5)
    title_page(d, title, subtitle, docnum, meta)
    blocks = expand(load_blocks(dsl_path))
    nfig, ntab = compile_dsl(blocks, d, figs_dir)
    d.core_properties.title = title
    d.core_properties.author = 'GPV engineering documentation'
    d.core_properties.comments = docnum
    d.save(out_path)
    paras = len([p for p in d.paragraphs if p.text.strip()])
    heads = len([p for p in d.paragraphs if p.style.name.startswith('Heading')])
    print(f'{os.path.basename(out_path)}: paragraphs={paras} headings={heads} figures={nfig} tables={ntab}')

if __name__ == '__main__':
    import json
    jobs = json.load(open(sys.argv[1]))
    for j in jobs:
        build(j['dsl'], j['out'], j['title'], j['subtitle'], j['docnum'],
              [(m[0], m[1]) for m in j['meta']], j.get('figs'))
