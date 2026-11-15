#!/usr/bin/env python3
"""[EXP-*] / [EXP0-MEASURE] 태그를 모아 docs/EXP_TAGS.md를 만든다.

    python3 scripts/gen_exp_tags.py          # docs/EXP_TAGS.md 갱신
    python3 scripts/gen_exp_tags.py --check  # 내용이 다르면 종료 코드 1 (커밋 전 확인)
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {'.git', 'archive', 'docs', 'build', 'install', 'log', '__pycache__'}
EXTS = ('.py', '.yaml', '.sh', '.md')
TAG = re.compile(r'\[(EXP-VAR|EXP-FIXED|EXP-TUNE|EXP0-MEASURE)\]')
ORDER = ('EXP-VAR', 'EXP-FIXED', 'EXP-TUNE', 'EXP0-MEASURE')
TITLES = {'EXP-VAR': '독립변인 — 바꾸면 비교 조건이 바뀜',
          'EXP-FIXED': '통제변인 — 방법 간 절대 바꾸지 말 것',
          'EXP-TUNE': '민감도 분석 대상 — bringup 인자로 덮어쓰기 허용',
          'EXP0-MEASURE': 'Exp0 실측 후 교체할 가정값'}


def collect():
    rows = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
        for name in sorted(filenames):
            if not name.endswith(EXTS) or name == 'gen_exp_tags.py':
                continue
            path = os.path.join(dirpath, name)
            rel = os.path.relpath(path, ROOT)
            with open(path, encoding='utf-8', errors='replace') as f:
                for no, line in enumerate(f, 1):
                    for tag in TAG.findall(line):
                        rows.append((tag, rel, no, line.strip()))
    return rows


def render(rows):
    out = ['# 실험 태그 목록', '',
           '자동 생성 파일 (`python3 scripts/gen_exp_tags.py`). 직접 고치지 말 것.',
           '같은 줄에 태그가 여러 개면 각 표에 모두 나온다.', '']
    for tag in ORDER:
        sel = [r for r in rows if r[0] == tag]
        out += [f'## [{tag}] {TITLES[tag]} ({len(sel)}개)', '',
                '| 위치 | 내용 |', '|---|---|']
        for _, rel, no, text in sel:
            text = text.replace('|', '\\|')
            out.append(f'| [{rel}:{no}](../{rel}#L{no}) | `{text}` |')
        out.append('')
    return '\n'.join(out)


def main():
    content = render(collect())
    path = os.path.join(ROOT, 'docs', 'EXP_TAGS.md')
    if '--check' in sys.argv:
        old = open(path, encoding='utf-8').read() if os.path.exists(path) else ''
        sys.exit(0 if old == content else 1)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f'{path} 갱신')


if __name__ == '__main__':
    main()
