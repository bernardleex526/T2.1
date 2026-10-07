#!/usr/bin/env python3
"""Validate every relative markdown link and every python command in the repo docs.

Checks:
  1. every relative link target exists on disk;
  2. every `python3 tools/<tool>.py` invocation names a real file;
  3. code fences are balanced in every doc.
"""
import os
import re
import sys

# ROOT is the REPOSITORY root, not this script's directory: the script lives in
# tests/ and must scan the whole tree.
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DOCS = []
for base, _dirs, files in os.walk(ROOT):
    if any(p in base.replace('\\', '/').split('/')
           for p in ('.git', 'node_modules', '__pycache__', '.pytest_cache')):
        continue
    for f in files:
        if f.endswith('.md'):
            DOCS.append(os.path.join(base, f))

link_re = re.compile(r'\[[^\]]*\]\(([^)#\s]+)(?:#[^)]*)?\)')
tool_re = re.compile(r'python3\s+(tools/[A-Za-z0-9_./-]+\.py)')

# Paths the repository DELIBERATELY does not publish (README §七.2 and
# .gitignore): local evaluation artefacts, raw datasets, build output, and large
# binaries.  A link to one of these is not a broken link - it is a pointer to
# evidence that lives outside the public repo by policy.  They are counted and
# reported separately so a genuinely broken link cannot hide among them.
EXCLUDED_PREFIXES = (
    'artifacts/', '.omp/', 'data/', 'datasets/', 'build/', 'install/', 'log/',
)

bad_links, bad_tools, bad_fences, excluded_links = [], [], [], []

for path in sorted(DOCS):
    rel = os.path.relpath(path, ROOT).replace('\\', '/')
    text = open(path, encoding='utf-8').read()

    if text.count('```') % 2 != 0:
        bad_fences.append(f"{rel}: odd number of ``` markers ({text.count('```')})")

    for m in link_re.finditer(text):
        target = m.group(1)
        if target.startswith(('http://', 'https://', 'mailto:')):
            continue
        resolved = os.path.normpath(os.path.join(os.path.dirname(path), target))
        if os.path.exists(resolved):
            continue
        # Classify by where the link points RELATIVE TO THE REPO ROOT, computed
        # from the already-resolved path.  Joining ROOT with a target that itself
        # contains '..' would escape the repo and misclassify the link.
        try:
            rel_to_root = os.path.relpath(resolved, ROOT).replace('\\', '/')
        except ValueError:
            rel_to_root = ''
        if rel_to_root.startswith('..') or any(
                rel_to_root.startswith(p) for p in EXCLUDED_PREFIXES):
            excluded_links.append(f"{rel}: -> {target}")
        else:
            bad_links.append(f"{rel}: -> {target}")

    for m in tool_re.finditer(text):
        resolved = os.path.join(ROOT, m.group(1))
        if not os.path.exists(resolved):
            bad_tools.append(f"{rel}: {m.group(1)}")

print(f"scanned {len(DOCS)} markdown files under {ROOT}")
print(f"excluded-by-policy links (artefacts/ etc.): {len(excluded_links)}")
for label, items in (("BROKEN LINKS", bad_links), ("MISSING TOOLS", bad_tools),
                     ("UNBALANCED FENCES", bad_fences)):
    print(f"\n=== {label}: {len(items)} ===")
    for i in items:
        print(f"  {i}")

# A scan that finds nothing is a bug in the scanner, not a clean repo.
if len(DOCS) < 5:
    print("\nSCANNER ERROR: too few markdown files found; check ROOT")
    sys.exit(1)

ok = not (bad_links or bad_tools or bad_fences)
print("\n" + ("DOC VALIDATION PASSED" if ok else "DOC VALIDATION FAILED"))
sys.exit(0 if ok else 1)
