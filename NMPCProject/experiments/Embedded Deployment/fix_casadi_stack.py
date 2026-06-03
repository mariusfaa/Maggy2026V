#!/usr/bin/env python3
"""
Rewrites CasADi-generated C files to move local `casadi_real aNNNN` variables
from the stack into a function-local `static casadi_real a[N]` array (BSS).

Handles files with multiple `casadi_fN` functions: each function body is
processed independently and gets its own static array sized to that function's
own max variable index. Idempotent — re-running on an already-fixed file is a
no-op.

Usage:
    python fix_casadi_stack.py file1.c [file2.c ...]
"""
import os
import re
import sys

# Matches the start of a CasADi-generated function we want to touch.
# Examples:
#   static int casadi_f0(const casadi_real** arg, ...) {
#   static int casadi_f12(...) {
FUNC_START_RE = re.compile(
    r'^\s*(?:static\s+)?(?:int|void|casadi_real)\s+casadi_f\d+\s*\('
)

# A declaration line: "  casadi_real a0000, a0001, ...;"  (1+ digit indices)
DECL_LINE_RE = re.compile(r'^\s*casadi_real\s+a\d+\s*(?:,\s*a\d+\s*)*;\s*$')

# A line that already has the fixed form: "  static casadi_real a[N];"
ALREADY_FIXED_RE = re.compile(r'^\s*static\s+casadi_real\s+a\s*\[\s*\d+\s*\]\s*;\s*$')

# Pulls each variable index out of a declaration line.
DECL_VAR_RE = re.compile(r'\ba(\d+)\b')


def find_function_spans(lines):
    """Return list of (start_idx, end_idx) for each casadi_fN function body
    (inclusive start = line with `casadi_fN(...) {`, exclusive end). Functions
    are delimited by the next FUNC_START or EOF — close enough for our use,
    since helpers like casadi_fmax/casadi_fill never have aNNNN variables and
    we only act on lines we recognize as decls."""
    starts = [i for i, ln in enumerate(lines) if FUNC_START_RE.match(ln)]
    spans = []
    for k, s in enumerate(starts):
        e = starts[k + 1] if k + 1 < len(starts) else len(lines)
        spans.append((s, e))
    return spans


def process_function(lines, start, end):
    """Mutate lines[start:end] in place: replace the contiguous declaration
    block with a single `static casadi_real a[N];` and rewrite all aNNNN
    references in the body to a[NNNN]. No-op if already fixed."""
    # Idempotency: if anywhere in the body we already have a `static casadi_real a[N];`,
    # assume this function is done.
    for i in range(start, end):
        if ALREADY_FIXED_RE.match(lines[i]):
            return 0  # 0 vars touched

    # Find the contiguous declaration block. CasADi puts it at the very top of
    # the function body, possibly preceded by blank lines and the opening brace.
    decl_start = None
    decl_end = None
    for i in range(start, end):
        if DECL_LINE_RE.match(lines[i]):
            if decl_start is None:
                decl_start = i
            decl_end = i + 1
        elif decl_start is not None:
            # Block ended.
            break

    if decl_start is None:
        return 0  # No CasADi local-variable decls in this function — nothing to do.

    # Collect every variable index declared in the block.
    indices = []
    for i in range(decl_start, decl_end):
        indices.extend(int(m) for m in DECL_VAR_RE.findall(lines[i]))
    if not indices:
        return 0

    n = max(indices) + 1
    # Preserve the indentation of the first decl line for the replacement.
    indent_match = re.match(r'^(\s*)', lines[decl_start])
    indent = indent_match.group(1) if indent_match else '  '
    replacement = f'{indent}static casadi_real a[{n}];'

    # Rewrite the body: replace the whole decl block with one line, and rewrite
    # `aNNNN` references in every other line of this function.
    new_body = []
    for i in range(start, end):
        if i == decl_start:
            new_body.append(replacement)
        elif decl_start < i < decl_end:
            continue  # drop additional decl lines
        else:
            new_body.append(DECL_VAR_RE.sub(lambda m: f'a[{int(m.group(1))}]', lines[i]))

    # Splice back into `lines`.
    lines[start:end] = new_body
    return n


def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    lines = content.split('\n')

    spans = find_function_spans(lines)
    if not spans:
        print(f"  {os.path.basename(filepath)}: no casadi_fN functions found, skipping.")
        return

    # Process from the bottom up so earlier (start, end) indices stay valid.
    results = []
    for s, e in reversed(spans):
        n = process_function(lines, s, e)
        if n:
            results.append(n)

    if not results:
        print(f"  {os.path.basename(filepath)}: already fixed or no work to do.")
        return

    total_bytes = sum(n * 8 for n in results)
    print(
        f"  {os.path.basename(filepath)}: rewrote {len(results)} function(s), "
        f"sizes={sorted(results, reverse=True)} ({total_bytes} bytes stack -> static)"
    )

    with open(filepath, 'w') as f:
        f.write('\n'.join(lines))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for fpath in sys.argv[1:]:
        fix_file(fpath)
