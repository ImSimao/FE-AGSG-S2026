#!/usr/bin/env python3
"""Flag CPython-only standard-library APIs in the firmware before it reaches the car.

The simulator runs the firmware on CPython, so every stdlib name CPython has and
MicroPython does not is invisible there and fails only on the board, mid-run:

    AttributeError: 'module' object has no attribute 'environ'    # os.environ
    AttributeError: 'module' object has no attribute 'hypot'      # math.hypot

Both cost a flash + a run to find. This walks the firmware's AST and reports any
attribute of a stdlib module that the MicroPython docs do not list, so the class
is caught in a second, offline.

    .venv/bin/python tools/mpy_audit.py                 # firmware/ by default
    .venv/bin/python tools/mpy_audit.py firmware/agsg   # one tree
    .venv/bin/python tools/mpy_audit.py --self-test     # prove the checks fire

Exit code is 1 when anything is found, so it can gate a flash script.

The allowlists below are transcribed from docs.micropython.org/en/latest/library/.
They are deliberately allowlists: a name has to be *documented* to pass, because
its absence on the board is what hurts. If MicroPython grows a function this file
does not know yet the finding is a false positive, and the fix is one line here.
A deliberate CPython-only use (a flag that is probed and falls back) is marked on
the line itself with `# mpy-ok` and is not reported.
"""

from __future__ import annotations

import argparse
import ast
import pathlib
import sys
from collections import namedtuple

Finding = namedtuple("Finding", "path line col module attr hint")

# --- what MicroPython actually has -------------------------------------------

ALLOWED = {
    "math": frozenset(
        """
        acos acosh asin asinh atan atan2 atanh ceil copysign cos cosh degrees
        erf erfc exp expm1 fabs floor fmod frexp gamma isfinite isinf isnan
        ldexp lgamma log log10 log2 modf pow radians sin sinh sqrt tan tanh
        trunc e pi
        """.split()
    ),
    "os": frozenset(
        """
        uname urandom chdir getcwd ilistdir listdir mkdir remove rmdir rename
        stat statvfs sync dupterm dupterm_notify mount umount
        VfsFat VfsLfs1 VfsLfs2 VfsPosix
        """.split()
    ),
    "sys": frozenset(
        """
        exit atexit print_exception settrace argv byteorder implementation
        maxsize modules path platform ps1 ps2 stderr stdin stdout
        tracebacklimit version version_info
        """.split()
    ),
    "time": frozenset(
        """
        gmtime localtime mktime sleep sleep_ms sleep_us ticks_ms ticks_us
        ticks_cpu ticks_add ticks_diff time time_ns
        """.split()
    ),
    "struct": frozenset("calcsize pack pack_into unpack unpack_from".split()),
    "json": frozenset("loads dumps".split()),
    "random": frozenset(
        "getrandbits seed randint randrange choice random uniform".split()
    ),
    "collections": frozenset("namedtuple OrderedDict deque".split()),
    "micropython": frozenset(
        """
        const opt_level alloc_emergency_exception_buf mem_info qstr_info
        stack_use native viper kbd_intr schedule
        """.split()
    ),
    "_thread": frozenset("start_new_thread stack_size allocate_lock get_ident exit".split()),
    "gc": frozenset(
        "enable disable collect mem_free mem_alloc threshold isenabled".split()
    ),
}

#: `import ustruct as struct` gives the same surface as the plain name.
ALIASES = {
    "utime": "time",
    "uos": "os",
    "usys": "sys",
    "ujson": "json",
    "ustruct": "struct",
    "ucollections": "collections",
    "urandom": "random",
    "ubinascii": "binascii",
}

#: Modules that stock MicroPython simply does not ship. Right-hand side is the
#: hint printed with the finding.
ABSENT = {
    "typing": "no typing on the board: drop the annotations",
    "dataclasses": "no dataclasses: use a plain class",
    "pathlib": "no pathlib: os.listdir() / open()",
    "threading": "use _thread (start_new_thread, allocate_lock)",
    "logging": "use print() or the firmware's Telemetry",
    "traceback": "use sys.print_exception(exc)",
    "itertools": "not shipped: write the loop",
    "functools": "not shipped: write the accumulation by hand",
    "enum": "not in stock builds: use module-level constants",
    "subprocess": "not on the board",
    "tempfile": "not on the board",
    "unittest": "not on the board: use firmware/debug_*.py",
    "argparse": "not on the board",
    "csv": "not shipped: split the line by hand",
    "datetime": "not in stock builds: time.localtime()",
    "decimal": "not shipped: use float",
    "statistics": "not shipped: write the formula",
    "textwrap": "not on the board",
    "shutil": "not on the board",
    "glob": "not shipped: os.listdir() + str.endswith()",
    "pickle": "not shipped",
    "os.path": "MicroPython's os has no path submodule",
}

#: Per-attribute advice for the names that fail most often.
HINTS = {
    ("math", "hypot"): "math.sqrt(x * x + y * y)",
    ("math", "inf"): "float('inf')",
    ("math", "nan"): "float('nan')",
    ("math", "isclose"): "abs(a - b) <= tol",
    ("math", "fsum"): "sum() over a plain float add",
    ("math", "prod"): "a loop, or 1.0 * ...",
    ("math", "gcd"): "a loop, or int math",
    ("os", "environ"): "no process environment on the board: guard and fall back",
    ("os", "getenv"): "no process environment on the board",
    ("os", "getpid"): "there is no process",
    ("os", "sep"): "'/' (the on-board FS is POSIX-style)",
    ("os", "linesep"): "'\\n'",
    ("os", "path"): "MicroPython's os has no path submodule",
    ("os", "system"): "not available",
    ("sys", "getsizeof"): "gc.mem_free() for a rough idea",
    ("sys", "intern"): "not available",
    ("time", "monotonic"): "time.ticks_ms() + ticks_diff()",
    ("time", "perf_counter"): "time.ticks_us() + ticks_diff()",
    ("time", "strftime"): "build the string with %-formatting",
    ("time", "strptime"): "parse it by hand",
    ("time", "process_time"): "not available",
    ("json", "dump"): "json.dumps() + write()",
    ("json", "load"): "json.loads(f.read())",
    ("collections", "defaultdict"): "dict.get(key, default)",
    ("collections", "Counter"): "a plain dict and a loop",
}

SUPPRESS = "mpy-ok"

#: Trees that do not run on the ESP32, so the ESP32's stdlib has no say:
#: firmware/agsg/Camera/ is the OpenMV H7 script (`sensor`, `image`, `pyb`), and
#: OpenMV's MicroPython fork does have time.clock() and friends.
SKIP_DIRS = {"Camera"}


def resolve(name: str) -> str:
    """`utime` -> `time`, so one allowlist covers both spellings."""
    return ALIASES.get(name, name)


class Audit(ast.NodeVisitor):
    """Walk one module, tracking which local names hold which stdlib module."""

    def __init__(self, path: str, source: str) -> None:
        self.path = path
        self.lines = source.splitlines()
        #: local name -> canonical module name, e.g. "_os" -> "os"
        self.modules: dict[str, str] = {}
        self.findings: list[Finding] = []

    # -- reporting ---------------------------------------------------------
    def report(self, node: ast.AST, module: str, attr: str) -> None:
        line = getattr(node, "lineno", 0)
        text = self.lines[line - 1] if 0 < line <= len(self.lines) else ""
        if SUPPRESS in text:
            return
        hint = HINTS.get((module, attr), "")
        self.findings.append(
            Finding(self.path, line, getattr(node, "col_offset", 0) + 1, module, attr, hint)
        )

    def report_raw(self, node: ast.AST, what: str, hint: str) -> None:
        line = getattr(node, "lineno", 0)
        text = self.lines[line - 1] if 0 < line <= len(self.lines) else ""
        if SUPPRESS in text:
            return
        self.findings.append(
            Finding(self.path, line, getattr(node, "col_offset", 0) + 1, what, "", hint)
        )

    # -- imports -----------------------------------------------------------
    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            if alias.name in ABSENT:
                self.report_raw(node, alias.name, ABSENT[alias.name])
                continue
            head = alias.name.split(".")[0]
            if "." in alias.name:
                # `import os.path`: the dotted module itself is the problem.
                dotted = alias.name.split(".")[0] + "." + alias.name.split(".")[1]
                if dotted in ABSENT:
                    self.report_raw(node, dotted, ABSENT[dotted])
                elif resolve(dotted) not in ALLOWED:
                    self.report_raw(
                        node, dotted, "only plain '%s' exists on MicroPython" % head
                    )
                continue
            if resolve(head) in ALLOWED:
                self.modules[alias.asname or head] = resolve(head)
        self.generic_visit(node)

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        if node.level or not node.module:
            self.generic_visit(node)
            return
        module = resolve(node.module)
        if node.module in ABSENT:
            self.report_raw(node, node.module, ABSENT[node.module])
            self.generic_visit(node)
            return
        if "." in node.module and module not in ALLOWED:
            if node.module in ABSENT:
                self.report_raw(node, node.module, ABSENT[node.module])
            self.generic_visit(node)
            return
        allowed = ALLOWED.get(module)
        if allowed is None:
            # machine, bno08x, sensor, a project module... nothing to say.
            self.generic_visit(node)
            return
        for alias in node.names:
            if alias.name == "*":
                continue
            if alias.name not in allowed:
                self.report(node, module, alias.name)
        self.generic_visit(node)

    # -- use ---------------------------------------------------------------
    def visit_Attribute(self, node: ast.Attribute) -> None:
        value = node.value
        if isinstance(value, ast.Name):
            module = self.modules.get(value.id)
            if module is not None:
                allowed = ALLOWED.get(module, ())
                if node.attr not in allowed and not node.attr.startswith("__"):
                    self.report(node, module, node.attr)
        self.generic_visit(node)


def audit_source(source: str, path: str = "<source>") -> list[Finding]:
    tree = ast.parse(source, filename=path)
    checker = Audit(path, source)
    checker.visit(tree)
    return checker.findings


def audit_path(root: pathlib.Path) -> list[Finding]:
    findings: list[Finding] = []
    if root.is_file():
        files = [root]
    else:
        files = sorted(
            p for p in root.rglob("*.py") if not SKIP_DIRS.intersection(p.parts)
        )
    for path in files:
        try:
            source = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as exc:
            print("skip %s: %s" % (path, exc), file=sys.stderr)
            continue
        try:
            findings.extend(audit_source(source, str(path)))
        except SyntaxError as exc:
            print("skip %s: %s" % (path, exc), file=sys.stderr)
    return findings


# ------------------------------------------------------------------ self-test

SELF_TEST = '''
import math, os, time, sys
from math import hypot
from ustruct import unpack_from


def f(dx, dy):
    a = math.hypot(dx, dy)
    b = os.environ.get("X")
    c = time.monotonic()
    d = math.sqrt(dx * dx + dy * dy)
    e = math.nan
    g = os.environ.get("Y")  # mpy-ok
    return a, b, c, d, e, g
'''


def self_test() -> int:
    found = {(f.line, f.module, f.attr) for f in audit_source(SELF_TEST)}
    expected = {
        (3, "math", "hypot"),        # from-import of a missing name
        (8, "math", "hypot"),        # math.hypot()
        (9, "os", "environ"),        # os.environ
        (10, "time", "monotonic"),   # time.monotonic()
        (12, "math", "nan"),         # math.nan
    }
    missing = expected - found
    extra = found - expected
    ok = not missing and not extra
    print("self-test:", "OK" if ok else "FAIL")
    for item in sorted(missing):
        print("  missed:", item)
    for item in sorted(extra):
        print("  false positive:", item)
    return 0 if ok else 1


# ----------------------------------------------------------------------- main


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "paths",
        nargs="*",
        type=pathlib.Path,
        help="files or trees to check (default: firmware/)",
    )
    parser.add_argument("--self-test", action="store_true", help="prove the checks fire")
    parser.add_argument("-q", "--quiet", action="store_true", help="count only")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()

    roots = args.paths or [pathlib.Path("robot")]
    findings: list[Finding] = []
    for root in roots:
        if not root.exists():
            print("%s: no such file or directory" % root, file=sys.stderr)
            return 2
        findings.extend(audit_path(root))

    findings.sort(key=lambda f: (f.path, f.line, f.col))
    if not args.quiet:
        for f in findings:
            where = "%s:%d:%d" % (f.path, f.line, f.col)
            what = f.module + ("." + f.attr if f.attr else "")
            hint = (" -> " + f.hint) if f.hint else ""
            print("%s: %s does not exist in MicroPython%s" % (where, what, hint))

    files = len({f.path for f in findings})
    if findings:
        print("\n%d CPython-only use(s) in %d file(s)" % (len(findings), files))
        return 1
    print("clean: %d file(s) checked against the MicroPython stdlib" % _count(roots))
    return 0


def _count(roots: list[pathlib.Path]) -> int:
    total = 0
    for root in roots:
        if root.is_file():
            total += 1
        else:
            total += len(
                [p for p in root.rglob("*.py") if not SKIP_DIRS.intersection(p.parts)]
            )
    return total


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
