#!/usr/bin/env python3
"""Assert the gateway's headers still form the dependency DAG their include order claims.

THE CONTRACT IS WRITTEN IN iotdata_gateway.c, above the include block; TIERS below must say the
same thing. A struct member points DOWN a tier or SIDEWAYS within one, and an upward edge is a
function pointer the lower tier declares and _begin() fills in from iotdata_gateway.c -- so a
lower-tier header naming a higher tier's identifier is the violation this looks for.

It checks three things, and the second two matter as much as the first:
  1. every iotdata_gateway_*.h is in TIERS       -- a new header cannot go unchecked by being new
  2. the include order in iotdata_gateway.c      -- the comment's claim, enforced
  3. no back-edges                               -- no lower tier names a higher tier's identifier

Ownership is DERIVED from the files, not listed here: whatever a header defines, it owns. That is
deliberate -- iotdata_gateway_exec.h defines ddup_insert_handler, so a by-prefix check would
mis-attribute it to ddup and miss the edge. Adding a function to a tier needs no edit here.

Run: make check (or python3 tests/check_header_tiers.py) from the project directory.
"""

import os
import re
import sys

# The tiers, lowest first. Mirrors the comment above the include block in iotdata_gateway.c.
TIERS = [
    ("utilities", ["iotdata_gateway_util.h"]),
    ("network", ["iotdata_gateway_netw.h"]),
    ("node framework + content", ["iotdata_gateway_node.h", "iotdata_gateway_node_content.h"]),
    ("mesh, with survey and trace", ["iotdata_gateway_mesh.h", "iotdata_gateway_mesh_survey.h", "iotdata_gateway_mesh_trace.h"]),
    ("ddup", ["iotdata_gateway_ddup.h"]),
    ("stats + state", ["iotdata_gateway_stat.h", "iotdata_gateway_state.h"]),
    ("control + execute", ["iotdata_gateway_ctrl.h", "iotdata_gateway_exec.h"]),
]

# Included separately and TWICE, before everything, for the X-macro expansion -- it is a table, not
# a tier, and it is not part of the ordered block.
UNTIERED = ["iotdata_gateway_conf.h"]

MAIN = "iotdata_gateway.c"

# COMMENTS AND STRING LITERALS ARE NOT DEPENDENCIES. ddup.h's release-callback comment names
# process_sensor_packet to say where a held reading goes, which is exactly the documentation the
# callback seam deserves and not an edge at all -- a check that flags prose is a check that gets
# turned off. Blanked rather than deleted, so every ^ anchor below still sees its own line.
_LITERAL = re.compile(r"""("(?:\\.|[^"\\])*")|('(?:\\.|[^'\\])*')|(/\*.*?\*/)|(//[^\n]*)""", re.S)


def code_only(body, keep_strings=False):
    def blank(m):
        if m.group(1) or m.group(2):  # a string or char literal
            return m.group(0) if keep_strings else '""'
        return re.sub(r"[^\n]", " ", m.group(0))  # a comment, blanked; newlines kept

    return _LITERAL.sub(blank, body)


# What a header DEFINES, at top level. These files are clang-format'd, so a definition starts at
# column 0; anything indented is inside a function or an initialiser and is a use, not a definition.
DEFINES = [
    re.compile(r"^\}\s*(\w+_t)\s*;", re.M),                                  # typedef struct { ... } x_t;
    re.compile(r"^typedef\s+[^;]*?\(\s*\*\s*(\w+)\s*\)", re.M),              # typedef r (*x_t)(...)
    re.compile(r"^typedef\s+(?:enum|struct)?[^;{]*?\b(\w+_t)\s*;", re.M),     # typedef int x_t;
    re.compile(r"^(?:static\s+|inline\s+)*[A-Za-z_]\w*[\w \t\*]*?\b(\w+)\s*\([^;]*$", re.M),  # function definition
]

# Identifiers that are not a tier's to own: C keywords the function pattern can catch, and the
# control-flow words a multi-line signature can start with.
NOT_OWNED = {"if", "for", "while", "switch", "return", "sizeof", "do", "else", "typedef", "struct", "union", "enum"}


def defined_in(body):
    names = set()
    for pat in DEFINES:
        for m in pat.finditer(body):
            n = m.group(1)
            if n and n not in NOT_OWNED:
                names.add(n)
    return names


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    os.chdir(root)
    fail = []

    # 1. every header is accounted for
    listed = [f for _, fs in TIERS for f in fs] + UNTIERED
    present = sorted(f for f in os.listdir(".") if re.fullmatch(r"iotdata_gateway_\w+\.h", f))
    for f in present:
        if f not in listed:
            fail.append("%s is not in TIERS: add it to the right tier (and to the comment in %s), or to UNTIERED with a reason" % (f, MAIN))
    for f in listed:
        if f not in present:
            fail.append("%s is in TIERS but does not exist: renamed or removed?" % f)
    if fail:  # the rest reads these files, so stop here
        for m in fail:
            print("FAIL  %s" % m)
        return 1

    # 2. the include order in the main translation unit is the tier order
    # comments blanked but strings kept -- a commented-out #include is not an include, and the
    # header name it would name is a string literal
    main_body = code_only(open(MAIN, encoding="utf-8").read(), keep_strings=True)
    included = [m.group(1) for m in re.finditer(r'^#include\s+"(iotdata_gateway_\w+\.h)"', main_body, re.M)]
    ordered = [f for f in included if f not in UNTIERED]
    expected = [f for _, fs in TIERS for f in fs]
    if ordered != expected:
        fail.append("the include order in %s is not the tier order\n        it reads:  %s\n        expected:  %s" % (MAIN, " ".join(ordered), " ".join(expected)))

    # 3. no back-edges: a lower tier naming something a higher tier defines
    bodies = {f: code_only(open(f, encoding="utf-8").read()) for f in expected}
    owned = {f: defined_in(bodies[f]) for f in expected}
    for i, (lname, lfiles) in enumerate(TIERS):
        for lf in lfiles:
            body = bodies[lf]
            for hname, hfiles in TIERS[i + 1:]:
                for hf in hfiles:
                    hits = sorted(n for n in owned[hf] if n not in owned[lf] and re.search(r"\b%s\b" % re.escape(n), body))
                    if hits:
                        fail.append("BACK-EDGE  %s (%s) names %s (%s): %s\n        an upward edge is a callback the lower tier declares, not a direct reference" % (lf, lname, hf, hname, ", ".join(hits)))

    for m in fail:
        print("FAIL  %s" % m)
    if fail:
        return 1
    print("header tiers OK (%d headers, %d tiers, no back-edges)" % (len(expected), len(TIERS)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
