#!/usr/bin/env python3
"""Append/refresh a run manifest with provenance: git head, hashes, commands, timestamps.

Usage:
    python3 run_manifest.py --out <run>/run_manifest.json --stage mapping \
        --set trajectory=/path/x.json --set bag_dir=/path/bag --set domain_id=87 \
        --add-cmd "python3 generate_test_bag.py ..." --hash prior_map=/path/map.pcd
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402


def parse_value(text):
    try:
        return json.loads(text)
    except (ValueError, TypeError):
        return text


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--stage", default=None, help="stage name, e.g. mapping / localization")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="set a top-level key (VALUE is parsed as JSON when possible)")
    ap.add_argument("--hash", action="append", default=[], metavar="KEY=PATH",
                    help="record sha256 of PATH under KEY (and KEY_path)")
    ap.add_argument("--add-cmd", action="append", default=[], metavar="CMD",
                    help="append an exact command to the manifest's command list")
    ap.add_argument("--note", action="append", default=[], metavar="TEXT")
    args = ap.parse_args(argv)

    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    man = sc.read_json(args.out) if os.path.isfile(args.out) else {}
    head, dirty = sc.git_head(repo)
    man.setdefault("repo", {})["git_head"] = head
    man["repo"]["git_dirty"] = dirty
    man.setdefault("stages", {})
    stage = args.stage or "misc"
    entry = man["stages"].setdefault(stage, {})
    entry["updated_utc"] = sc.utcnow()
    for item in args.set:
        key, _, value = item.partition("=")
        entry[key] = parse_value(value)
    for item in args.hash:
        key, _, path = item.partition("=")
        entry[key] = os.path.abspath(path)
        entry[key + "_sha256"] = sc.sha256_file(path)
        entry[key + "_exists"] = bool(path) and os.path.isfile(path)
    cmds = entry.setdefault("commands", [])
    cmds.extend(args.add_cmd)
    if args.note:
        entry.setdefault("notes", []).extend(args.note)
    man["last_updated_utc"] = sc.utcnow()
    sc.write_json(args.out, man)
    print("[run_manifest] %s stage=%s keys=%d commands=%d"
          % (args.out, stage, len(entry), len(cmds)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
