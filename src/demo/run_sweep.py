#!/usr/bin/env python3
"""Run //demo:demo across a Cartesian product of parameter values.

This script invokes `bazel run //demo:demo -- ...` once per combination of the
parameter grid specified via CLI flags, captures stdout/stderr, scrapes a few
summary metrics from the LOG_INFO lines emitted at the end of main.cpp, and
prints a final summary table using pandas for data processing and tabulate for
pretty printing.

Run with:
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10,20

Examples:
    # Sweep over routers and agents
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10,20

    # Add a fixed argument to every invocation
    bazel run //demo:run_sweep -- -p router=astar,stat --fixed --tick --fixed 0.05

    # Filter to specific combos
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10 --filter agents=10

    # Sort results by multiple columns
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10,20 --sort-by router,agents:desc,avg_wait

    # Display only specific columns
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10,20 --columns router,agents,avg_wait,time_s

    # Combine sorting and column filtering
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10,20 --sort-by avg_wait:desc --columns router,agents,avg_wait

    # Dry run to see what would be executed
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10 --dry-run
"""

from __future__ import annotations

import argparse
import itertools
import os
import re
import subprocess
import sys
import time
from typing import Any

import pandas as pd
from tabulate import tabulate


# ---------------------------------------------------------------------------
# Metric scraping: regexes against LOG_INFO output from main.cpp tail.
# ---------------------------------------------------------------------------

_METRIC_PATTERNS: dict[str, re.Pattern[str]] = {
    "orders":       re.compile(r"Number of orders:\s*([-\d.eE+]+)"),
    "conflicts":    re.compile(r"Number of conflicts:\s*([-\d.eE+]+)"),
    "avg_wait":     re.compile(r"Average waiting time:\s*([-\d.eE+]+)"),
    "avg_reverse":  re.compile(r"Average reverse time:\s*([-\d.eE+]+)"),
    "avg_speed":    re.compile(r"Average speed:\s*([-\d.eE+]+)"),
    "sim_speed":    re.compile(r"Simulation speed:\s*([-\d.eE+]+)"),
}


def _scrape_metrics(text: str) -> dict[str, str]:
    found: dict[str, str] = {}
    for name, pat in _METRIC_PATTERNS.items():
        m = pat.search(text)
        if m:
            found[name] = m.group(1)
    return found


def _parse_sort_spec(sort_by: str) -> list[tuple[str, bool]]:
    """Parse sort specification string into list of (column, ascending) tuples.

    Args:
        sort_by: Comma-separated list of column names with optional :desc suffix

    Returns:
        List of (column_name, ascending) tuples

    Examples:
        "router,agents" -> [("router", True), ("agents", True)]
        "avg_wait:desc,time_s" -> [("avg_wait", False), ("time_s", True)]
    """
    if not sort_by:
        return []

    spec = []
    for col in sort_by.split(","):
        col = col.strip()
        if not col:
            continue
        if col.endswith(":desc"):
            spec.append((col[:-5].strip(), False))
        elif col.endswith(":asc"):
            spec.append((col[:-4].strip(), True))
        else:
            spec.append((col, True))
    return spec

# ---------------------------------------------------------------------------
# Combo generation.
# ---------------------------------------------------------------------------

def _generate_combos(
    grid: dict[str, list[Any]],
    filters: dict[str, str],
) -> list[dict[str, Any]]:
    keys = list(grid.keys())
    value_lists = [grid[k] for k in keys]
    combos: list[dict[str, Any]] = []
    for values in itertools.product(*value_lists):
        combo = dict(zip(keys, values))
        # Apply --filter k=v constraints (k may be the bare name, e.g. "router").
        ok = True
        for fk, fv in filters.items():
            flag = fk if fk.startswith("--") else f"--{fk}"
            if flag not in combo:
                ok = False
                break
            if str(combo[flag]) != fv:
                ok = False
                break
        if ok:
            combos.append(combo)
    return combos


def _combo_to_argv(combo: dict[str, Any], fixed_args: list[str]) -> list[str]:
    argv: list[str] = []
    for flag, value in combo.items():
        argv.extend([flag, str(value)])
    argv.extend(fixed_args)
    return argv


def _format_combo(combo: dict[str, Any]) -> str:
    return " ".join(f"{k.lstrip('-')}={v}" for k, v in combo.items())


# ---------------------------------------------------------------------------
# Subprocess runner.
# ---------------------------------------------------------------------------

def _resolve_workspace_root() -> str:
    """Return the workspace root we should cwd into before `bazel run`.

    When launched via `bazel run`, BUILD_WORKSPACE_DIRECTORY points at the
    workspace root containing MODULE.bazel. Fall back to script-relative.
    """
    root = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if root and os.path.isdir(root):
        return root
    # Fallback: this file lives at <workspace>/demo/run_sweep.py.
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _run_one(
    combo: dict[str, Any],
    fixed_args: list[str],
    workspace_root: str,
    timeout: float | None,
) -> dict[str, Any]:
    argv = ["bazel", "run", "--ui_event_filters=-info,-stdout,-stderr",
            "--noshow_progress", "//demo:demo", "--"] + _combo_to_argv(combo, fixed_args)
    start = time.monotonic()
    try:
        proc = subprocess.run(
            argv,
            cwd=workspace_root,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        elapsed = time.monotonic() - start
        return {
            "combo": combo,
            "argv": argv,
            "returncode": proc.returncode,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
            "elapsed": elapsed,
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as e:
        elapsed = time.monotonic() - start
        def _to_str(data: Any) -> str:
            if isinstance(data, bytes):
                return data.decode()
            if isinstance(data, (bytearray, memoryview)):
                return bytes(data).decode()
            return data or ""
        return {
            "combo": combo,
            "argv": argv,
            "returncode": None,
            "stdout": _to_str(e.stdout),
            "stderr": _to_str(e.stderr) + f"\n[timeout after {timeout}s]",
            "elapsed": elapsed,
            "timed_out": True,
        }


# ---------------------------------------------------------------------------
# Output formatting.
# ---------------------------------------------------------------------------

def _print_run_result(idx: int, total: int, result: dict[str, Any]) -> None:
    bar = "=" * 78
    print(f"\n{bar}")
    print(f"[{idx}/{total}] {_format_combo(result['combo'])}")
    print(f"  cmd: {' '.join(result['argv'])}")
    rc = result["returncode"]
    rc_str = "TIMEOUT" if result["timed_out"] else str(rc)
    print(f"  returncode={rc_str}  elapsed={result['elapsed']:.2f}s")
    print(bar)
    # if result["stdout"]:
    #     print("--- stdout ---")
    #     print(result["stdout"].rstrip())
    if result["stderr"]:
        print("--- stderr ---")
        print(result["stderr"].rstrip())


def _results_to_dataframe(results: list[dict[str, Any]]) -> pd.DataFrame:
    """Convert results list to pandas DataFrame for easy manipulation.

    Args:
        results: List of result dictionaries from _run_one

    Returns:
        DataFrame with columns for parameters, rc, time_s, and metrics
    """
    if not results:
        return pd.DataFrame()

    # Get parameter keys from first result
    param_keys = [k.lstrip("-") for k in results[0]["combo"].keys()]
    metric_keys = list(_METRIC_PATTERNS.keys())

    # Build data for DataFrame
    data = []
    for r in results:
        metrics = _scrape_metrics((r["stdout"] or "") + "\n" + (r["stderr"] or ""))
        rc_str = "TIMEOUT" if r["timed_out"] else (
            "-" if r["returncode"] is None else str(r["returncode"]))

        row = {}
        # Add parameters
        for k in param_keys:
            row[k] = r["combo"][f"--{k}"]

        # Add metadata
        row["rc"] = rc_str
        row["time_s"] = r["elapsed"]

        # Add metrics
        for mk in metric_keys:
            row[mk] = metrics.get(mk, "-")

        data.append(row)

    df = pd.DataFrame(data)

    return df


def _print_summary(
    results: list[dict[str, Any]],
    sort_by: str | None = None,
    columns: str | None = None
) -> None:
    """Print summary table using pandas for sorting/filtering and tabulate for formatting.

    Args:
        results: List of result dictionaries from _run_one
        sort_by: Comma-separated list of columns to sort by (with optional :desc suffix)
        columns: Comma-separated list of columns to display
    """
    if not results:
        return

    # Convert to DataFrame
    df = _results_to_dataframe(results)

    # Apply sorting if specified
    if sort_by:
        sort_spec = _parse_sort_spec(sort_by)
        if sort_spec:
            sort_columns = []
            ascending = []
            for col, asc in sort_spec:
                if col in df.columns:
                    sort_columns.append(col)
                    ascending.append(asc)
                else:
                    print(f"Warning: Sort column '{col}' not found in data", file=sys.stderr)

            if sort_columns:
                df = df.sort_values(by=sort_columns, ascending=ascending)

    # Apply column filtering if specified
    if columns:
        col_list = [col.strip() for col in columns.split(",") if col.strip()]
        # Validate columns exist
        valid_cols = [col for col in col_list if col in df.columns]
        missing_cols = [col for col in col_list if col not in df.columns]

        if missing_cols:
            print(f"Warning: Columns not found: {', '.join(missing_cols)}", file=sys.stderr)

        if valid_cols:
            df = df[valid_cols]

    # Convert DataFrame to list of lists for tabulate
    # Replace NaN with "-" for display
    df_display = df.fillna("-")

    print("\n" + "#" * 78)
    print("# Summary")
    print("#" * 78)

    # Use tabulate for pretty printing
    table = tabulate(
        df_display.values.tolist(),
        headers=df_display.columns.tolist(),
        tablefmt="simple",
        disable_numparse=True,
    )
    print(table)


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run //demo:demo across a Cartesian product of parameters.")
    parser.add_argument("-p", "--param", action="append", default=[],
                        metavar="KEY=VAL[,VAL,...]",
                        help="Add a parameter axis. KEY is the flag name (with or "
                             "without leading --). VAL is a comma-separated list of "
                             "values. Multiple -p for the same KEY extend the list.")
    parser.add_argument("--fixed", action="append", default=[],
                        metavar="ARG",
                        help="Add a fixed argument to every invocation. May be passed "
                             "multiple times.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the combos that would run, then exit.")
    parser.add_argument("--filter", action="append", default=[],
                        metavar="KEY=VAL",
                        help="Restrict combos to those matching KEY=VAL. "
                             "May be passed multiple times. KEY is the flag "
                             "name without leading dashes (e.g. router=astar).")
    parser.add_argument("--timeout", type=float, default=None,
                        help="Per-run timeout in seconds (default: none).")
    parser.add_argument("--stop-on-error", action="store_true",
                        help="Abort the sweep on the first non-zero return code.")
    parser.add_argument("--sort-by", default=None,
                        metavar="COL[,COL:desc,...]",
                        help="Sort summary by specified columns. Columns are sorted in "
                             "the order specified. Append ':desc' for descending order. "
                             "Available columns: parameter names, rc, time_s, and "
                             "metrics (orders, conflicts, avg_wait, avg_reverse, avg_speed, sim_speed). "
                             "Example: --sort-by router,agents:desc,avg_wait")
    parser.add_argument("--columns", default=None,
                        metavar="COL[,COL,...]",
                        help="Display only specified columns in the given order. "
                             "If not specified, all columns are shown. "
                             "Available columns: parameter names, rc, time_s, and "
                             "metrics (orders, conflicts, avg_wait, avg_reverse, avg_speed, sim_speed). "
                             "Example: --columns router,agents,avg_wait,time_s")
    args = parser.parse_args()

    # Parse --param KEY=VAL[,VAL,...] into a grid dict.
    cli_grid: dict[str, list[str]] = {}
    for spec in args.param:
        if "=" not in spec:
            print(f"error: --param expects KEY=VAL[,VAL,...], got: {spec}", file=sys.stderr)
            return 2
        k, v = spec.split("=", 1)
        key = k.strip()
        if not key.startswith("--"):
            key = f"--{key}"
        values = [val.strip() for val in v.split(",") if val.strip()]
        if not values:
            print(f"error: --param {spec} has no values", file=sys.stderr)
            return 2
        if key not in cli_grid:
            cli_grid[key] = []
        cli_grid[key].extend(values)

    if not cli_grid:
        print("error: at least one --param is required", file=sys.stderr)
        return 2

    # Parse --filter KEY=VAL into a dict.
    filters: dict[str, str] = {}
    for spec in args.filter:
        if "=" not in spec:
            print(f"error: --filter expects KEY=VAL, got: {spec}", file=sys.stderr)
            return 2
        k, v = spec.split("=", 1)
        filters[k.strip()] = v.strip()

    # Skip predicate: ccbs router requires --resolver=none.
    def _should_skip(combo: dict[str, Any]) -> str | None:
        if combo.get("--router") == "ccbs" and combo.get("--resolver", "none") != "none":
            return "ccbs router requires --resolver=none"
        return None

    all_combos = _generate_combos(cli_grid, filters)
    runnable: list[dict[str, Any]] = []
    skipped: list[tuple[dict[str, Any], str]] = []
    for combo in all_combos:
        reason = _should_skip(combo)
        if reason:
            skipped.append((combo, reason))
        else:
            runnable.append(combo)

    print(f"Generated {len(all_combos)} combos: "
          f"{len(runnable)} runnable, {len(skipped)} skipped.")
    if skipped:
        print("Skipped:")
        for combo, reason in skipped:
            print(f"  - {_format_combo(combo)}  ({reason})")

    if args.dry_run:
        print("Runnable combos:")
        for combo in runnable:
            print(f"  - {_format_combo(combo)}")
        return 0

    workspace_root = _resolve_workspace_root()
    print(f"Workspace root: {workspace_root}")

    results: list[dict[str, Any]] = []
    failures = 0
    for i, combo in enumerate(runnable, 1):
        result = _run_one(combo, args.fixed, workspace_root, args.timeout)
        results.append(result)
        _print_run_result(i, len(runnable), result)
        if result["timed_out"] or (result["returncode"] not in (0, None)):
            failures += 1
            if args.stop_on_error:
                print("Stopping early due to --stop-on-error.")
                break

    _print_summary(results, sort_by=args.sort_by, columns=args.columns)
    print(f"\nDone: {len(results)} runs, {failures} failure(s).")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
