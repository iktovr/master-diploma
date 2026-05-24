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

    # Per-combo time budget: re-run each combo until predicted next run would
    # blow the budget, then average results in the summary.
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10 --budget 60

    # Write a per-combo CSV with every individual run (string values preserved).
    bazel run //demo:run_sweep -- -p router=astar,stat -p agents=5,10 --budget 60 --output-dir /tmp/sweep_csv

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
from pathlib import Path
from typing import Any

import pandas as pd
from tabulate import tabulate


# ---------------------------------------------------------------------------
# Metric scraping: regexes against LOG_INFO output from main.cpp tail.
# ---------------------------------------------------------------------------

_METRIC_PATTERNS: dict[str, re.Pattern[str]] = {
    "orders":         re.compile(r"Number of orders:\s*([-\d.eE+]+)"),
    "conflicts":      re.compile(r"Number of conflicts:\s*([-\d.eE+]+)"),
    "avg_wait":       re.compile(r"Average waiting time:\s*([-\d.eE+]+)"),
    "avg_reverse":    re.compile(r"Average reverse time:\s*([-\d.eE+]+)"),
    "avg_speed":      re.compile(r"Average speed:\s*([-\d.eE+]+)"),
    "avg_order_time": re.compile(r"Average order time:\s*([-\d.eE+]+)"),
    "sim_speed":      re.compile(r"Simulation speed:\s*([-\d.eE+]+)"),
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

def _print_run_result(run: int, idx: int, total: int, result: dict[str, Any]) -> None:
    bar = "=" * 78
    print(f"\n{bar}")
    print(f"[{run}/{idx}/{total}] {_format_combo(result['combo'])}")
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


def _result_to_row(
    result: dict[str, Any],
    param_keys: list[str],
    metric_keys: list[str],
) -> dict[str, Any]:
    """Convert one _run_one result into a row dict with raw string values."""
    metrics = _scrape_metrics((result["stdout"] or "") + "\n" + (result["stderr"] or ""))
    if result["timed_out"]:
        rc_str = "TIMEOUT"
    elif result["returncode"] is None:
        rc_str = "-"
    else:
        rc_str = str(result["returncode"])

    row: dict[str, Any] = {}
    for k in param_keys:
        row[k] = result["combo"][f"--{k}"]
    row["rc"] = rc_str
    row["time_s"] = f"{result['elapsed']:.3f}"
    for mk in metric_keys:
        row[mk] = metrics.get(mk, "-")
    return row


# ---------------------------------------------------------------------------
# Budgeted per-combo execution.
# ---------------------------------------------------------------------------

def _run_combo_budgeted(
    combo: dict[str, Any],
    fixed_args: list[str],
    workspace_root: str,
    timeout: float | None,
    budget: float | None,
    combo_idx: int,
    combo_total: int,
    stop_on_error: bool,
    param_keys: list[str],
    metric_keys: list[str],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
    """Run a combo one or more times, respecting an optional wall-time budget.

    Returns:
        (raw_results, rows, failures)
        raw_results: list of dicts from _run_one (for verbose per-run printing).
        rows: list of stringly-typed row dicts (one per run) for CSV/aggregation.
        failures: count of runs that timed out or returned non-zero rc.
    """
    raw_results: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    failures = 0

    total_elapsed = 0.0
    avg = 0.0
    n = 0

    while True:
        result = _run_one(combo, fixed_args, workspace_root, timeout)
        n += 1
        total_elapsed += result["elapsed"]
        avg = total_elapsed / n

        raw_results.append(result)
        rows.append(_result_to_row(result, param_keys, metric_keys))
        _print_run_result(n, combo_idx, combo_total, result)

        is_failure = result["timed_out"] or (result["returncode"] not in (0, None))
        if is_failure:
            failures += 1

        if budget is not None:
            # Estimate remaining runs based on current running average.
            remaining_budget = max(0.0, budget - total_elapsed)
            est_remaining = int(remaining_budget // avg) if avg > 0 else 0
            print(f"  [combo {combo_idx}/{combo_total}] run {n} done, "
                  f"avg={avg:.2f}s, budget used {total_elapsed:.2f}/{budget:.2f}s, "
                  f"est remaining {est_remaining} run(s)")

        if stop_on_error and is_failure:
            break

        if budget is None:
            # Without a budget, the legacy behavior is exactly one run per combo.
            break

        # Stop if we are already over budget, or if the predicted next run would
        # exceed the budget by more than 0.5 * running_avg (i.e. budget + 0.5*avg).
        if total_elapsed >= budget:
            break
        if total_elapsed + avg * 0.5 > budget:
            break

    return raw_results, rows, failures


# ---------------------------------------------------------------------------
# CSV report writing.
# ---------------------------------------------------------------------------

_SAFE_CHARS = re.compile(r"[^A-Za-z0-9._=+-]+")


def _sanitize_combo_filename(combo: dict[str, Any]) -> str:
    parts = [f"{k.lstrip('-')}={v}" for k, v in combo.items()]
    stem = "_".join(parts)
    stem = _SAFE_CHARS.sub("_", stem)
    return stem or "combo"


def _write_combo_csv(
    output_dir: Path,
    combo: dict[str, Any],
    rows: list[dict[str, Any]],
) -> str:
    """Write per-combo CSV with string values preserved (no numeric coercion)."""
    if not rows:
        return ""
    path = output_dir / (_sanitize_combo_filename(combo) + ".csv")
    df = pd.DataFrame(rows).astype(str)
    df.to_csv(path, index=False)
    return str(path)


# ---------------------------------------------------------------------------
# Summary aggregation.
# ---------------------------------------------------------------------------

# Columns that should be averaged across runs in the summary.
_NUMERIC_SUMMARY_COLS: list[str] = ["time_s"] + list(_METRIC_PATTERNS.keys())


def _aggregate_rc(rc_values: list[str]) -> str:
    """Return the 'worst' rc seen across runs (TIMEOUT > non-zero > 0)."""
    if any(v == "TIMEOUT" for v in rc_values):
        return "TIMEOUT"
    non_zero = [v for v in rc_values if v not in ("0", "-")]
    if non_zero:
        return non_zero[0]
    if all(v == "-" for v in rc_values):
        return "-"
    return "0"


def _aggregate_combo_rows(
    rows: list[dict[str, Any]],
    param_keys: list[str],
) -> dict[str, Any]:
    """Average numeric columns across runs of a single combo.

    Non-numeric / missing values are converted with pd.to_numeric(errors='coerce')
    and then averaged with NaN ignored. If all values are NaN the column is '-'.
    """
    agg: dict[str, Any] = {}
    for k in param_keys:
        agg[k] = rows[0][k]

    agg["rc"] = _aggregate_rc([r["rc"] for r in rows])
    agg["runs"] = len(rows)

    for col in _NUMERIC_SUMMARY_COLS:
        series = pd.to_numeric(
            pd.Series([r.get(col, "-") for r in rows]), errors="coerce")
        mean = series.mean()
        if pd.isna(mean):
            agg[col] = "-"
        else:
            # Keep a compact numeric representation; tabulate's
            # disable_numparse=True means we control formatting here.
            agg[col] = f"{mean:.4g}"

    return agg


def _build_summary_dataframe(
    per_combo_rows: list[list[dict[str, Any]]],
    param_keys: list[str],
) -> pd.DataFrame:
    if not per_combo_rows:
        return pd.DataFrame()

    summary_rows = [
        _aggregate_combo_rows(rows, param_keys) for rows in per_combo_rows if rows
    ]
    if not summary_rows:
        return pd.DataFrame()

    # Preserve a stable column order:
    #   params..., rc, runs, time_s, metrics...
    columns = list(param_keys) + ["rc", "runs", "time_s"] + list(_METRIC_PATTERNS.keys())
    return pd.DataFrame(summary_rows, columns=columns)


def _print_summary(
    df: pd.DataFrame,
    sort_by: str | None = None,
    columns: str | None = None,
) -> None:
    """Print summary table using pandas for sorting/filtering and tabulate for formatting."""
    if df.empty:
        return

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
                # For numeric columns sort by their numeric interpretation so
                # that "10" sorts after "9", etc.
                tmp_keys: list[str] = []
                for col in sort_columns:
                    if col in _NUMERIC_SUMMARY_COLS:
                        key = f"__sort__{col}"
                        df[key] = pd.to_numeric(df[col], errors="coerce")
                        tmp_keys.append(key)
                    else:
                        tmp_keys.append(col)
                df = df.sort_values(by=tmp_keys, ascending=ascending)
                df = df.drop(columns=[k for k in tmp_keys if k.startswith("__sort__")])

    if columns:
        col_list = [col.strip() for col in columns.split(",") if col.strip()]
        valid_cols = [col for col in col_list if col in df.columns]
        missing_cols = [col for col in col_list if col not in df.columns]

        if missing_cols:
            print(f"Warning: Columns not found: {', '.join(missing_cols)}", file=sys.stderr)

        if valid_cols:
            df = df[valid_cols]

    df_display = df.fillna("-")

    print("\n" + "#" * 78)
    print("# Summary")
    print("#" * 78)

    table = tabulate(
        df_display.values.tolist(),
        headers=df_display.columns.tolist(),
        tablefmt="simple",
        floatfmt=".4f"
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
    parser.add_argument("--budget", type=float, default=None,
                        help="Per-combo wall-time budget in seconds. When set, each "
                             "combo is executed repeatedly until the predicted next "
                             "run would exceed the budget by more than 0.5 * avg, or "
                             "the elapsed wall-time meets/exceeds the budget. The "
                             "running average is updated after every run. Without "
                             "this flag, each combo is executed exactly once.")
    parser.add_argument("--output-dir", default=None, metavar="DIR",
                        help="Directory where per-combo CSV reports are written, "
                             "one CSV per combo with every individual run kept as "
                             "string values (no numeric coercion). Created if missing.")
    parser.add_argument("--stop-on-error", action="store_true",
                        help="Abort the sweep on the first non-zero return code.")
    parser.add_argument("--sort-by", default=None,
                        metavar="COL[,COL:desc,...]",
                        help="Sort summary by specified columns. Columns are sorted in "
                             "the order specified. Append ':desc' for descending order.")
    parser.add_argument("--columns", default=None,
                        metavar="COL[,COL,...]",
                        help="Display only specified columns in the given order. "
                             "If not specified, all columns are shown.")
    args = parser.parse_args()

    work_dir = Path(os.getenv("BUILD_WORKING_DIRECTORY", "./"))
    def _resolve(p: str) -> Path:
        path = Path(p)
        if not path.is_absolute():
            path = (work_dir / path).resolve()
        return path

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

    # Pre-build //demo:demo once so per-run timings reflect execution only
    # and we don't pay a rebuild cost on the first combo.
    build_argv = ["bazel", "build",
                  "--ui_event_filters=-info,-stdout,-stderr",
                  "--noshow_progress", "//demo:demo"]
    print(f"Pre-building: {' '.join(build_argv)}")
    build_start = time.monotonic()
    build_proc = subprocess.run(build_argv, cwd=workspace_root)
    build_elapsed = time.monotonic() - build_start
    if build_proc.returncode != 0:
        print(f"error: pre-build failed (rc={build_proc.returncode})", file=sys.stderr)
        return build_proc.returncode
    print(f"Pre-build done in {build_elapsed:.2f}s")

    # Prepare output directory upfront if requested.
    output_dir = None
    if args.output_dir:
        output_dir =_resolve(args.output_dir)
        os.makedirs(output_dir, exist_ok=True)
        print(f"CSV output dir: {output_dir}")

    # Param/metric key sets are stable across combos (Cartesian product keys).
    param_keys: list[str] = [k.lstrip("-") for k in cli_grid.keys()]
    metric_keys: list[str] = list(_METRIC_PATTERNS.keys())

    per_combo_rows: list[list[dict[str, Any]]] = []
    total_runs = 0
    total_failures = 0
    aborted = False

    for i, combo in enumerate(runnable, 1):
        raw_results, rows, failures = _run_combo_budgeted(
            combo=combo,
            fixed_args=args.fixed,
            workspace_root=workspace_root,
            timeout=args.timeout,
            budget=args.budget,
            combo_idx=i,
            combo_total=len(runnable),
            stop_on_error=args.stop_on_error,
            param_keys=param_keys,
            metric_keys=metric_keys,
        )
        per_combo_rows.append(rows)
        total_runs += len(rows)
        total_failures += failures

        if output_dir and rows:
            path = _write_combo_csv(output_dir, combo, rows)
            if path:
                print(f"  wrote {path} ({len(rows)} run(s))")

        if args.stop_on_error and failures > 0:
            print("Stopping early due to --stop-on-error.")
            aborted = True
            break

    summary_df = _build_summary_dataframe(per_combo_rows, param_keys)
    _print_summary(summary_df, sort_by=args.sort_by, columns=args.columns)

    completed_combos = len(per_combo_rows)
    print(f"\nDone: {completed_combos} combo(s), {total_runs} total run(s), "
          f"{total_failures} failure(s).")
    if aborted:
        return 1
    return 0 if total_failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
