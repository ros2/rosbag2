#!/usr/bin/env python3
""" Runs a parametric sweep of runs measuring the throughput and memory usage of
rosbag2_storage::BaseWriteInterface implementations. Writes a CSV summary of each run to
stdout, or to a file if a path is provided.
"""

from pathlib import Path
import subprocess
import shutil
from tempfile import mkdtemp
import csv
import sys
from io import StringIO

import yaml

"""CONFIG_DIMENSIONS contains the parameters to sweep across and their values.
CONFIG_DIMENSIONS is structured as `{ parameter label : { variant label : config fragment } }`.
the function `build_configs()` iterates through the variants of each parameter and produces
a config for each combination."""
CONFIG_DIMENSIONS = {
    # The size and distribution of messages to write.
    "messages": {
        "large": { "topics": [{"name": "/large", "message_size": 1_000_000}]},
        "medium": { "topics": [{"name": "/medium", "message_size": 10_000}]},
        "small": { "topics": [{"name": "/small", "message_size": 100}]},
        "mixed": { "topics": [
            {"name": "/small", "message_size": 100, "write_proportion": 0.1},
            {"name": "/medium", "message_size": 10_000, "write_proportion": 0.2},
            {"name": "/large", "message_size": 1_000_000, "write_proportion": 0.7},
        ]},
    },
    # The size of batches to write in each write() call.
    "batch_size": {
        "small": { "min_batch_size_bytes": 1000 },
        "default": { "min_batch_size_bytes": 10000000 },
    },
    # Configuration parameters for the writer plugin to use.
    "plugin_config": {
        "mcap_default": {"storage_id": "mcap"},
        "mcap_nocrc": {
            "storage_id": "mcap",
            "storage_options": {
                "noCRC": True,
            }
        },
        "mcap_zstdfast": {
            "storage_id": "mcap",
            "storage_options": {
                "chunkSize": 10_000_000,
                "compression": "Zstd",
                "compressionLevel": "Fastest",
            }
        },
        "mcap_uncompressed": {
            "storage_id": "mcap",
            "storage_options": {
                "compression": "None",
            }
        },
        "mcap_nochunking": {
            "storage_id": "mcap",
            "storage_options": {
                "noCRC": True,
                "noChunking": True,
            }
        },
        "sqlite_default": {"storage_id": "sqlite3"},
        "sqlite_resilient": {
            "storage_id": "sqlite3",
            "storage_options": {
                "write": {"pragmas": [
                    "journal_mode=WAL",
                    "synchronous=NORMAL",
                ]},
            }
        },
    }
}


def should_ignore(name_dict):
    return (
        name_dict.get("plugin_config", "").startswith("mcap_") and
        name_dict.get("batch_size", "") in ["medium", "large"]
    )


def executable_path():
    """Returns the path to the benchmark binary that actually writes the bag file.
    This is separated out into a separate process so that all resources are cleaned up
    between runs.
    """
    return Path(__file__).resolve().parent / "single_benchmark"

def build_configs():
    """ Iterates through each combination of variants in CONFIG_DIMENSIONS and produces a list of
    ({parameter label : variant label}, config) tuples.
    """
    configs = [({}, {})]
    for dimension_name, dimension in CONFIG_DIMENSIONS.items():
        new_configs = []
        for (existing_name, existing_config) in configs:
            for variant_name, variant_fragment in dimension.items():
                label = f"{dimension_name}_{variant_name}"
                new_name = label if existing_name == "" else f"{existing_name}-{label}"
                new_name = dict(**existing_name)
                new_name[dimension_name] = variant_name
                if should_ignore(new_name):
                    print(f"ignoring configuration {new_name}", file=sys.stderr)
                    continue
                new_config = dict(**existing_config)
                new_config.update(variant_fragment)
                new_configs.append((new_name, new_config))
        configs = new_configs
    return configs

def run_once(config):
    """ Runs `single_benchmark` with the given config and returns the resulting CSV content. """
    outdir = mkdtemp()
    try:
        res = subprocess.run(
            [executable_path(), yaml.dump(config), outdir],
            check=True,
            stdout=subprocess.PIPE
        )
    finally:
        shutil.rmtree(outdir)
    return res.stdout.decode("utf-8")

def make_digest(name, csv_content):
    """ Aggregates the results of one benchmark run into a row for the final digest CSV. """
    reader = csv.DictReader(StringIO(csv_content))
    write_times = []
    arena_sizes = []
    in_use_sizes = []
    byte_throughputs = []
    mmap_sizes = []
    close_time = None
    for row in reader:
        if row["close_ns"]:
            close_time = float(row["close_ns"]) / 1e9
        else:
            write_times.append(float(row["write_ns"]) / 1e9)
            arena_sizes.append(int(row["arena_bytes"]))
            byte_throughputs.append(float(row["num_bytes"]) / (float(row["write_ns"]) / 1e9))
            in_use_sizes.append(int(row["in_use_bytes"]))
            mmap_sizes.append(int(row["mmap_bytes"]))
    res = dict(**name)
    res.update({
        "name": ";".join([f"{k}={v}" for k, v in name.items()]),
        "avg_byte_throughput": sum(byte_throughputs) / len(byte_throughputs),
        "max_arena_size": max(arena_sizes),
        "max_in_use_size": max(in_use_sizes),
        "max_mmap_size": max(mmap_sizes),
        "close_time": close_time,
    })
    return res


def write_csv_from_dicts(outfile, rows):
    """ takes a list of {column name: value} dicts and writes a CSV to a file-like object `outfile`. """
    writer = csv.DictWriter(
        outfile,
        list(rows[0].keys()),
    )
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def main():
    configs = build_configs()
    digest_rows = []
    for name, config in configs:
        print(f"Running benchmark: {name}...", file=sys.stderr)
        result = run_once(config)
        digest_rows.append(make_digest(name, result))
    if len(sys.argv) > 1:
        with open(sys.argv[1], "w") as f:
            write_csv_from_dicts(f, digest_rows)
    else:
        s = StringIO()
        write_csv_from_dicts(s, digest_rows)
        print(s.getvalue())


if __name__ == "__main__":
    main()
