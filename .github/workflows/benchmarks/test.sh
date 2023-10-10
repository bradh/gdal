#!/bin/bash

set -eu

BENCHMARK_STORAGE="file:///tmp"

# Dry run to hopefully stabilize later timings
(cd old_version/gdal/build; source ../scripts/setdevenv.sh; pytest autotest/benchmark --capture=no -ra -vv)

# Run reference (old) build and save its results
(cd old_version/gdal/build; source ../scripts/setdevenv.sh; pytest autotest/benchmark --benchmark-save=ref "--benchmark-storage=${BENCHMARK_STORAGE}" --capture=no -ra -vv)

# Run target build and compare its results to the reference one.
# Fail if we get results 5% slower or more.
# Retry if that fails a first time.
(source ${GDAL_SOURCE_DIR:=..}/scripts/setdevenv.sh; pytest autotest/benchmark --benchmark-compare-fail="min:5%" --benchmark-compare=0001_ref "--benchmark-storage=${BENCHMARK_STORAGE}" --capture=no -ra -vv || (echo "Retrying..."; pytest autotest/benchmark --benchmark-compare-fail="min:5%" --benchmark-compare=0001_ref "--benchmark-storage=${BENCHMARK_STORAGE}" --capture=no -ra -vv))
