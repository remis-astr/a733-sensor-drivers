#!/usr/bin/env bash
# NPU smoke test on the board (called by 30-checks, or alone; no root needed).
# Runs prebuilt NBG models for the A733 NPU (VIP9000 family, NPU_VERSION v3,
# VIPLite 2.0 = npu-runtime 2.0.x) from github.com/ZIFENG278/ai-sdk with
# vpm_run, 20 loops each, and prints vpm_run's timing lines.
#   npu-test.sh [--resnet50]   (resnet50 adds a 17 MB download)
set -uo pipefail

BASE=https://raw.githubusercontent.com/ZIFENG278/ai-sdk/HEAD/examples
DIR=/var/tmp/npu-test
mkdir -p "$DIR"

fetch() {	# url dest
	[[ -s "$2" ]] || curl -fsSL -o "$2" "$1" || { echo "download failed: $1" >&2; return 1; }
}

run() {		# name nb input
	local d="$DIR/$1"
	mkdir -p "$d"
	fetch "$2" "$d/model.nb" && fetch "$3" "$d/input.dat" || return 1
	printf '[network]\n./model.nb\n[input]\n./input.dat\n' > "$d/sample.txt"
	echo "--- $1"
	( cd "$d" && vpm_run -s sample.txt -l 20 -b 1 2>&1 ) \
		| grep -iE 'time|fps|cost|error|fail|init|version' | head -12
}

command -v vpm_run >/dev/null || { echo "vpm_run not installed (npu-runtime)"; exit 1; }
ls -l /dev/vipcore 2>/dev/null || echo "WARNING: /dev/vipcore missing"

run operator "$BASE/vpm_run/operator/v3/network_binary.nb" "$BASE/vpm_run/operator/input_0.dat"
run lenet "$BASE/lenet/model/v3/lenet.nb" "$BASE/lenet/input_data/lenet.dat"
if [[ "${1:-}" == --resnet50 ]]; then
	# Input: the operator example's 224x224x3 uint8 tensor (150528 bytes) fits resnet50's input shape.
	run resnet50 "$BASE/resnet50/model/v3/resnet50.nb" "$BASE/vpm_run/operator/input_0.dat"
fi
