"""Download one Hugging Face repo snapshot into a plain directory (no cache symlinks).

Usage: python -I hf_snapshot.py REPO_ID REVISION LOCAL_DIR [REPO_TYPE]

Honours HF_ENDPOINT (a mirror of https://huggingface.co). Resumable: re-running skips
files that are already complete. Mirrors may answer 429 (Too Many Requests)
when hit with many parallel requests, so the download uses few workers
(HF_SNAPSHOT_WORKERS, default 2) and retries with back-off
(HF_SNAPSHOT_ATTEMPTS, default 8).
"""

import os
import sys
import time

from huggingface_hub import snapshot_download


def main() -> None:
    repo_id, revision, local_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    repo_type = sys.argv[4] if len(sys.argv) > 4 else "dataset"
    workers = int(os.environ.get("HF_SNAPSHOT_WORKERS", "2"))
    attempts = int(os.environ.get("HF_SNAPSHOT_ATTEMPTS", "8"))
    for attempt in range(1, attempts + 1):
        try:
            path = snapshot_download(
                repo_id=repo_id,
                repo_type=repo_type,
                revision=revision,
                local_dir=local_dir,
                max_workers=workers,
            )
        except Exception as exc:  # noqa: BLE001 - 429s / transient network errors
            if attempt == attempts:
                raise
            wait = min(30 * attempt, 180)
            print(f"attempt {attempt}/{attempts} failed ({type(exc).__name__}: "
                  f"{str(exc)[:200]}); retrying in {wait}s", flush=True)
            time.sleep(wait)
            continue
        print(f"snapshot ready: {path}", flush=True)
        return


if __name__ == "__main__":
    main()
