#!/usr/bin/env python3
import argparse
import os
from pathlib import Path

from huggingface_hub import snapshot_download


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cache", default="models")
    args = p.parse_args()
    cache = str(Path(args.cache).resolve())
    os.environ["HF_HOME"] = cache
    for repo in ("IDEA-Research/grounding-dino-tiny", "facebook/sam2.1-hiera-small"):
        print(f"Downloading {repo} into {cache}")
        snapshot_download(repo_id=repo, cache_dir=cache)
    print("Done. Keep model.local_files_only=true for inference without network access.")


if __name__ == "__main__":
    main()
