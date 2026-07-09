#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Download a LeRobot dataset from Hugging Face Hub.

This script now follows the exact same download path/logic as:
`lerobot-record --resume=true`

Examples:
    # Same behavior as lerobot-record --dataset.repo_id=... --resume=true
    python -m lerobot.scripts.download_hf_dataset \
        --repo-id Xense/assemble_box_with_phone_stand_test

    # Explicit root (same semantics as --dataset.root in lerobot-record)
    python -m lerobot.scripts.download_hf_dataset \
        --repo-id Xense/assemble_box_with_phone_stand_test \
        --root /home/xense/.cache/huggingface/lerobot/Xense/assemble_box_with_phone_stand_test

    # Download metadata + parquet only (skip videos)
    python -m lerobot.scripts.download_hf_dataset \
        --repo-id Xense/assemble_box_with_phone_stand_test \
        --no-videos
"""

import argparse
import logging
from pathlib import Path


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(asctime)s %(filename)s:%(lineno)d %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Download a LeRobot dataset from Hugging Face using resume-compatible logic."
    )
    parser.add_argument(
        "--repo-id",
        "--dataset-id",
        dest="repo_id",
        type=str,
        required=True,
        help="Dataset repository ID, e.g. 'Xense/assemble_box_with_phone_stand_test'.",
    )
    parser.add_argument(
        "--root",
        type=str,
        default=None,
        help=(
            "Dataset root path, same meaning as --dataset.root in lerobot-record. "
            "If omitted, defaults to HF cache path '~/.cache/huggingface/lerobot/<repo_id>'."
        ),
    )
    parser.add_argument(
        "--revision",
        type=str,
        default=None,
        help="Optional branch/tag/commit.",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        nargs="+",
        default=None,
        help="Optional episode indices to download (space-separated). If omitted, download all episodes.",
    )
    parser.add_argument(
        "--no-videos",
        action="store_true",
        help="Skip downloading video files.",
    )
    parser.add_argument(
        "--force-cache-sync",
        action="store_true",
        help="Force sync local cache with remote before loading.",
    )
    return parser.parse_args()


def resolve_target_path(repo_id: str, root: str | None) -> Path:
    from lerobot.utils.constants import HF_LEROBOT_HOME

    if root:
        return Path(root).expanduser().resolve()
    return (HF_LEROBOT_HOME / repo_id).expanduser().resolve()


def main() -> None:
    setup_logging()
    args = parse_args()

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ModuleNotFoundError:
        # Allow running as: python src/lerobot/scripts/download_hf_dataset.py ...
        import sys

        src_root = Path(__file__).resolve().parents[2]
        if str(src_root) not in sys.path:
            sys.path.insert(0, str(src_root))
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

    target_path = resolve_target_path(args.repo_id, args.root)
    target_path.mkdir(parents=True, exist_ok=True)

    logging.info("Downloading with resume-compatible LeRobotDataset logic...")
    logging.info("repo_id=%s revision=%s episodes=%s", args.repo_id, args.revision, args.episodes)
    logging.info("download_videos=%s", not args.no_videos)
    logging.info("target_path=%s", target_path)

    dataset = LeRobotDataset(
        repo_id=args.repo_id,
        root=target_path,
        revision=args.revision,
        episodes=args.episodes,
        force_cache_sync=args.force_cache_sync,
        download_videos=not args.no_videos,
    )

    logging.info("Download completed")
    logging.info("num_episodes=%s num_frames=%s fps=%s", dataset.num_episodes, dataset.num_frames, dataset.fps)
    logging.info("Local path: %s", dataset.root)


if __name__ == "__main__":
    main()
