#!/usr/bin/env bash
# Sync this repository with the Raspberry Pi 5 next to the robot.
#
#   tools/sync-pi.sh push [BRANCH]   # laptop -> Pi hub (default: current branch)
#   tools/sync-pi.sh pull            # Pi hub -> laptop (fetch only; merge yourself)
#
# The hub is a bare repository on the Pi (raspberrypi5:git/mycobot-280-lab.git), remote "pi". The
# Pi's working copy (~/myCobot/mycobot-280-lab) clones it and pushes and pulls it like any remote.
# A plain SSH remote has no Git LFS server, so this script copies the LFS objects with rsync
# and skips the LFS pre-push hook (--no-verify). It never pushes to GitHub.
set -euo pipefail

HOST=raspberrypi5
HUB=git/mycobot-280-lab.git
cd "$(git rev-parse --show-toplevel)"
LFS="$(git rev-parse --git-common-dir)/lfs/objects"   # also right in a git worktree

case "${1:-}" in
  push)
    branch="${2:-$(git branch --show-current)}"
    ssh "$HOST" "mkdir -p ~/$HUB/lfs/objects"
    rsync -a "$LFS/" "$HOST:$HUB/lfs/objects/"
    git push --no-verify pi "$branch"
    ;;
  pull)
    git fetch pi
    mkdir -p "$LFS"
    rsync -a "$HOST:$HUB/lfs/objects/" "$LFS/"
    echo "Fetched. Merge with: git merge pi/<branch>"
    ;;
  *)
    sed -n '2,10p' "$0"
    exit 1
    ;;
esac
