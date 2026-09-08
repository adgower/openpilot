#!/usr/bin/env bash
# Transfer only. Does not prepare, build, launch or select a boot checkout.
set -euo pipefail
[[ $# == 2 ]] || { echo 'Usage: transfer.sh PACKAGE_DIRECTORY SSH_HOST'; exit 2; }
pkg=$(cd "$1" && pwd)
host=$2
[[ "$host" =~ ^[A-Za-z0-9_.@-]+$ ]] || { echo 'Invalid SSH host'; exit 2; }
[[ -f "$pkg/package.json" ]] || { echo 'Missing package manifest'; exit 2; }
remote="/data/navigator-staging-inbox-$(date +%Y%m%d-%H%M%S)"
ssh -o IdentitiesOnly=yes -i "$HOME/.ssh/id_rsa" "$host" "umask 077; mkdir '$remote'"
scp -o IdentitiesOnly=yes -i "$HOME/.ssh/id_rsa" -r "$pkg/." "$host:$remote/"
printf 'Copied to %s. No device software was started.\n' "$remote"
