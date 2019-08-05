#!/bin/bash

# Set up proper error handling and disallow unset arguments
set -o errexit -o errtrace -o pipefail -o nounset

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Add the hover-jet repository's "scripts" directory to path for new shells
echo "export PATH=\$PATH:"$DIR >> ~/.bashrc

# Source the modified "~/.bashrc" file in this shell
source ~/.bashrc
