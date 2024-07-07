#!/usr/bin/env fish

# Print the word "Querying" in bold green like the rust compiler
echo -en "\033[1;32m    Querying\033[0m "
# We run st-info --probe to check connection to the board
sudo st-info --probe | head -n 1

sudo probe-rs run --chip stm32f401RETx $argv
