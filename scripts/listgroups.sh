#!/bin/bash

getent group | awk -F: '{printf "Group %s with GID=%d\n", $1, $3}'
