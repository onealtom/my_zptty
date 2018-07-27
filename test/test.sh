#!/bin/bash

cat < /dev/zptty0

sleep 1

exec "ls /dev/nodebuff"

