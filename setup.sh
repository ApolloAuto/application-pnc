#!/bin/bash
cd `dirname $0`

arch=`uname -m`
if [[ ${arch} = "x86_64" ]]; then
    ln -snf .env.x86_64 .env
    ln -snf .workspace.json.x86_64 .workspace.json
    ln -snf .buildtool.conf.x86_64 .buildtool.conf
elif [[ ${arch} = "aarch64" ]]; then
    ln -snf .env.aarch64 .env
    ln -snf .workspace.json.aarch64 .workspace.json
    ln -snf .buildtool.conf.aarch64 .buildtool.conf
else
    echo "unsupported arch: ${arch}"
    exit 1
fi
