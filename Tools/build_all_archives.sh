#!/bin/sh

make archives BOARDS=vrbrain-v45
make archives BOARDS=vrbrain-v51
make archives BOARDS=vrbrain-v51Pro
make archives BOARDS=vrbrain-v52
make archives BOARDS=vrbrain-v52Pro
make archives BOARDS=vrubrain-v51
make archives BOARDS=vrubrain-v52

make distclean