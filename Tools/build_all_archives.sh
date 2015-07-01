#!/bin/sh

make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v45
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v45P
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51P
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51Pro
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51ProP
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52P
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52Pro
make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52ProP
make archives MAXOPTIMIZATION=-Os BOARDS=vrubrain-v51
make archives MAXOPTIMIZATION=-Os BOARDS=vrubrain-v51P
make archives MAXOPTIMIZATION=-Os BOARDS=vrubrain-v52

make distclean