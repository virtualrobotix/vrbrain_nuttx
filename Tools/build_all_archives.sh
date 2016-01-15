#!/bin/sh

make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v45
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v45P
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51P
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51Pro
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v51ProP
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52P
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52Pro
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrbrain-v52ProP
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrubrain-v51
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrubrain-v51P
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrubrain-v52
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrcore-v10
make -f Makefile.make archives MAXOPTIMIZATION=-Os BOARDS=vrcore-v10P

make -f Makefile.make distclean