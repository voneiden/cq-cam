#!/bin/sh
(cd doc && sphinx-build -E -a -b html . ../docs)
