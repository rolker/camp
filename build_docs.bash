#!/bin/bash

rosdoc2 build --package-path . --doc-build-directory ./docs_build/build/ --cross-reference-directory ./docs_build/cross_reference/ --output-directory ./docs_build/output
