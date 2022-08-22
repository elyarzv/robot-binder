SHELL=/bin/bash

ROBOT_NAME?=condor

include binder/Makefile.env
-include binder/ais-makefile/Makefile

src-update-submodule: ## Update submodules to match their version
	git submodule sync --recursive
	git submodule update --init --recursive
	git submodule foreach "git lfs pull"
