#!/bin/bash

rosservice call /set_dimmer_values "lamp_values:
- 100
- 100
- 100
- 100
- 100
- 100
"