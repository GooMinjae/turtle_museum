# turtlebot4_tutorials

Tutorial source code for the TurtleBot 4.

Tutorials are available in the [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/).

## Installing OpenAI Pip packages on Ubuntu 24.04

On Ubuntu 24.04, `rosdep` cannot globally install Pip packages by default.  If you encounter errors installing dependencies, you may need to add
```
# Allow rosdep to install system-wide pip packages
[install]
break-system-packages = true
```
to `/etc/pip.conf` or
```
export PIP_BREAK_SYSTEM_PACKAGES=1
```
to `$HOME/.bashrc`.

Alternatively, you can set up a `venv` and install Pip packages manually without using `rosdep`.