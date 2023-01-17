# TigerASI
a lightweight python interface for ASI brand Tiger Controllers

## Installation

To install this package from the github repository in editable mode, from this directory invoke `pip install -e .`

## Intro and Basic Usage

````python
from tigerasi.tiger_controller import TigerController

box = TigerController("COM4")
````

The basic command syntax looks like this:
````python
box.zero_in_place('x', 'y')  # Zero out the specified axes at their current location.
box.move_axes_absolute(x=1000, y=25)  # Move to an absolute location in "stage units" (tenths of microns).
box.move_axes_relative(z=100) # Move z +100 stage units in the positive z direction.
````

### Syntax Basics
All commands that reference stage axes accept a variable, optional number of arguments.
````python
box.zero_in_place('x')  # only zeros the x axis. Other axes are ignored.
````
Stage axes are also case-insensitive,
````python
box.zero_in_place('X', 'y', 'Z')  # also ok
````
and the order doesn't matter.
````python
box.zero_in_place('y', 'z', 'x')  # also ok 
````

All commands that query stage axes return a dict, keyed by *upper-case* stage axis.
````python
box.get_position('x', 'z', 'y')
# {'X': 100.0, 'Y': 305.0, 'Z': 10000.0}
````

Some commands can take an axis setting to be "current value" and another axis setting to be a specified value.
The syntax for these commands look like this:
````python
box.set_home('x', 'z', y=100.0) # Set x and z axes homing location to current spot. Set y axis to specific spot.
box.set_home('z', 'y', 'x', m=100.0, n=200.0) # variable number of arguments ok! order and case don't matter.
````

## Implementation Details

### Blocking or Non-Blocking?
All commands to the Tigerbox return a reply.
Commands that query the Tigerbox state will also return data with that reply.

Waiting for a reply introduces 10-20[ms] of execution time before the function returns.
By default, methods *will block* until receiving this reply unless otherwise specified, like this:
````
box.move_axes_absolute(x=1000, y=25, wait_for_output=False, wait_for_reply=False) # will not block.
````
This behavior can only be used for commands to change the Tigerbox state.
Commands that query the Tigerbox state will always block until they receive a hardware reply.


