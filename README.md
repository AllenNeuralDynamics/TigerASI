# TigerASI
a feature-rich Python interface for ASI Tiger Controllers.

This driver was written to simplify the serial api to ASI's [Tiger Controllers](https://www.asiimaging.com/controllers/tiger-controller/) while reducing reliance on the full [documentation](https://asiimaging.com/docs/products/serial_commands) for most users.
Many (but not all!) commands have been exposed and wrapped in a simplified, self-consistent interface and documented for easy usage.

## Installation
To install this package from [PyPI](https://pypi.org/project/TigerASI/0.0.2/), invoke: `pip install TigerASI`.

To install this package from the Github in editable mode, from this directory invoke: `pip install -e .`

To install this package in editable mode and build the docs locally, invoke: `pip install -e .[dev]`

## Intro and Basic Usage

````python
from tigerasi.tiger_controller import TigerController

box = TigerController("COM4")
````

The basic command syntax looks like this:
````python
box.zero_in_place('x', 'y')  # Zero out the specified axes at their current location.
box.move_absolute(x=1000, y=25)  # Move to an absolute location in "stage units" (tenths of microns).
box.move_relative(z=100) # Move z +100 stage units in the positive z direction.
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

Some commands assume *all* axes if none are specified.
````python
box.zero_in_place()  # will zero ALL lettered axes.
box.reset_lower_travel_limits()  # will reset ALL lettered axes.

box.get_home()  # will get ALL lettered axis home positions.
box.get_lower_travel_limits() # will get ALL lettered axis lower travel limits.
````

For setting values, this might not be your desired behavior, so it is safer to default to passing in axes explicitly.
````python
box.zero_in_place('x', 'y', 'z')  # will zero only x, y, and z axes.
box.reset_lower_travel_limits('x', 'y', 'z')  # will reset only x, y, and z axes.
````
When in doubt, check the docs.

## Simulation
This package also features a simulated version of the TigerController
````python
from tigerasi.sim_tiger_controller import SimTigerController

box = SimTigerController()  # OR
box = SimTigerController('COM4')  # com port is ignored. # OR
box = SimTigerController(build_config={'Motor Axes': ['X', 'Y', 'Z']})

# This object tracks its internal state for position and speed.
box.home_in_place('x', 'y', 'z')  # home mocked axes.
box.move_absolute(z=10)  # move mocked axis.
````
This feature can be useful for testing higher level code using the current api without the need to interact with real hardware.

## Advanced Usage
Many (but not all!) of ASI's more advanced features have been made available via this simplified API.
This list includes joystick enabling/disabling and remapping, setting stage travel limits, queuing moves into the hardware buffer, and many other more nuanced features.
For a breakdown of what commands have been exposed, have a look at the [examples folder](https://github.com/AllenNeuralDynamics/TigerASI/tree/main/examples) and the docs.

## Documentation
Docs can be generated via Sphinx but are also available on [readthedocs](https://tigerasi.readthedocs.io/en/latest/).

## Implementation Details

### Blocking or Non-Blocking?
All commands to the Tigerbox return a reply.
Commands that query the Tigerbox state will also return data with that reply.

Waiting for a reply introduces 10-20[ms] of execution time before the function returns an 'ACK'knowledgement.
By default, methods *will block* until receiving this acknowledgement unless otherwise specified, like this:
````python
box.move_absolute(x=1000, y=25, wait=False) # will not block.
````
This behavior can only be used for commands to change the Tigerbox state.
Commands that query the Tigerbox state will always block until they receive a hardware reply.


