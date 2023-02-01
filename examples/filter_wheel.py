#!/usr/bin/env python3
"""Example communicating with a filter wheel."""

from tigerasi.tiger_controller import TigerController


class FilterWheel:
    """Filter Wheel abstraction operating through an ASI Tiger Controller."""

    def __init__(self, tigerbox: TigerController, tiger_axis: int = 1):
        """Constructor.

        :param tigerbox: tigerbox hardware.
        :param tiger_axis: which axis the wheel shows up as according to the
            tigerbox.
        """
        self.tigerbox = tigerbox
        self.tiger_axis = tiger_axis
        self.log = logging.getLogger(__name__ + "." + self.__class__.__name__)

    def get_index(self):
        """return all axes positions as a dict keyed by axis."""
        return self.tigerbox.get_position(str(self.tiger_axis))

    def set_index(self, index: int, wait=True):
        """Set the filterwheel index."""
        cmd_str = f"MP {index}\r\n"
        self.log.debug(f"FW{self.tiger_axis} move to index: {index}.")
        # Note: the filter wheel has a different reply line termination.
        self.tigerbox.send("FW 1\r\n", read_until=f"\n\r{self.tiger_axis}>")
        self.tigerbox.send(cmd_str, read_until=f"\n\r{self.tiger_axis}>")
        # TODO: add a "busy" check because tigerbox.is_moving() doesn't apply
        #   to filter wheels.


if __name__ == "__main__":
    box = TigerController()
    filter_wheel_axes = box.ordered_filter_wheels
    filter_wheel = FilterWheel(box, axes=filter_wheel_axes[0])

    filter_wheel.set_index(0)
    filter_wheel.set_index(1)

