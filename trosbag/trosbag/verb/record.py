"""tros record - wrapper around ros2bag record with custom defaults."""
from ros2bag.verb.record import RecordVerb as _RecordVerb
from trosbag.verb import VerbExtension


class RecordVerb(VerbExtension):
    """Record ROS data to a bag (TROS version)."""

    _impl = _RecordVerb()

    def add_arguments(self, parser, cli_name):
        self._impl.add_arguments(parser, cli_name)

    def main(self, *, args):
        return self._impl.main(args=args)
