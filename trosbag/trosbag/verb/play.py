"""tros play - wrapper around ros2bag play with custom defaults."""
from ros2bag.verb.play import PlayVerb as _PlayVerb
from trosbag.verb import VerbExtension


class PlayVerb(VerbExtension):
    """Play back ROS data from a bag (TROS version)."""

    _impl = _PlayVerb()

    def add_arguments(self, parser, cli_name):
        self._impl.add_arguments(parser, cli_name)

    def main(self, *, args):
        return self._impl.main(args=args)
