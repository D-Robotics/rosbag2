# Copyright 2026 D-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ros2 tros_bag interval - analyze header.stamp intervals of a bag."""

from trosbag import interval_analyzer
from trosbag.verb import VerbExtension


class IntervalVerb(VerbExtension):
    """Analyze header.stamp intervals of messages in a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_path', help='Path to the bag file or directory')
        parser.add_argument(
            '--storage', default='mcap',
            help="Storage identifier (default: 'mcap')")
        parser.add_argument(
            '--topics', type=str, nargs='+', default=None,
            help='Specific topics to analyze (default: all topics)')
        parser.add_argument(
            '--output-dir', default=None,
            help='Directory to save plots (default: <bag_dir>/bag_interval_plots)')
        parser.add_argument(
            '--no-plot', action='store_true',
            help='Only print summary, do not generate plots')

    def main(self, *, args):  # noqa: D102
        return interval_analyzer.run(args)
