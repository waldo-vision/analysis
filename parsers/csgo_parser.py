from csgo.parser import DemoParser

from parsers.replay_parser import ReplayParser


class CSGOParser(ReplayParser):
    """The parser for parsing CS:GO demo files.

       Args:
           parse_rate (int) : One of 128, 64, 32, 16, 8, 4, 2, or 1.
           The lower the value, the more frames are collected.
           Indicates spacing between parsed demo frames in ticks. Default is 128.
    """
    def __init__(self, parse_rate=128) -> None:
        super().__init__()
        self.parse_rate = parse_rate

    def parse(self, file_path: str) -> None:
        """Parses the demo file and stores the data in the class as a DataFrame.
        
        Args:
            file_path (str): The path to the demo file, which ends in .dem.
        """
        self.filename = file_path
        self.parser = DemoParser(demofile=self.filename,
                                 parse_rate=self.parse_rate)
        # self.parsed_data = self.parser.parse(return_type="df")
        self.parsed_data = self.parser.parse()
