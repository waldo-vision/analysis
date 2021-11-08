from abc import ABC, abstractmethod


class ReplayParser(ABC):
    """This class is used to parse the replay file of a FPS game.
    """
    def __init__(self) -> None:
        self.filename = ""
        self.parser = None
        self.parsed_data = None

    @abstractmethod
    def parse(self, file_path: str) -> None:
        """This method is used to parse the replay file of a FPS game.
        
        Args:
            file_path (str): The path to the replay file.
        """
        raise NotImplementedError()

    def get_parsed_data(self) -> dict:
        """This method is used to get the parsed data
        """
        return self.parsed_data
