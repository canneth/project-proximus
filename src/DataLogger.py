
import csv
import re
from collections import defaultdict
from pathlib import Path

class DataLogger:
    def __init__(
        self,
        data_fields = [],
        save_file_path = Path(".") / "data" / "test_data.csv"
    ):
        self.save_file_path = save_file_path
        self.data_fields = data_fields

        with open(self.save_file_path, 'w') as f:
            header_line = re.sub("\[|\'|\]", "", "{}".format(self.data_fields)) + "\n"
            f.write(header_line)

    def writeData(self, dict_of_data):
        with open(self.save_file_path, 'a') as f:
            line = ""
            for key in list(dict_of_data.keys()):
                line = line + re.sub("\[|\'|\]", "", "{}".format(dict_of_data[key])) + ", "
            line = line.rstrip(", ") + "\n"
            f.write(line)

if __name__ == "__main__":
    data_logger = DataLogger(data_fields = ["x", "y", "z"])
    data_logger.writeData({"x": 1, "y": 2, "z": 3})
    data_logger.writeData({"x": 1, "y": 2, "z": 3})
    data_logger.writeData({"x": 1, "y": 2, "z": 3})
    data_logger.writeData({"x": 1, "y": 2, "z": 3})