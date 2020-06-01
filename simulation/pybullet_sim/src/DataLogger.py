
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
        self.data_dict = {key: 0 for key in self.data_fields}

        with open(self.save_file_path, 'w') as f:
            header_line = re.sub("\[|\'|\]", "", "{}".format(self.data_fields)) + "\n"
            f.write(header_line)

    def writeData(self):
        with open(self.save_file_path, 'a') as f:
            line = ""
            for key in list(self.data_dict.keys()):
                if key == "t":
                    line = line + re.sub("\[|\'|\]", "", "{:.3f}".format(self.data_dict[key])) + ","
                elif (type(self.data_dict[key]) == float):
                    line = line + re.sub("\[|\'|\]", "", "{:.6f}".format(self.data_dict[key])) + ","
                else:
                    line = line + re.sub("\[|\'|\]", "", "{}".format(self.data_dict[key])) + ","
            line = line.rstrip(", ") + "\n"
            f.write(line)
