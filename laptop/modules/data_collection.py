"""
    Module to hand data collection
    Author: Neo Vorsatz
    Last updated: 20 October 2026
"""

# IMPORTS ================================

# External

# Internal

# TYPES ================================


# CONSTANTS ================================

_OUT_FILE = "data/data.csv"

# VARIABLES ================================

_printed_head:bool = False # whether or not the heading has been written
_file = open(_OUT_FILE, "w") # the file
_headings:list[str] = [] # all headings
_row:dict[str,float] = {} # all data associated with the data

# HELPER FUNCTIONS ================================


# API FUNCTIONS ================================

def add_heading(heading:str):
    """
    Adds a heading to the CSV file, allowing a new column of data to be displayed

    Args:
        heading(str): The name of the heading
    """
    global _headings
    if not heading in _headings:
        _headings.append(heading)

def record(data:float, heading:str):
    """
    Records a piece of data to a specified heading. This overwrites past data if `flush` wasn't called

    Args:
        data(float): The data to be recorded
        heading(str): The heading to record the data to
    """
    global _row
    _row[heading] = data

def flush():
    """
    Writes all pending data to the CSV file
    """
    # Global variables
    global _printed_head, _file, _headings, _row

    # Print the heading
    if not _printed_head:
        _file.write(", ".join(_headings)+", ")
        _file.write("\n")
        _printed_head = True

    # Writes the data
    data = ""
    for heading in _headings:
        if heading in _row.keys():
            data += str(_row[heading])
        data += ", "
    _file.write(data+"\n")

    # Empty the buffer
    _row = {}

def finish():
    """
    Closes the file pointer
    """
    global _file
    _file.close()
