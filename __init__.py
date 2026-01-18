"""
py-ols - WinOLS File Reader

Parse WinOLS .ols project files to extract parameter definitions,
CAL offsets, and embedded binary data.
"""

from .ols_reader import (
    OLSReader,
    OLSFile,
    Parameter,
    AxisInfo,
    BinaryVersion,
    CALBlock,
    read_ols,
)

__version__ = "1.1.0"
__all__ = [
    "OLSReader",
    "OLSFile",
    "Parameter",
    "AxisInfo",
    "BinaryVersion",
    "CALBlock",
    "read_ols",
]
