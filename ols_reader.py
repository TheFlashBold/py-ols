"""
OLS File Reader - Parse WinOLS .ols project files

This module provides a clean interface for reading WinOLS .ols files,
extracting parameter definitions, addresses, and embedded binary data.

OLS files are proprietary project files used by WinOLS ECU tuning software.
They contain vehicle metadata, parameter definitions with addresses, and
optionally embedded binary data.

Supported OLS Versions:
    - v250-285: Old format (limited binary version support)
    - v300-399: Intermediate format (parent ID based versions)
    - v400-596: Standard format (parent ID based versions)
    - v597+:    Multi-version format (edit history with version slots)

Usage:
    from ols_reader import read_ols, OLSReader

    # Quick read
    ols = read_ols("file.ols")
    print(f"Parameters: {len(ols.parameters)}")

    # With binary extraction
    reader = OLSReader("file.ols")
    ols = reader.parse()
    for version in ols.binary_versions:
        data = reader.extract_binary(version)
        with open(f"{version.name}.bin", "wb") as f:
            f.write(data)
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class AxisInfo:
    """Axis definition for CURVE/MAP parameters"""
    points: int
    offset: int  # CAL offset for axis data
    unit: str = ""
    factor: float = 1.0  # Conversion factor (physical = raw * factor)


@dataclass
class Parameter:
    """ECU parameter definition extracted from OLS file"""
    name: str
    description: str = ""
    param_type: str = "VALUE"  # VALUE, CURVE, or MAP
    data_type: str = "UWORD"  # UBYTE, SBYTE, UWORD, SWORD, ULONG, SLONG
    cols: int = 1
    rows: int = 1
    data_offset: int = 0  # CAL offset for parameter data
    x_axis: Optional[AxisInfo] = None
    y_axis: Optional[AxisInfo] = None
    unit: str = ""
    factor: float = 1.0  # Conversion factor (physical = raw * factor)
    folder_id: int = 0  # Folder/category ID (numeric ID only - folder names not found in OLS files)


@dataclass
class BinaryVersion:
    """
    Binary version entry (e.g., "Original", "NOCS", "Daten")

    For standard format (v300-596):
        - parent_id indicates the parent folder (typically 1)
        - blob_offset is the direct file offset to binary data

    For multi-version format (v597+):
        - version_index is the slot index (0-based)
        - version_count is total edit history depth
        - blob_offset is calculated from: blob_base + version_index * blob_size
    """
    name: str
    description: str = ""  # Import path/creation description
    parent_id: Optional[int] = None  # None = root folder (0x003fffff marker)
    is_root: bool = False
    blob_offset: int = 0  # File offset where binary blob starts
    blob_size: int = 0  # Binary blob size in bytes
    version_index: int = 0  # Version index (0-based) for multi-version files (v597+)
    version_count: int = 0  # Total number of version slots (v597+)


@dataclass
class CALBlock:
    """Embedded calibration binary data"""
    offset: int  # File offset where CAL starts
    cas_id: str  # CAS identifier (e.g., "CASC2E55")
    epk: str  # EPK string (e.g., "SC8F830")
    data: bytes = b""


@dataclass
class OLSFile:
    """Parsed OLS file contents"""
    filename: str
    version: int = 0  # OLS format version (e.g., 250, 440, 478)
    make: str = ""
    model: str = ""
    engine: str = ""
    year: str = ""
    parameters: list[Parameter] = field(default_factory=list)
    binary_versions: list[BinaryVersion] = field(default_factory=list)
    cal_block: Optional[CALBlock] = None


class OLSReader:
    """
    Reader for WinOLS .ols project files.

    Supports OLS format versions 250 through 597+, including:
    - Parameter extraction with offsets, types, units, and conversion factors
    - Binary version extraction (Original, edit history)
    - CAL block extraction (Simos-specific)

    Usage:
        reader = OLSReader("file.ols")
        ols = reader.parse()

        # Access parameters
        for param in ols.parameters:
            print(f"{param.name}: offset={param.data_offset}")

        # Extract binary data
        for version in ols.binary_versions:
            data = reader.extract_binary(version)

    Attributes:
        filepath: Path to the OLS file
        data: Raw file bytes
        version: OLS format version (set during parsing)
    """

    # Reference: Common parameter prefixes (not used for detection anymore)
    # Detection now uses structural pattern matching which works for any naming convention
    # Simos ECU: c_, kf_, kl_, ip_, lc_, id_, trq_, ign_, lam_, map_, tab_, etc.
    # DSG/TCU: ges_, kad_, dam_, umw_, kan_, mmr_, tip_, kkr_, pag_, acc_, etc.
    # DL501: Ahd_, Ges_, Gss_, Krm_, Umw_, Khs_, etc.

    # Format version thresholds
    # < 300: Old format (dims at +62/+66, flag at +106)
    # 300-399: Intermediate format (dims at +118/+122, flag at +163, offset at +164)
    # >= 400: New format (dims at +117/+121, flag at +160, offset at +161)
    NEW_FORMAT_VERSION = 300
    NEWER_FORMAT_VERSION = 400

    def __init__(self, filepath: str | Path):
        self.filepath = Path(filepath)
        self.data = self.filepath.read_bytes()
        self.pos = 0
        self.version = 0  # Will be set during header parsing

    def parse(self) -> OLSFile:
        """Parse the OLS file and return structured data"""
        ols = OLSFile(filename=self.filepath.name)

        # Parse header for vehicle metadata
        self._parse_header(ols)
        ols.version = self.version

        # Extract binary versions (Daten, Original, NOCS, etc.)
        ols.binary_versions = self._extract_binary_versions()

        # Extract parameters with addresses
        ols.parameters = self._extract_parameters()

        # Find and extract CAL block
        ols.cal_block = self._extract_cal_block()

        return ols

    def _read_u16(self, offset: int) -> int:
        """Read unsigned 16-bit little-endian value"""
        return struct.unpack_from('<H', self.data, offset)[0]

    def _read_u32(self, offset: int) -> int:
        """Read unsigned 32-bit little-endian value"""
        return struct.unpack_from('<I', self.data, offset)[0]

    def _read_string(self, offset: int) -> tuple[str, int]:
        """Read length-prefixed string, return (string, bytes_consumed)"""
        length = self._read_u32(offset)
        if length == 0 or length > 500:
            return "", 4
        # OLS files use Windows-1252 encoding (Western European), not UTF-8
        s = self.data[offset + 4:offset + 4 + length].decode('cp1252', errors='replace')
        s = s.rstrip('\x00').strip()
        return s, 4 + length

    def _parse_header(self, ols: OLSFile) -> bool:
        """
        Parse OLS file header.

        Header structure:
        - [0-3]   Magic number (0x0b)
        - [4-15]  Signature "WinOLS File"
        - [16-17] Version (u16)
        - [18-21] Flags (u32)
        - [22+]   Length-prefixed strings: make, model, engine, year, fuel_type, power
        """
        # Check magic
        if self._read_u32(0) != 0x0b:
            return False

        # Check signature
        sig = self.data[4:16].decode('ascii', errors='replace').rstrip('\x00')
        if not sig.startswith("WinOLS File"):
            return False

        # Read version (determines structure format)
        self.version = self._read_u16(16)

        # Structure: magic(4) + sig(12) + version(2) + flags/timestamp(6) = 24 bytes
        # Then length-prefixed strings starting at 0x18
        pos = 0x18

        ols.make, consumed = self._read_string(pos)
        pos += consumed

        ols.model, consumed = self._read_string(pos)
        pos += consumed

        ols.engine, consumed = self._read_string(pos)
        pos += consumed

        ols.year, consumed = self._read_string(pos)

        return True

    def _extract_binary_versions(self) -> list[BinaryVersion]:
        """
        Extract binary version entries (Daten, Original, NOCS, Hexdump, etc.)

        Binary versions are stored in the header area with structure that varies by version:

        Old format (version < 597):
        - Root folders: 0x003fffff marker + length + name + child_count + child_list
        - Child versions: [blob_offset at pos-12] [blob_size at pos-8] [timestamp] [parent_id] + length + name

        New format (version >= 597):
        - [0:4] [index:4] [metadata_offset:4] [blob_size:4] [timestamp:4] [count:4] [name_len:4] [name]
        - Actual blob offset = blob_base + index * blob_size
        - blob_base is typically at a 4MB-aligned boundary before first blob
        """
        versions = []

        # Known version names that are valid binary versions
        VALID_VERSION_NAMES = {'Daten', 'Daten 2', 'Original', 'NOCS', 'Hexdump', 'Binary data'}

        # Start scanning after the header metadata area (skip to ~0x400)
        pos = 0x400
        scan_end = min(0x2000, len(self.data) - 28)

        while pos < scan_end:
            # Try v597+ format: [0:4] [index:4] [metadata_offset:4] [blob_size:4] [timestamp:4] [count:4] [name_len:4] [name]
            if pos + 32 <= len(self.data):
                zero_field = self._read_u32(pos)
                version_index = self._read_u32(pos + 4)
                metadata_offset = self._read_u32(pos + 8)
                blob_size = self._read_u32(pos + 12)
                # timestamp at pos + 16 (skip)
                version_count = self._read_u32(pos + 20)
                name_len = self._read_u32(pos + 24)

                # Validate v597 pattern: zero field, index < count, valid sizes
                # Key difference from old format: version_count > 1 (old format has parent_id = 1)
                # and index must be > 0 for a meaningful multi-version file
                if (zero_field == 0 and
                    version_index < 20 and
                    version_count > 1 and version_count <= 20 and  # Must have >1 versions
                    version_index < version_count and
                    0x100000 < metadata_offset < len(self.data) and
                    0x100000 <= blob_size <= 0x1000000 and  # 1MB to 16MB chunks (based on original binary size)
                    0 < name_len < 50 and
                    pos + 28 + name_len <= len(self.data)):

                    try:
                        name = self.data[pos + 28:pos + 28 + name_len].decode('cp1252')
                        if name.isprintable() and name.strip():
                            name_clean = name.strip()
                            if name_clean in VALID_VERSION_NAMES:
                                # Calculate actual blob offset
                                # Find blob base by looking for aligned boundary
                                # Blobs are typically at 4MB-aligned addresses
                                blob_base = (metadata_offset // blob_size) * blob_size
                                if blob_base < metadata_offset:
                                    blob_base += blob_size
                                # Adjust to find actual first blob
                                while blob_base > blob_size and blob_base - blob_size > metadata_offset:
                                    blob_base -= blob_size

                                actual_blob_offset = blob_base + version_index * blob_size

                                # Validate calculated offset - allow truncated last blob
                                if actual_blob_offset >= len(self.data):
                                    actual_blob_offset = metadata_offset  # Fallback only if start is past EOF

                                # Try to read description after name
                                desc = ""
                                desc_pos = pos + 28 + name_len
                                if desc_pos + 4 <= len(self.data):
                                    desc_len = self._read_u32(desc_pos)
                                    if 0 < desc_len < 500 and desc_pos + 4 + desc_len <= len(self.data):
                                        desc_bytes = self.data[desc_pos + 4:desc_pos + 4 + desc_len]
                                        desc = desc_bytes.decode('cp1252', errors='replace').strip()

                                versions.append(BinaryVersion(
                                    name=name_clean,
                                    parent_id=None,
                                    is_root=False,
                                    description=desc,
                                    blob_offset=actual_blob_offset,
                                    blob_size=blob_size,
                                    version_index=version_index,
                                    version_count=version_count,
                                ))
                                pos += 28 + name_len
                                continue
                    except (UnicodeDecodeError, ValueError):
                        pass

            val = self._read_u32(pos)

            # Check for root folder marker (0x003fffff)
            if val == 0x003fffff:
                name_len = self._read_u32(pos + 4)
                if 0 < name_len < 50:
                    try:
                        name = self.data[pos + 8:pos + 8 + name_len].decode('cp1252')
                        if name.isprintable() and name.strip():
                            name_clean = name.strip()
                            # Check if it's a known root folder name
                            # Hexdump can be both a root folder and entry type for binary storage
                            if any(name_clean.startswith(v) for v in ['Daten', 'Binary', 'Hexdump']):
                                versions.append(BinaryVersion(
                                    name=name_clean,
                                    is_root=True,
                                    parent_id=None,
                                ))
                            pos += 8 + name_len
                            continue
                    except (UnicodeDecodeError, ValueError):
                        pass

            # Check for version with parent ID (small integer 1-10) - old format
            # Must be preceded by size field and have a valid version name
            if 0 < val <= 10:
                name_len = self._read_u32(pos + 4)
                if 0 < name_len < 50 and pos + 8 + name_len <= len(self.data):
                    try:
                        name = self.data[pos + 8:pos + 8 + name_len].decode('cp1252')
                        if name.isprintable() and name.strip():
                            name_clean = name.strip()

                            # Must be a known version name (not parameter or metadata)
                            is_valid_version = name_clean in VALID_VERSION_NAMES

                            if is_valid_version:
                                # Try to read description
                                desc = ""
                                desc_pos = pos + 8 + name_len
                                if desc_pos + 4 <= len(self.data):
                                    desc_len = self._read_u32(desc_pos)
                                    if 0 < desc_len < 500 and desc_pos + 4 + desc_len <= len(self.data):
                                        desc_bytes = self.data[desc_pos + 4:desc_pos + 4 + desc_len]
                                        desc = desc_bytes.decode('cp1252', errors='replace').strip()

                                # Get blob_offset and blob_size (look back before parent_id)
                                # Structure: [blob_index] [blob_offset] [blob_size] [timestamp] [parent_id]
                                blob_offset = 0
                                blob_size = 0
                                if pos >= 16:
                                    blob_offset = self._read_u32(pos - 12)
                                    blob_size = self._read_u32(pos - 8)
                                    # Validate - offset should be reasonable file position
                                    if blob_offset > len(self.data) or blob_size > 0x10000000:
                                        blob_offset = 0
                                        blob_size = 0

                                versions.append(BinaryVersion(
                                    name=name_clean,
                                    parent_id=val,
                                    is_root=False,
                                    description=desc,
                                    blob_offset=blob_offset,
                                    blob_size=blob_size,
                                ))
                                pos += 8 + name_len
                                continue
                    except (UnicodeDecodeError, ValueError):
                        pass

            pos += 1

        return versions

    def _extract_parameters(self) -> list[Parameter]:
        """
        Extract parameter definitions from OLS file.

        OLS stores parameters with structure:
        - Description string (length-prefixed)
        - Zero padding
        - Metadata (6 x u32 values) where meta[0] = type code (2=VALUE, 3=CURVE, 4+=MAP)
        - Name string (length-prefixed)
        - Post-name metadata (conversion, display settings)

        This method finds parameters by their structural pattern, not by name prefixes,
        which allows it to work with any ECU type (Simos, DSG, etc.) regardless of
        naming conventions.
        """
        parameters = []
        found_names: set[str] = set()

        # Scan for metadata + name pattern
        # Metadata is 6 x u32 (24 bytes) followed by name_len (u32) + name
        pos = 0x100  # Skip file header
        while pos < len(self.data) - 50:
            try:
                # Read potential metadata (6 x u32)
                meta = struct.unpack_from('<6I', self.data, pos)

                # Validate metadata pattern:
                # meta[0] = type code (2=VALUE, 3=CURVE, 4+=MAP)
                # meta[1-3] should be small values (often 1 for dimensions)
                # meta[4] should be 0 or small (row/col related)
                # meta[5] is often 0
                type_code = meta[0]
                if type_code in (2, 3) or 4 <= type_code <= 20:
                    # Check that other values look reasonable
                    if all(m < 1000 for m in meta[1:5]):
                        # Check for name length at pos + 24
                        name_pos = pos + 24
                        if name_pos + 4 < len(self.data):
                            name_len = self._read_u32(name_pos)

                            if 2 < name_len < 100 and name_pos + 4 + name_len < len(self.data):
                                try:
                                    name = self.data[name_pos + 4:name_pos + 4 + name_len].decode('ascii')

                                    # Validate name: must be alphanumeric with underscores, brackets
                                    # and contain at least one underscore or be all uppercase
                                    clean_name = name.replace('_', '').replace('[', '').replace(']', '').replace('.', '')
                                    if (name.isprintable() and
                                        clean_name.isalnum() and
                                        len(name) >= 3 and
                                        ('_' in name or name.isupper() or '.' in name) and
                                        name not in found_names and
                                        not name.startswith('WinOLS') and
                                        not name.endswith('.ols')):

                                        found_names.add(name)
                                        param = self._parse_parameter_record(name_pos, name_len, name)
                                        if param:
                                            parameters.append(param)
                                        pos = name_pos + 4 + name_len
                                        continue
                                except (UnicodeDecodeError, ValueError):
                                    pass
            except struct.error:
                pass
            pos += 1

        return parameters

    def _parse_parameter_record(self, name_pos: int, name_len: int, name: str) -> Optional[Parameter]:
        """
        Parse a single parameter record.

        Structure varies by OLS version:

        Older format (version < 300):
        - Dimensions at +62/+66 as u16
        - Flag 0x8000 around +106, followed by 16-bit offsets

        Newer format (version >= 300):
        - Dimensions at +117 (cols) and +121 (rows) as single bytes
        - Flag 0x80 at +160, followed by 24-bit offsets at +161-163
        - Axis offset at +165-167
        """
        name_end = name_pos + 4 + name_len

        if name_end + 180 > len(self.data):
            return Parameter(name=name)

        # Read dimensions - position and format varies by OLS version
        if self.version < self.NEW_FORMAT_VERSION:
            # Old format (< 300): u16 at +62/+66
            cols = self._read_u16(name_end + 62)
            rows = self._read_u16(name_end + 66)
        elif self.version < self.NEWER_FORMAT_VERSION:
            # Intermediate format (300-399): single byte at +118 and +122
            cols = self.data[name_end + 118]
            rows = self.data[name_end + 122]
        else:
            # New format (>= 400): single byte at +117 and +121
            cols = self.data[name_end + 117]
            rows = self.data[name_end + 121]

        # Sanity check dimensions
        if cols > 100 or rows > 100 or cols == 0:
            cols = 1
        if rows > 100 or rows == 0:
            rows = 1

        # Determine parameter type
        if cols > 1 and rows > 1:
            param_type = "MAP"
        elif cols > 1 or rows > 1:
            param_type = "CURVE"
        else:
            param_type = "VALUE"

        # Read data type indicator and determine data_type string
        # Data type indicator: 0/1=UBYTE, 2=SBYTE, 3=UWORD, 4=SWORD, 5=ULONG, 6=SLONG
        data_type = "UWORD"  # Default
        if self.version < self.NEW_FORMAT_VERSION:
            dtype_ind = self.data[name_end + 78] if name_end + 78 < len(self.data) else 3
        else:
            dtype_ind = self.data[name_end + 133] if name_end + 133 < len(self.data) else 3

        dtype_map = {0: "UBYTE", 1: "UBYTE", 2: "SBYTE", 3: "UWORD", 4: "SWORD", 5: "ULONG", 6: "SLONG"}
        data_type = dtype_map.get(dtype_ind, "UWORD")

        # Read unit string and factor (position varies by version)
        unit = ""
        factor = 1.0
        if self.version < self.NEW_FORMAT_VERSION:
            # Old format: unit length at +86, unit at +90, factor follows unit
            unit_len = self.data[name_end + 86] if name_end + 86 < len(self.data) else 0
            if 0 < unit_len < 20:
                unit_bytes = self.data[name_end + 90:name_end + 90 + unit_len]
                unit = unit_bytes.decode('ascii', errors='replace').rstrip('\x00').strip()
            # Factor at 4-byte aligned position after unit (around +92)
            factor_pos = name_end + 92
            if factor_pos + 8 <= len(self.data):
                factor = struct.unpack_from('<d', self.data, factor_pos)[0]
                if not (1e-15 < abs(factor) < 1e15):
                    factor = 1.0
        else:
            # New format: unit length at +141, unit at +145, factor follows unit
            unit_len = self.data[name_end + 141] if name_end + 141 < len(self.data) else 0
            if 0 < unit_len < 20 and unit_len != 0xff:
                unit_bytes = self.data[name_end + 145:name_end + 145 + unit_len]
                unit = unit_bytes.decode('ascii', errors='replace').rstrip('\x00').strip()
                factor_pos = name_end + 145 + unit_len
            else:
                factor_pos = name_end + 145
            if factor_pos + 8 <= len(self.data):
                factor = struct.unpack_from('<d', self.data, factor_pos)[0]
                if not (1e-15 < abs(factor) < 1e15):
                    factor = 1.0

        # Extract offsets - format varies by version
        data_offset = 0
        x_axis = None
        y_axis = None

        if self.version < self.NEW_FORMAT_VERSION:
            # Old format (< 300, e.g., v250): search for '0b 00 [addr]' marker pattern
            # This pattern appears consistently for both VALUE and CURVE/MAP types
            # The marker is at variable offset (typically 115-140) after parameter name

            # Search for 0x0b 0x00 marker followed by valid address
            marker_pos = None
            for check in range(110, 150):
                if name_end + check + 4 <= len(self.data):
                    if self.data[name_end + check] == 0x0b and self.data[name_end + check + 1] == 0x00:
                        addr = self._read_u16(name_end + check + 2)
                        # Validate address is in reasonable CAL range
                        if 0x0100 <= addr <= 0xFFFF:
                            marker_pos = check
                            data_offset = addr
                            break

            # If no marker found, try the legacy 0x80 pattern for CURVE/MAP types
            if marker_pos is None and param_type in ("CURVE", "MAP"):
                for check in range(100, 130):
                    if name_end + check + 7 <= len(self.data):
                        if self.data[name_end + check] == 0x80:
                            addr1 = self._read_u16(name_end + check + 1)
                            padding = self._read_u16(name_end + check + 3)
                            addr2 = self._read_u16(name_end + check + 5)
                            # Pattern: 0x80 [addr] 00 00 [addr+delta]
                            if padding == 0 and 0x0100 <= addr1 <= 0xFFFF:
                                delta = addr2 - addr1 if addr2 >= addr1 else 0
                                # For CURVE/MAP, delta should match cols or rows
                                if 0 <= delta <= 1000:
                                    data_offset = addr1
                                    break

            # Extract axis info for CURVE/MAP if we have a valid data offset
            if data_offset > 0 and param_type in ("CURVE", "MAP"):
                # Search for 0x80 pattern which contains axis offset info
                for check in range(100, 130):
                    if name_end + check + 7 <= len(self.data):
                        if self.data[name_end + check] == 0x80:
                            addr1 = self._read_u16(name_end + check + 1)
                            addr2 = self._read_u16(name_end + check + 5)
                            if addr1 == data_offset and addr2 > addr1:
                                # addr2 is start of axis data (or end of parameter data)
                                # For CURVE: axis at addr2, with cols points
                                # For MAP: x-axis at addr2, y-axis follows
                                points = cols if cols > 1 else rows
                                x_axis = AxisInfo(points=points, offset=addr2)
                                if param_type == "MAP" and rows > 1:
                                    y_axis = AxisInfo(points=rows, offset=addr2 + cols)
                                break

        elif self.version < self.NEWER_FORMAT_VERSION:
            # Intermediate format (300-399): 0x80 flag at +163, 24-bit offsets at +164
            if name_end + 172 <= len(self.data):
                # Read 24-bit data offset at +164-166
                data_offset = (self.data[name_end + 164] |
                              (self.data[name_end + 165] << 8) |
                              (self.data[name_end + 166] << 16))

                if param_type in ("CURVE", "MAP"):
                    # Read 24-bit axis offset at +168-170
                    xaxis_offset = (self.data[name_end + 168] |
                                   (self.data[name_end + 169] << 8) |
                                   (self.data[name_end + 170] << 16))

                    x_axis = AxisInfo(points=cols, offset=xaxis_offset)
                    if param_type == "MAP":
                        y_axis = AxisInfo(points=rows, offset=xaxis_offset + cols)
        else:
            # New format (>= 400): search for 0x80 flag in range 158-170, then read 24-bit offsets
            if name_end + 175 <= len(self.data):
                flag_found = False
                flag_pos = 160  # Default position

                # Search for 0x80 flag in expected range
                for check_pos in range(158, 170):
                    if self.data[name_end + check_pos] == 0x80:
                        flag_pos = check_pos
                        flag_found = True
                        break

                # Read 24-bit data offset after flag
                ofs_pos = flag_pos + 1
                data_offset = (self.data[name_end + ofs_pos] |
                              (self.data[name_end + ofs_pos + 1] << 8) |
                              (self.data[name_end + ofs_pos + 2] << 16))

                # Validate offset (should be < 1MB for typical files)
                if data_offset > 0x100000 and not flag_found:
                    # No flag found and offset looks invalid, try fixed position
                    data_offset = (self.data[name_end + 161] |
                                  (self.data[name_end + 162] << 8) |
                                  (self.data[name_end + 163] << 16))
                    if data_offset > 0x100000:
                        data_offset = 0

                if param_type in ("CURVE", "MAP"):
                    # Read 24-bit axis offset 4 bytes after data offset
                    axis_ofs_pos = ofs_pos + 4
                    xaxis_offset = (self.data[name_end + axis_ofs_pos] |
                                   (self.data[name_end + axis_ofs_pos + 1] << 8) |
                                   (self.data[name_end + axis_ofs_pos + 2] << 16))

                    x_axis = AxisInfo(points=cols, offset=xaxis_offset)
                    if param_type == "MAP":
                        y_axis = AxisInfo(points=rows, offset=xaxis_offset + cols)

        # Extract description (search backwards from metadata)
        description = self._find_description_before(name_pos)

        # Extract folder_id from metadata[5] (for v300+ files)
        folder_id = 0
        if self.version >= self.NEW_FORMAT_VERSION:
            meta_start = name_pos - 24
            if meta_start >= 0:
                folder_id = self._read_u32(meta_start + 20)  # metadata[5]
                # Sanity check - folder IDs should be small
                if folder_id > 1000:
                    folder_id = 0

        return Parameter(
            name=name,
            description=description,
            param_type=param_type,
            data_type=data_type,
            cols=cols,
            rows=rows,
            data_offset=data_offset,
            x_axis=x_axis,
            y_axis=y_axis,
            unit=unit,
            factor=factor,
            folder_id=folder_id,
        )

    def _find_description_before(self, name_pos: int) -> str:
        """Find length-prefixed description string before parameter name"""
        # Metadata is 24 bytes before name, description is before that
        meta_start = name_pos - 24
        if meta_start < 4:
            return ""

        # Skip trailing zeros to find description end
        desc_end = meta_start
        while desc_end > 0 and self.data[desc_end - 1] == 0:
            desc_end -= 1

        if desc_end < 10:
            return ""

        # Search backwards for length-prefixed string
        for back in range(1, min(500, desc_end)):
            check_pos = desc_end - back
            if check_pos < 4:
                break

            try:
                str_len = self._read_u32(check_pos)
                if 10 < str_len < 500 and check_pos + 4 + str_len <= desc_end:
                    desc = self.data[check_pos + 4:check_pos + 4 + str_len]
                    desc = desc.decode('cp1252', errors='replace').strip()
                    if desc and all(c.isprintable() or c == ' ' for c in desc):
                        return desc
            except (struct.error, ValueError):
                pass

        return ""

    def _extract_cal_block(self) -> Optional[CALBlock]:
        """
        Extract embedded CAL (calibration) binary data.

        CAL block structure:
        - Identified by "CAS" + 5 alphanumeric chars (e.g., "CASC2E55")
        - EPK string at offset 8 (e.g., "SC8F830")
        - Binary data follows, typically 0.5-2 MB
        - End marked by large 0xFF section or EOF
        """
        # Find CAS signature
        pattern = rb'CAS[A-Z][A-Z0-9]{4}'
        match = re.search(pattern, self.data)

        if not match:
            return None

        offset = match.start()

        # Read CAS ID and EPK
        cas_id = self.data[offset:offset + 8].decode('ascii', errors='replace')

        # EPK is at offset 8, typically 7 chars
        epk_area = self.data[offset + 8:offset + 30].decode('ascii', errors='replace')
        epk_match = re.match(r'(S[A-Z][0-9A-Z]{5})', epk_area)
        epk = epk_match.group(1) if epk_match else ""

        # Find CAL end (large 0xFF section or EOF)
        end = min(offset + 0x200000, len(self.data))
        ff_count = 0

        for i in range(offset + 0x10000, end):
            if self.data[i] == 0xff:
                ff_count += 1
                if ff_count > 0x8000:  # 32KB of 0xFF
                    end = i - ff_count
                    break
            else:
                ff_count = 0

        # Align to 4KB boundary
        end = ((end + 0xfff) // 0x1000) * 0x1000
        end = min(end, len(self.data))

        cal_data = self.data[offset:end]

        # Verify it starts with CAS
        if not cal_data[:3] == b'CAS':
            return None

        return CALBlock(
            offset=offset,
            cas_id=cas_id,
            epk=epk,
            data=cal_data,
        )

    def extract_binary(self, version: BinaryVersion) -> bytes:
        """
        Extract binary blob data for a given binary version.

        Args:
            version: BinaryVersion with blob_offset and blob_size

        Returns:
            Raw binary data bytes
        """
        if version.blob_offset == 0 or version.blob_size == 0:
            return b""

        if version.blob_offset + version.blob_size > len(self.data):
            # Truncate to available data
            return self.data[version.blob_offset:]

        return self.data[version.blob_offset:version.blob_offset + version.blob_size]


def read_ols(filepath: str | Path) -> OLSFile:
    """
    Convenience function to parse an OLS file.

    Args:
        filepath: Path to .ols file

    Returns:
        OLSFile with parsed contents
    """
    return OLSReader(filepath).parse()


# CLI usage
if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python ols_reader.py <file.ols>")
        sys.exit(1)

    ols = read_ols(sys.argv[1])

    print(f"File: {ols.filename}")
    print(f"OLS Version: {ols.version}")
    print(f"Vehicle: {ols.make} {ols.model} {ols.engine} ({ols.year})")
    print(f"Parameters: {len(ols.parameters)}")

    if ols.binary_versions:
        print(f"\nBinary versions ({len(ols.binary_versions)}):")
        for v in ols.binary_versions:
            if v.version_count > 0:
                # v597+ format with version index
                idx_str = f"idx={v.version_index}/{v.version_count}"
            elif v.is_root:
                idx_str = "ROOT"
            else:
                idx_str = f"parent={v.parent_id}"
            size_str = f" ({v.blob_size / 1024:.0f} KB)" if v.blob_size else ""
            offset_str = f" @{hex(v.blob_offset)}" if v.blob_offset else ""
            print(f"  {v.name} ({idx_str}){size_str}{offset_str}")

    if ols.cal_block:
        print(f"\nCAL: {ols.cal_block.epk} ({len(ols.cal_block.data) / 1024:.1f} KB)")

    # Show sample parameters with offsets, data type, factor, and folder IDs
    print("\nSample parameters:")
    for p in ols.parameters[:10]:
        axis_info = ""
        if p.x_axis:
            axis_info = f" [xaxis@{hex(p.x_axis.offset)}]"
        folder_str = f" folder={p.folder_id}" if p.folder_id else ""
        factor_str = f" *{p.factor}" if p.factor != 1.0 else ""
        unit_str = f" {p.unit}" if p.unit else ""
        print(f"  {p.param_type:5} {p.data_type:5} {p.name}: @{hex(p.data_offset)}{factor_str}{unit_str}{axis_info}{folder_str}")
