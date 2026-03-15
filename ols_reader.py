"""
OLS File Reader - Parse WinOLS .ols and .kp (Kennfeld Pack) files

This module provides a clean interface for reading WinOLS .ols files and
.kp (map pack) files, extracting parameter definitions, addresses, and
embedded binary data.

OLS files are proprietary project files used by WinOLS ECU tuning software.
They contain vehicle metadata, parameter definitions with addresses, and
optionally embedded binary data.

KP files (Kennfeld Pack / Map Pack) are a variant used for sharing map
definitions. They use ZIP compression for parameter storage and contain
direct file offsets rather than embedded binary data.

Supported OLS Versions:
    - v250-285: Old format (limited binary version support)
    - v300-399: Intermediate format (parent ID based versions)
    - v400-596: Standard format (parent ID based versions)
    - v597+:    Multi-version format (edit history with version slots)
    - v597+ KP: Map pack format with ZIP-compressed parameters
    - v804+:    WinOLS 5 format (15 metadata strings, derived version records)

Usage:
    from ols_reader import read_ols, OLSReader

    # Quick read (works for both .ols and .kp files)
    ols = read_ols("file.ols")
    print(f"Parameters: {len(ols.parameters)}")

    # KP files contain direct file offsets
    ols = read_ols("mappack.kp")
    for param in ols.parameters:
        print(f"{param.name}: @0x{param.data_offset:x}")

    # With binary extraction (OLS files only)
    reader = OLSReader("file.ols")
    ols = reader.parse()
    for version in ols.binary_versions:
        data = reader.extract_binary(version)
        with open(f"{version.name}.bin", "wb") as f:
            f.write(data)
"""

from __future__ import annotations

import io
import re
import struct
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class FolderEntry:
    """Folder/function entry from ols structure"""
    idx: int  # Folder index (matches folder_id in parameters)
    name: str  # Folder name (e.g., "ACQ_KNK_sensor_signal_gen")
    description: str = ""  # Description (e.g., "Acquisition of knock sensor signal (gen)")


@dataclass
class AxisInfo:
    """Axis definition for CURVE/MAP parameters"""
    points: int
    address: int = 0  # CAL offset/address for axis data
    offset: int = 0  # Conversion offset (physical = raw * factor + offset)
    unit: str = ""
    factor: float = 1.0  # Conversion factor (physical = raw * factor + offset)
    id: str = ""  # Axis ID (e.g., "n_32", "_CNV_A_R_LINEAR_____33_CM")
    description: str = ""  # Axis description (e.g., "Engine speed -Resolution 32 rpm")
    data_source: int = 0  # Data source type (1=values, 2=eeprom, etc.)
    data_type: str = "UWORD"  # Data type (UBYTE, UWORD, etc.)


@dataclass
class Parameter:
    """ECU parameter definition extracted from OLS file"""
    name: str
    description: str = ""
    param_type: str = "VALUE"  # VALUE, CURVE, MAP, or MAP_INVERSE
    data_type: str = "UWORD"  # UBYTE, SBYTE, UWORD, SWORD, ULONG, SLONG
    cols: int = 1
    rows: int = 1
    data_offset: int = 0  # CAL offset for parameter data
    x_axis: Optional[AxisInfo] = None
    y_axis: Optional[AxisInfo] = None
    unit: str = ""
    factor: float = 1.0  # Conversion factor (physical = raw * factor + offset)
    offset: float = 0.0  # Conversion offset
    folder_id: int = 0  # Folder/category ID (numeric ID only - folder names not found in OLS files)
    type_code: int = 0  # Raw type code from metadata (2=VALUE, 3=CURVE, 4=MAP, 5=MAP_INVERSE)


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

    Verification:
        - epk: Identification string found in binary (e.g., "SC1C910", "SC8F830")
        - epk_offset: Offset relative to blob_offset where EPK was found
    """
    name: str
    description: str = ""  # Import path/creation description
    parent_id: Optional[int] = None  # None = root folder (0x003fffff marker)
    is_root: bool = False
    blob_offset: int = 0  # File offset where binary blob starts
    blob_size: int = 0  # Binary blob size in bytes
    version_index: int = 0  # Version index (0-based) for multi-version files (v597+)
    version_count: int = 0  # Total number of version slots (v597+)
    epk: str = ""  # EPK/identification string found in binary
    epk_offset: int = 0  # Offset relative to blob_offset where EPK was found


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
    version: int = 0  # OLS format version (e.g., 250, 440, 478, 804)
    make: str = ""
    model: str = ""
    engine: str = ""
    year: str = ""
    fuel_type: str = ""
    displacement: str = ""
    power: str = ""
    gearbox: str = ""
    memory_type: str = ""
    ecu_manufacturer: str = ""
    ecu_type: str = ""
    sw_number: str = ""
    hw_number: str = ""
    hw_code: str = ""
    user: str = ""
    parameters: list[Parameter] = field(default_factory=list)
    binary_versions: list[BinaryVersion] = field(default_factory=list)
    cal_block: Optional[CALBlock] = None
    is_kp_file: bool = False  # True if this is a KP (Kennfeld Pack) file
    big_endian_data: bool = False  # True if binary data should be read as big-endian (common for KP/DSG)
    folders: list[FolderEntry] = field(default_factory=list)  # Folder/Function entries


class OLSReader:
    """
    Reader for WinOLS .ols project files.

    Supports OLS format versions 100 through 834+, including:
    - Sequential parameter extraction (reads fields in WinOLS order)
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

    # Format version thresholds (for binary version / folder detection only)
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

        # Check if this is a KP (Kennfeld Pack) file with ZIP-compressed parameters
        if self._is_kp_file():
            ols.is_kp_file = True
            ols.big_endian_data = True  # KP files typically have big-endian data
            ols.parameters = self._extract_kp_parameters()
            # KP files don't have embedded binary versions or CAL blocks
            return ols

        # Extract binary versions (Daten, Original, NOCS, etc.)
        ols.binary_versions = self._extract_binary_versions()

        # Extract parameters with addresses
        ols.parameters = self._extract_parameters()

        # Find and extract CAL block
        ols.cal_block = self._extract_cal_block()

        ols.folders = self._extract_folder_entries()

        return ols

    def _is_kp_file(self) -> bool:
        """
        Check if this is a KP (Kennfeld Pack) file.

        KP files are v597+ OLS files that contain ZIP-compressed parameter data
        instead of raw parameter records. They're identified by:
        - Version >= 597
        - ZIP signature (PK\\x03\\x04) present after header
        - Contains "intern" file in the ZIP archive
        """
        if self.version < 597:
            return False

        # Look for ZIP signature after header (typically around 0x400-0x500)
        zip_sig = b'PK\x03\x04'
        zip_pos = self.data.find(zip_sig, 0x100, 0x1000)

        if zip_pos < 0:
            return False

        # Verify it's a valid ZIP with "intern" file
        try:
            zip_data = self.data[zip_pos:]
            with zipfile.ZipFile(io.BytesIO(zip_data)) as zf:
                return 'intern' in zf.namelist()
        except (zipfile.BadZipFile, Exception):
            return False

    def _extract_kp_parameters(self) -> list[Parameter]:
        """
        Extract parameters from KP (Kennfeld Pack) file.

        KP files store parameters in a ZIP-compressed "intern" file with structure:
        - Metadata (6 x u32): type_code, and dimension info
        - Name length (u32) + name string
        - Post-name data (~0x9c-0xb0 bytes) containing address

        Addresses in KP files are direct file offsets into the corresponding .bin file.
        """
        parameters = []
        found_names: set[str] = set()

        # Find and extract intern file
        zip_sig = b'PK\x03\x04'
        zip_pos = self.data.find(zip_sig, 0x100, 0x1000)

        if zip_pos < 0:
            return parameters

        try:
            zip_data = self.data[zip_pos:]
            with zipfile.ZipFile(io.BytesIO(zip_data)) as zf:
                intern_data = zf.read('intern')
        except (zipfile.BadZipFile, KeyError, Exception):
            return parameters

        # Parse parameters from intern file
        pos = 0
        while pos < len(intern_data) - 300:
            try:
                # Read potential metadata (6 x u32)
                meta = struct.unpack_from('<6I', intern_data, pos)
                type_code = meta[0]

                # Validate metadata pattern
                if type_code in (2, 3) or 4 <= type_code <= 20:
                    if all(m < 1000 for m in meta[1:5]):
                        # Check for name length at pos + 24
                        name_len = struct.unpack_from('<I', intern_data, pos + 24)[0]

                        if 3 < name_len < 60 and pos + 28 + name_len < len(intern_data):
                            try:
                                name = intern_data[pos + 28:pos + 28 + name_len].decode('ascii').rstrip('\x00')

                                # Validate name
                                clean_name = name.replace('_', '').replace('[', '').replace(']', '').replace('.', '')
                                if (name.isprintable() and
                                    clean_name.isalnum() and
                                    len(name) >= 3 and
                                    ('_' in name or '.' in name) and
                                    name not in found_names):

                                    found_names.add(name)
                                    param = self._parse_kp_parameter_record(
                                        intern_data, pos, name_len, name, meta
                                    )
                                    if param:
                                        parameters.append(param)

                                    # Skip to next record
                                    pos = pos + 28 + name_len + 0x100
                                    continue
                            except (UnicodeDecodeError, ValueError):
                                pass
            except struct.error:
                pass

            pos += 4

        return parameters

    def _parse_kp_parameter_record(
        self,
        intern_data: bytes,
        meta_pos: int,
        name_len: int,
        name: str,
        meta: tuple[int, ...]
    ) -> Optional[Parameter]:
        """
        Parse a single parameter record from KP intern file.

        Structure:
        - Metadata at meta_pos (6 x u32)
        - Name at meta_pos + 28
        - Post-name data: ~0xa0 bytes containing address and axis info
        - Address is at variable offset 0x9c-0xb0 from name end
        """
        name_end = meta_pos + 28 + name_len

        if name_end + 0xb0 > len(intern_data):
            return None

        # Determine type from metadata
        type_code = meta[0]
        if type_code == 2:
            param_type = "VALUE"
        elif type_code == 3:
            param_type = "CURVE"
        else:
            param_type = "MAP"

        # Extract dimensions from metadata (meta[4] often contains points/cols info)
        cols = 1
        rows = 1
        if param_type == "CURVE":
            cols = meta[4] if 1 < meta[4] < 100 else 1
        elif param_type == "MAP":
            cols = meta[4] if 1 < meta[4] < 100 else 1
            rows = meta[5] if 1 < meta[5] < 100 else 1

        # Find address - scan range 0x9c-0xb0 from name_end
        # KP addresses are direct file offsets stored as little-endian
        # Typically in 0x60000-0x80000 range for DSG
        data_offset = 0
        for off in range(0x9c, 0xb0):
            addr_pos = name_end + off
            if addr_pos + 4 <= len(intern_data):
                candidate = struct.unpack_from('<I', intern_data, addr_pos)[0]
                # Valid address range depends on ECU type
                # DSG: 0x60000-0x80000, but allow wider range for other ECUs
                if 0x1000 <= candidate < 0x800000:
                    data_offset = candidate
                    break

        # Try to find axis offset (typically 4 bytes after data offset)
        x_axis = None
        y_axis = None
        if data_offset > 0 and param_type in ("CURVE", "MAP"):
            for off in range(0x9c, 0xb0):
                addr_pos = name_end + off
                if addr_pos + 8 <= len(intern_data):
                    addr1 = struct.unpack_from('<I', intern_data, addr_pos)[0]
                    addr2 = struct.unpack_from('<I', intern_data, addr_pos + 4)[0]
                    if addr1 == data_offset and 0x1000 <= addr2 < 0x800000:
                        x_axis = AxisInfo(points=cols, offset=addr2)
                        if param_type == "MAP" and rows > 1:
                            # Y-axis offset typically follows X-axis data
                            y_axis = AxisInfo(points=rows, offset=addr2 + cols * 2)
                        break

        # Try to extract description (search backwards from metadata)
        description = ""
        if meta_pos > 50:
            # Look for length-prefixed string before metadata
            for back in range(4, min(200, meta_pos), 4):
                check_pos = meta_pos - back
                try:
                    str_len = struct.unpack_from('<I', intern_data, check_pos)[0]
                    if 5 < str_len < 200 and check_pos + 4 + str_len <= meta_pos:
                        desc_bytes = intern_data[check_pos + 4:check_pos + 4 + str_len]
                        desc = desc_bytes.decode('cp1252', errors='replace').rstrip('\x00').strip()
                        if desc and len(desc) > 5 and all(c.isprintable() or c in ' \t' for c in desc):
                            description = desc
                            break
                except (struct.error, UnicodeDecodeError):
                    pass

        return Parameter(
            name=name,
            description=description,
            param_type=param_type,
            data_type="UWORD",  # KP files typically use UWORD (big-endian in the bin file)
            cols=cols,
            rows=rows,
            data_offset=data_offset,
            x_axis=x_axis,
            y_axis=y_axis,
        )

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
        - [22+]   Length-prefixed strings (up to 15):
                   make, model, engine, year, fuel_type, displacement, power,
                   gearbox, memory_type, ecu_manufacturer, ecu_type, sw_number,
                   hw_number, hw_code, user

        Older OLS versions may have fewer strings (minimum 4).
        v597+ typically have all 15 strings (some may be empty).
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
        pos += consumed

        # Extended metadata strings (v597+, up to 11 more)
        # Each is length-prefixed; older files may stop early
        extended_fields = [
            'fuel_type', 'displacement', 'power', 'gearbox',
            'memory_type', 'ecu_manufacturer', 'ecu_type',
            'sw_number', 'hw_number', 'hw_code', 'user',
        ]
        for field_name in extended_fields:
            if pos + 4 >= len(self.data):
                break
            val, consumed = self._read_string(pos)
            if consumed <= 0:
                break
            # Stop if the length field looks like non-string data
            # (length-prefixed strings should have length < 200)
            raw_len = self._read_u32(pos)
            if raw_len > 200:
                break
            setattr(ols, field_name, val)
            pos += consumed

        return True

    def _extract_binary_versions(self) -> list[BinaryVersion]:
        """
        Extract binary version entries (Original, Daten, NOCS, user-defined, etc.)

        Binary versions are stored in the header area. Two formats exist:

        v597+ format (including v804):
            Base record (identified by 0x42007899 marker):
                [version_index:4] [blob_offset:4] [blob_size:4] [0x42007899:4]
                [version_count:4] [name_len:4] [name] [desc_len:4] [desc]

            Derived record (for additional versions like user edits):
                [0xFFFFFFFF:4] [name_len:4] [name] [desc_len:4] [desc]

            Blob storage: blob_offset + i * blob_size for version i.
            file_size == blob_offset + version_count * blob_size + 4 (0xFFFFFFFF trailer).

        Old format (version < 597):
            Root folders: 0x003fffff marker + length + name
            Child versions: [blob_offset at pos-12] [blob_size at pos-8] [parent_id] + length + name
        """
        versions = []

        # --- Try v597+ format: scan for 0x42007899 marker ---
        BLOB_MARKER = 0x42007899
        marker_bytes = struct.pack('<I', BLOB_MARKER)
        scan_start = 0x100
        scan_end = min(0x2000, len(self.data) - 28)

        base_found = False
        marker_pos = self.data.find(marker_bytes, scan_start, scan_end)

        while marker_pos >= 0 and not base_found:
            # Base record: marker is at offset +12 from record start
            rec_start = marker_pos - 12
            if rec_start < 0:
                marker_pos = self.data.find(marker_bytes, marker_pos + 4, scan_end)
                continue

            version_index = self._read_u32(rec_start)
            blob_offset = self._read_u32(rec_start + 4)
            blob_size = self._read_u32(rec_start + 8)
            # marker at rec_start + 12
            version_count = self._read_u32(rec_start + 16)
            name_len = self._read_u32(rec_start + 20)

            # Validate base record structure
            if (version_index < 20 and
                version_count > 0 and version_count <= 20 and
                version_index < version_count and
                0x100000 <= blob_size <= 0x1000000 and
                blob_offset + version_count * blob_size <= len(self.data) + 4 and
                0 < name_len < 50 and
                rec_start + 24 + name_len <= len(self.data)):

                try:
                    name = self.data[rec_start + 24:rec_start + 24 + name_len].decode('cp1252')
                    if name.isprintable() and name.strip():
                        name_clean = name.strip()

                        # Read description after name
                        desc = ""
                        desc_pos = rec_start + 24 + name_len
                        if desc_pos + 4 <= len(self.data):
                            desc_len = self._read_u32(desc_pos)
                            if 0 < desc_len < 500 and desc_pos + 4 + desc_len <= len(self.data):
                                desc = self.data[desc_pos + 4:desc_pos + 4 + desc_len].decode('cp1252', errors='replace').strip()

                        actual_blob_offset = blob_offset + version_index * blob_size

                        versions.append(BinaryVersion(
                            name=name_clean,
                            blob_offset=actual_blob_offset,
                            blob_size=blob_size,
                            version_index=version_index,
                            version_count=version_count,
                            description=desc,
                        ))

                        base_found = True

                        # Scan forward for derived records (0xFFFFFFFF sentinel)
                        derived_search_start = marker_pos + 4
                        derived_search_end = min(derived_search_start + 0x500, len(self.data) - 8)
                        derived_index = 0  # Next slot to assign

                        d_pos = derived_search_start
                        while d_pos < derived_search_end:
                            if self._read_u32(d_pos) == 0xFFFFFFFF:
                                if d_pos + 8 <= len(self.data):
                                    d_name_len = self._read_u32(d_pos + 4)
                                    if (0 < d_name_len < 50 and
                                        d_pos + 8 + d_name_len <= len(self.data)):
                                        try:
                                            d_name = self.data[d_pos + 8:d_pos + 8 + d_name_len].decode('cp1252')
                                            if d_name.isprintable() and d_name.strip():
                                                # Skip base record's index
                                                while derived_index == version_index:
                                                    derived_index += 1
                                                if derived_index >= version_count:
                                                    break

                                                d_desc = ""
                                                d_desc_pos = d_pos + 8 + d_name_len
                                                if d_desc_pos + 4 <= len(self.data):
                                                    d_desc_len = self._read_u32(d_desc_pos)
                                                    if 0 < d_desc_len < 500 and d_desc_pos + 4 + d_desc_len <= len(self.data):
                                                        d_desc = self.data[d_desc_pos + 4:d_desc_pos + 4 + d_desc_len].decode('cp1252', errors='replace').strip()

                                                d_blob_offset = blob_offset + derived_index * blob_size

                                                versions.append(BinaryVersion(
                                                    name=d_name.strip(),
                                                    blob_offset=d_blob_offset,
                                                    blob_size=blob_size,
                                                    version_index=derived_index,
                                                    version_count=version_count,
                                                    description=d_desc,
                                                ))

                                                derived_index += 1
                                                # Skip past this derived record
                                                d_pos = d_desc_pos + (4 + d_desc_len if d_desc else 4)
                                                continue
                                        except (UnicodeDecodeError, ValueError):
                                            pass
                            d_pos += 1

                except (UnicodeDecodeError, ValueError):
                    pass

            if not base_found:
                marker_pos = self.data.find(marker_bytes, marker_pos + 4, scan_end)

        # --- Fallback: old format (pre-v597 or files without 0x42007899 marker) ---
        if not base_found:
            pos = 0x400
            old_scan_end = min(0x2000, len(self.data) - 28)

            while pos < old_scan_end:
                val = self._read_u32(pos)

                # Root folder marker (0x003fffff)
                if val == 0x003fffff:
                    name_len = self._read_u32(pos + 4)
                    if 0 < name_len < 50:
                        try:
                            name = self.data[pos + 8:pos + 8 + name_len].decode('cp1252')
                            if name.isprintable() and name.strip():
                                versions.append(BinaryVersion(
                                    name=name.strip(),
                                    is_root=True,
                                    parent_id=None,
                                ))
                                pos += 8 + name_len
                                continue
                        except (UnicodeDecodeError, ValueError):
                            pass

                # Version with parent ID (small integer 1-10)
                if 0 < val <= 10:
                    name_len = self._read_u32(pos + 4)
                    if 0 < name_len < 50 and pos + 8 + name_len <= len(self.data):
                        try:
                            name = self.data[pos + 8:pos + 8 + name_len].decode('cp1252')
                            if name.isprintable() and name.strip():
                                name_clean = name.strip()

                                # Validate blob_offset and blob_size from preceding fields
                                blob_offset = 0
                                blob_size = 0
                                if pos >= 16:
                                    blob_offset = self._read_u32(pos - 12)
                                    blob_size = self._read_u32(pos - 8)
                                    if blob_offset > len(self.data) or blob_size > 0x10000000:
                                        blob_offset = 0
                                        blob_size = 0

                                # Only accept if we have valid blob info
                                if blob_offset > 0 and blob_size > 0:
                                    desc = ""
                                    desc_pos = pos + 8 + name_len
                                    if desc_pos + 4 <= len(self.data):
                                        desc_len = self._read_u32(desc_pos)
                                        if 0 < desc_len < 500 and desc_pos + 4 + desc_len <= len(self.data):
                                            desc_bytes = self.data[desc_pos + 4:desc_pos + 4 + desc_len]
                                            desc = desc_bytes.decode('cp1252', errors='replace').strip()

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

        # Find EPK/verification info for each binary version
        for v in versions:
            if v.blob_offset > 0 and v.blob_size > 0:
                epk, epk_offset = self._find_binary_verification(v.blob_offset, v.blob_size)
                v.epk = epk
                v.epk_offset = epk_offset

        return versions

    def _find_binary_verification(self, blob_offset: int, blob_size: int) -> tuple[str, int]:
        """
        Search for EPK/identification string within a binary block.

        First searches at known CAL offsets within the binary, then falls back
        to searching the entire file for CAS/EPK patterns.

        Returns:
            (epk, offset) where offset is relative to blob_offset (or 0 if found outside)
        """
        # Known CAL offsets for different ECU types (from EPK prefix)
        # Simos18.10 (Aurix TC29x): SCG -> 0x820000
        # Simos18.1 (TC1793): SC8 -> 0x800000
        # Simos12 (TC1797): SC1, SA3 -> 0x40000
        # Simos16: SC4 -> 0x100000
        # MED17: S82, S85 -> 0x40000
        cal_offsets = [0x40000, 0x100000, 0x800000, 0x820000, 0x0]

        # Method 1: Search at known CAL offsets within binary
        for cal_off in cal_offsets:
            if cal_off + 20 > blob_size:
                continue

            pos = blob_offset + cal_off
            if pos + 20 > len(self.data):
                continue

            header = self.data[pos:pos + 20]
            epk, rel_offset = self._extract_epk_from_header(header)
            if epk:
                return epk, cal_off + rel_offset

        # Method 2: Search entire file for CAS pattern (for files where CAL is separate)
        cas_pattern = rb'CAS[A-Z][A-Z0-9]{4}'
        for match in re.finditer(cas_pattern, self.data):
            cas_pos = match.start()
            # EPK is 8 bytes after CAS
            epk_pos = cas_pos + 8
            if epk_pos + 7 <= len(self.data):
                epk_bytes = self.data[epk_pos:epk_pos + 7]
                epk = epk_bytes.split(b'\x00')[0].decode('ascii', errors='replace')
                if epk and len(epk) >= 6 and epk[0] == 'S':
                    # Calculate offset relative to binary start
                    rel_offset = cas_pos - blob_offset + 8
                    # Use known CAL offset based on EPK prefix
                    known_offset = self._get_cal_offset_for_epk(epk)
                    if known_offset > 0:
                        return epk, known_offset + 8  # +8 for CAS header
                    return epk, rel_offset if rel_offset >= 0 else 0

        return "", 0

    def _extract_epk_from_header(self, header: bytes) -> tuple[str, int]:
        """Extract EPK from a 20-byte header area."""
        # Look for CAS header, EPK follows at +8
        cas_match = re.search(rb'CAS[A-Z][A-Z0-9]{4}', header)
        if cas_match:
            epk_pos = cas_match.start() + 8
            if epk_pos + 7 <= len(header):
                epk_bytes = header[epk_pos:epk_pos + 7]
                epk = epk_bytes.split(b'\x00')[0].decode('ascii', errors='replace')
                if epk and len(epk) >= 6:
                    return epk, epk_pos

        # Direct EPK search (for files without CAS header)
        epk_match = re.search(rb'S[A-Z][0-9A-Z]{5}\x00', header)
        if epk_match:
            epk = epk_match.group()[:-1].decode('ascii')
            return epk, epk_match.start()

        return "", 0

    def _get_cal_offset_for_epk(self, epk: str) -> int:
        """Return CAL offset based on EPK prefix (for full bin files)."""
        epk_upper = epk.upper()
        # Simos18.10 (Aurix TC29x)
        if epk_upper.startswith('SCG'):
            return 0x820000
        # Simos18.1 (TC1793)
        if epk_upper.startswith('SC8'):
            return 0x800000
        # Simos12 (TC1797)
        if epk_upper.startswith('SC1') or epk_upper.startswith('SA3'):
            return 0x40000
        # Simos16
        if epk_upper.startswith('SC4'):
            return 0x100000
        # MED17
        if epk_upper.startswith('S82') or epk_upper.startswith('S85'):
            return 0x40000
        # Default
        return 0

    # -------------------------------------------------------------------
    # Sequential reading primitives (advance self.pos)
    # -------------------------------------------------------------------

    def _seq_read_u8(self) -> int:
        if self.pos + 1 > len(self.data):
            raise EOFError(f"read_u8 at 0x{self.pos:x}")
        val = self.data[self.pos]
        self.pos += 1
        return val

    def _seq_read_bool(self) -> bool:
        return self._seq_read_u8() != 0

    def _seq_read_u16(self) -> int:
        if self.pos + 2 > len(self.data):
            raise EOFError(f"read_u16 at 0x{self.pos:x}")
        val = struct.unpack_from('<H', self.data, self.pos)[0]
        self.pos += 2
        return val

    def _seq_read_u32(self) -> int:
        if self.pos + 4 > len(self.data):
            raise EOFError(f"read_u32 at 0x{self.pos:x}")
        val = struct.unpack_from('<I', self.data, self.pos)[0]
        self.pos += 4
        return val

    def _seq_read_i32(self) -> int:
        if self.pos + 4 > len(self.data):
            raise EOFError(f"read_i32 at 0x{self.pos:x}")
        val = struct.unpack_from('<i', self.data, self.pos)[0]
        self.pos += 4
        return val

    def _seq_read_f64(self) -> float:
        if self.pos + 8 > len(self.data):
            raise EOFError(f"read_f64 at 0x{self.pos:x}")
        val = struct.unpack_from('<d', self.data, self.pos)[0]
        self.pos += 8
        return val

    def _seq_read_bytes(self, count: int) -> bytes:
        if self.pos + count > len(self.data):
            raise EOFError(f"read_bytes({count}) at 0x{self.pos:x}")
        result = self.data[self.pos:self.pos + count]
        self.pos += count
        return result

    def _seq_read_array(self) -> bytes:
        """Read u32(byte_count) + byte_count bytes."""
        count = self._seq_read_u32()
        if count == 0 or count >= 0xFFFFFFF0:
            return b""
        if count > 0x1000000:
            raise ValueError(f"read_array at 0x{self.pos:x}: count {count}")
        if self.pos + count > len(self.data):
            raise EOFError(f"read_array at 0x{self.pos:x}: need {count}")
        result = self.data[self.pos:self.pos + count]
        self.pos += count
        return result

    def _seq_read_u32_array(self) -> list[int]:
        count = self._seq_read_u32()
        if count == 0:
            return []
        if count > 0x100000:
            raise ValueError(f"read_u32_array: count {count}")
        return [self._seq_read_u32() for _ in range(count)]

    def _seq_read_string(self) -> str:
        """Read version-aware length-prefixed string."""
        length = self._seq_read_u32()
        if length == 0 or length >= 0xFFFFFFF0:
            return ""
        if length > 0x100000:
            raise ValueError(f"read_string at 0x{self.pos:x}: length {length}")
        if self.pos + length > len(self.data):
            raise EOFError(f"read_string at 0x{self.pos:x}: need {length}")
        raw = self.data[self.pos:self.pos + length]
        if self._has_feature(439):
            self.pos += length
        else:
            self.pos += length + 1
        return raw.decode('cp1252', errors='replace').rstrip('\x00')

    def _seq_read_string_array(self) -> list[str]:
        count = self._seq_read_u32()
        return [self._seq_read_string() for _ in range(count)]

    def _seq_read_lang_string(self) -> str:
        s = self._seq_read_string()
        if self._has_feature(345):
            self._seq_read_u32()  # lang_id
        return s

    def _seq_read_multi_lang_string(self) -> str:
        if not self._has_feature(330):
            return self._seq_read_string()
        primary = self._seq_read_lang_string()
        if self._has_feature(345):
            self._seq_read_u32()  # flags
        sub_count = self._seq_read_u32()
        for _ in range(sub_count):
            self._seq_read_lang_string()
        return primary

    def _has_feature(self, feature_id: int) -> bool:
        return feature_id <= self.version

    # -------------------------------------------------------------------
    # Sequential map list detection
    # -------------------------------------------------------------------

    def _find_map_list_start(self) -> Optional[tuple[int, int]]:
        """
        Find the map list start position and count.

        The map list is preceded by checksum 0x11883377, then:
        - compressed (bool/u8)
        - count (u32)
        - map records

        Returns (first_map_pos, map_count) or None.
        """
        needle = struct.pack('<I', 0x11883377)
        idx = self.data.find(needle, 0x100, 0x20000)
        if idx < 0:
            return None

        pos = idx + 4
        if pos + 5 > len(self.data):
            return None

        compressed = self.data[pos]
        count = struct.unpack_from('<I', self.data, pos + 1)[0]
        first_map = pos + 5

        if compressed:
            return None

        if count == 0 or count > 0x100000:
            return None

        return (first_map, count)

    # -------------------------------------------------------------------
    # Sequential axis unit descriptor reader
    # -------------------------------------------------------------------

    def _seq_read_axis_unit_desc(self) -> tuple[str, float, float, int, int]:
        """
        Read axis unit descriptor.
        Returns (unit, factor, offset, data_offset, end_offset).
        """
        unit = self._seq_read_string()
        self._seq_read_string()  # alt_unit
        factor = self._seq_read_f64()
        offset = self._seq_read_f64()
        data_offset = self._seq_read_u32()
        end_offset = self._seq_read_u32()
        self._seq_read_u32()  # field_4c
        if self._has_feature(264):
            self._seq_read_f64()
        if self._has_feature(61):
            self._seq_read_u32()
        if self._has_feature(105):
            self._seq_read_u32()
            self._seq_read_u32()
            self._seq_read_u32()
            self._seq_read_i32()
        return (unit, factor, offset, data_offset, end_offset)

    # -------------------------------------------------------------------
    # Sequential axis data reader
    # -------------------------------------------------------------------

    def _seq_read_axis_data(self) -> AxisInfo:
        """Read a single axis data block."""
        description = self._seq_read_multi_lang_string()
        axis_id = self._seq_read_string()
        factor = self._seq_read_f64()
        offset = self._seq_read_f64()
        type_code = self._seq_read_u32()
        self._seq_read_u32()
        self._seq_read_u32()
        self._seq_read_u32()
        data_format = self._seq_read_u32()
        if data_format not in (2, 10, 16):
            data_format = 10
        data_type = {2: "UBYTE", 10: "UWORD", 16: "ULONG"}.get(data_format, "UWORD")
        self._seq_read_bool()
        self._seq_read_bool()
        if self._has_feature(264):
            self._seq_read_f64()
        if self._has_feature(241):
            self._seq_read_u32()
        data_source = 0
        if self._has_feature(8):
            data_source = self._seq_read_u32()
        if self._has_feature(805):
            self._seq_read_u32()
        if self._has_feature(12):
            self._seq_read_bool()
        if self._has_feature(73):
            self._seq_read_array()
        if self._has_feature(77):
            self._seq_read_u32()
        if self._has_feature(91):
            self._seq_read_i32()
        if self._has_feature(354):
            self._seq_read_string()
        if self._has_feature(372):
            count_372 = self._seq_read_u32()
            for _ in range(count_372):
                self._seq_read_multi_lang_string()
        if self._has_feature(440):
            self._seq_read_i32()
        if self._has_feature(805):
            self._seq_read_u32()
            self._seq_read_u32()
            self._seq_read_u32()
            self._seq_read_u32()
        return AxisInfo(
            points=0,
            unit="",
            factor=factor,
            offset=offset,
            id=axis_id,
            description=description,
            data_source=data_source,
            data_type=data_type,
        )

    # -------------------------------------------------------------------
    # Sequential map reader
    # -------------------------------------------------------------------

    def _seq_read_one_map(self) -> Parameter:
        """Read a single map record sequentially."""
        if self._has_feature(268):
            self._seq_read_u8()
        if self._has_feature(282):
            self._seq_read_u32()
        alt_description = ""
        if self._has_feature(287):
            alt_description = self._seq_read_multi_lang_string()
        if self._has_feature(93):
            self._seq_read_u8()

        description = self._seq_read_multi_lang_string()
        type_code = self._seq_read_u32()
        self._seq_read_u32()
        self._seq_read_u32()
        self._seq_read_u32()
        raw_ds = self._seq_read_u32()
        data_size = raw_ds if raw_ds in (2, 10, 16) else 10
        data_type = {2: "UBYTE", 10: "UWORD", 16: "ULONG"}.get(data_size, "UWORD")

        name = ""
        folder_id = 0
        if self._has_feature(80):
            folder_id = self._seq_read_u32()
            if folder_id > 1000:
                folder_id = 0
            name = self._seq_read_string()
        if self._has_feature(298):
            self._seq_read_u32()
        if self._has_feature(299):
            self._seq_read_u32()
        if self._has_feature(94):
            self._seq_read_u32()
        if self._has_feature(74):
            self._seq_read_bool()

        # File mode check (OLS files always use mode 1)
        file_mode = 1
        if file_mode == 1:
            if self._has_feature(123):
                self._seq_read_u32()
        elif file_mode == 3:
            if self._has_feature(221):
                self._seq_read_u32()

        if self._has_feature(300):
            for _ in range(6):
                self._seq_read_f64()
        if self._has_feature(67):
            if self._has_feature(300):
                for _ in range(6):
                    self._seq_read_f64()
            else:
                for _ in range(6):
                    self._seq_read_f64()
        elif self._has_feature(66):
            self._seq_read_f64()
            self._seq_read_f64()
        elif not self._has_feature(59):
            self._seq_read_u32()
            self._seq_read_u32()

        for _ in range(4):
            self._seq_read_bool()

        dim1 = self._seq_read_u32()
        dim2 = self._seq_read_u32()
        self._seq_read_u32()
        self._seq_read_u32()
        self._seq_read_u32()

        # Axis unit descriptor
        unit_str, factor, conv_offset, data_offset, axis_offset = self._seq_read_axis_unit_desc()

        # Two axis data blocks
        axis1 = self._seq_read_axis_data()
        axis2 = self._seq_read_axis_data()

        if dim1 > 0x4000:
            dim1 = 0x4000

        if self._has_feature(58):
            self._seq_read_bool()
        if self._has_feature(68):
            self._seq_read_bool()
        if self._has_feature(90):
            self._seq_read_u32()
        if self._has_feature(9):
            self._seq_read_u32()
            self._seq_read_u32()
        if self._has_feature(49):
            self._seq_read_u32()
            self._seq_read_bool()
            self._seq_read_bool()
            self._seq_read_f64()
            self._seq_read_f64()
        if self._has_feature(51):
            self._seq_read_u32()
        if self._has_feature(53):
            self._seq_read_bool()
            self._seq_read_f64()
            self._seq_read_f64()
            self._seq_read_f64()
            self._seq_read_u32()
        if self._has_feature(54):
            self._seq_read_f64()
        if self._has_feature(55):
            self._seq_read_bool()
            self._seq_read_bool()
            self._seq_read_bool()
        if self._has_feature(315):
            self._seq_read_u32()
        if self._has_feature(383):
            self._seq_read_i32()
            self._seq_read_bytes(16)
        if self._has_feature(329):
            self._seq_read_u32()
        if self._has_feature(346):
            self._seq_read_u32()
            self._seq_read_u32()
        if self._has_feature(395):
            self._seq_read_u32()
        if self._has_feature(476):
            self._seq_read_u32()
            self._seq_read_u32_array()
            self._seq_read_string_array()
        if self._has_feature(503):
            self._seq_read_u32()
            self._seq_read_f64()
        if self._has_feature(596):
            self._seq_read_u32()

        # Derive parameter type
        type_map = {1: "VALUE", 2: "VALUE", 3: "CURVE", 4: "MAP", 5: "MAP"}
        param_type = type_map.get(type_code, "MAP" if type_code > 5 else "VALUE")

        # Derive dimensions
        if type_code <= 2:
            cols = max(1, dim1)
            rows = 1
        elif type_code == 3:
            cols = max(1, dim1)
            rows = 1
        else:
            cols = max(1, dim1)
            rows = max(1, dim2)
        if cols > 0x4000:
            cols = 1
        if rows > 0x4000:
            rows = 1

        # Assign axes
        x_axis = None
        y_axis = None
        if param_type == "CURVE":
            x_axis = axis1
            if x_axis:
                x_axis.points = cols
                x_axis.address = axis_offset
        elif param_type == "MAP":
            y_axis = axis1
            x_axis = axis2
            if y_axis:
                y_axis.points = rows
                y_axis.address = axis_offset
            if x_axis:
                x_axis.points = cols

        # OLS data_offset points 2 entries before actual cell data
        type_size = {2: 1, 10: 2, 16: 4}.get(data_size, 2)
        adjusted_offset = data_offset + 2 * type_size if data_offset > 0 else 0

        return Parameter(
            name=name,
            description=description,
            param_type=param_type,
            data_type=data_type,
            cols=cols,
            rows=rows,
            data_offset=adjusted_offset,
            x_axis=x_axis,
            y_axis=y_axis,
            unit=unit_str,
            factor=factor,
            offset=conv_offset,
            type_code=type_code,
            folder_id=folder_id,
        )

    # -------------------------------------------------------------------
    # Parameter extraction (sequential)
    # -------------------------------------------------------------------

    def _extract_parameters(self) -> list[Parameter]:
        """
        Extract parameter definitions by reading the map list sequentially.

        Locates the map list via the 0x11883377 checksum marker, then reads
        each map record field-by-field in the exact order WinOLS uses.
        """
        found = self._find_map_list_start()
        if found is None:
            return []

        first_map_pos, map_count = found
        self.pos = first_map_pos
        parameters = []

        for _ in range(map_count):
            try:
                p = self._seq_read_one_map()
                parameters.append(p)
            except (EOFError, ValueError, struct.error):
                break

        return parameters

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

    def _extract_folder_entries(self) -> list[FolderEntry]:
        """
        Extract folder/function entries from OLS file.

        These entries define the folder structure used to organize parameters.
        The folder index (idx) matches the folder_id in Parameter objects.

        Structure:
        - [ffffffff] Marker (4 bytes)
        - [00000000] Padding (4 bytes)
        - [idx]      Folder index (4 bytes)
        - [00000000] Padding (4 bytes)
        - [strlen]   String length (4 bytes)
        - [string]   Format: "NAME: \\"DESCRIPTION\\""

        Example:
        - __DDS_EXPA2_DEFAULT_FUNC__: "" (root folder)
        - ACQ_KNK_sensor_signal_gen: "Calibration Hint: Acquisition of knock sensor signal (gen)"
        """
        entries: list[FolderEntry] = []
        found_names: set[str] = set()

        # Find folder section - search for the ffffffff marker pattern
        # Start from 1/3 into file (folders are typically after CAL block)
        search_start = len(self.data) // 3
        marker = b'\xff\xff\xff\xff'

        pos = search_start
        while True:
            # Find next ffffffff marker
            pos = self.data.find(marker, pos)
            if pos < 0 or pos + 20 > len(self.data):
                break

            # Check structure: [marker] [pad1=0] [idx] [pad2=0] [strlen]
            pad1 = struct.unpack_from('<I', self.data, pos + 4)[0]
            idx = struct.unpack_from('<I', self.data, pos + 8)[0]
            pad2 = struct.unpack_from('<I', self.data, pos + 12)[0]
            strlen = struct.unpack_from('<I', self.data, pos + 16)[0]

            # Validate structure
            if pad1 == 0 and pad2 == 0 and 5 < strlen < 300 and idx < 2000:
                if pos + 20 + strlen <= len(self.data):
                    try:
                        s = self.data[pos + 20:pos + 20 + strlen].decode('cp1252', errors='replace')

                        # Must have the NAME: "DESC" format
                        if ': "' in s:
                            name, desc = s.split(': "', 1)
                            desc = desc.rstrip('"')

                            # Validate name: alphanumeric with underscores
                            clean_name = name.replace('_', '').replace('.', '')
                            if clean_name.isalnum() and name not in found_names:
                                found_names.add(name)
                                entries.append(FolderEntry(
                                    idx=idx,
                                    name=name,
                                    description=desc,
                                ))
                                # Skip past this entry
                                pos = pos + 20 + strlen
                                continue
                    except (UnicodeDecodeError, ValueError):
                        pass

            pos += 1

        return entries

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
        print("Usage: python ols_reader.py <file.ols|file.kp>")
        sys.exit(1)

    ols = read_ols(sys.argv[1])

    print(f"File: {ols.filename}")
    print(f"OLS Version: {ols.version}")
    if ols.is_kp_file:
        print("Type: KP (Kennfeld Pack / Map Pack)")
        print(f"Data format: {'Big-endian' if ols.big_endian_data else 'Little-endian'}")
    else:
        print("Type: OLS (Project file)")
    print(f"Vehicle: {ols.make} {ols.model} {ols.engine} ({ols.year})")
    if ols.ecu_type:
        ecu_info = ols.ecu_type
        if ols.ecu_manufacturer:
            ecu_info = f"{ols.ecu_manufacturer} {ecu_info}"
        if ols.sw_number:
            ecu_info += f", SW: {ols.sw_number}"
        print(f"ECU: {ecu_info}")
    print(f"Parameters: {len(ols.parameters)}")

    # Count by type
    type_counts: dict[str, int] = {}
    for p in ols.parameters:
        type_counts[p.param_type] = type_counts.get(p.param_type, 0) + 1
    if type_counts:
        type_str = ", ".join(f"{t}: {c}" for t, c in sorted(type_counts.items()))
        print(f"  By type: {type_str}")

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

    if ols.folders:
        print(f"\nFolders ({len(ols.folders)}):")
        # Create lookup for folder names
        folder_lookup = {f.idx: f for f in ols.folders}
        for f in ols.folders[:10]:
            desc_str = f": \"{f.description[:40]}\"" if f.description else ""
            print(f"  [{f.idx:4d}] {f.name}{desc_str}")
        if len(ols.folders) > 10:
            print(f"  ... and {len(ols.folders) - 10} more")

    # Create folder lookup for parameter display
    folder_lookup = {f.idx: f.name for f in ols.folders} if ols.folders else {}

    # Show sample parameters with offsets, data type, factor, and folder names
    print("\nSample parameters:")
    for p in ols.parameters[:10]:
        axis_info = ""
        if p.x_axis:
            axis_info = f" [xaxis@{hex(p.x_axis.offset)}]"
        folder_name = folder_lookup.get(p.folder_id, "")
        folder_str = f" [{folder_name}]" if folder_name else (f" [folder={p.folder_id}]" if p.folder_id else "")
        factor_str = f" *{p.factor}" if p.factor != 1.0 else ""
        unit_str = f" {p.unit}" if p.unit else ""
        print(f"  {p.param_type:5} {p.data_type:5} {p.name}: @{hex(p.data_offset)}{factor_str}{unit_str}{axis_info}{folder_str}")
