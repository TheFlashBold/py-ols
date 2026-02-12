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
    - v804:     WinOLS 5 format (15 metadata strings, derived version records)

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

    Supports OLS format versions 250 through 804+, including:
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

    def _read_axis_string(self, pos: int) -> tuple[str, int]:
        """Read a length-prefixed string at position, return (string, end_position)"""
        if pos + 4 > len(self.data):
            return "", pos
        strlen = self._read_u32(pos)
        if strlen == 0 or strlen > 200:
            return "", pos + 4
        end = pos + 4 + strlen
        if end > len(self.data):
            return "", pos + 4
        s = self.data[pos + 4:end].decode('cp1252', errors='replace').rstrip('\x00')
        return s, end

    def _parse_parameter_record(self, name_pos: int, name_len: int, name: str) -> Optional[Parameter]:
        """
        Parse a single parameter record.

        Structure varies by OLS version:

        Older format (version < 300):
        - Dimensions at +62/+66 as u16
        - Flag 0x8000 around +106, followed by 16-bit offsets

        Newer format (version >= 400):
        - Dimensions at +117 (cols) and +121 (rows) as single bytes
        - Factor at +146 (8 bytes double)
        - Marker 0x80 at +161, data offset at +162
        - Y-axis block at +202: ID, data_source, unit, factor, offset, address
        - X-axis block at +328: description, unit, factor, address, ID
        """
        name_end = name_pos + 4 + name_len

        if name_end + 180 > len(self.data):
            return Parameter(name=name)

        # Get type_code from metadata
        meta_start = name_pos - 24
        type_code = self._read_u32(meta_start) if meta_start >= 0 else 0

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

        # Determine parameter type from type_code or dimensions
        # type_code: 2=VALUE, 3=CURVE, 4=MAP, 5=MAP_INVERSE
        if type_code == 5:
            param_type = "MAP_INVERSE"
        elif type_code == 4 or (cols > 1 and rows > 1):
            param_type = "MAP"
        elif type_code == 3 or cols > 1 or rows > 1:
            param_type = "CURVE"
        else:
            param_type = "VALUE"

        # Read data type indicator and determine data_type string
        # Data type indicator: 0/1=UBYTE, 2=SBYTE, 3=UWORD, 4=SWORD, 5=ULONG, 6=SLONG
        dtype_map = {0: "UBYTE", 1: "UBYTE", 2: "SBYTE", 3: "UWORD", 4: "SWORD", 5: "ULONG", 6: "SLONG"}
        if self.version < self.NEW_FORMAT_VERSION:
            dtype_ind = self.data[name_end + 78] if name_end + 78 < len(self.data) else 3
        else:
            dtype_ind = self.data[name_end + 133] if name_end + 133 < len(self.data) else 3
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
            # New format (>= 400): unit at +145, factor at +146
            unit_len = self.data[name_end + 141] if name_end + 141 < len(self.data) else 0
            if 0 < unit_len < 20 and unit_len != 0xff:
                unit_bytes = self.data[name_end + 145:name_end + 145 + unit_len]
                unit = unit_bytes.decode('cp1252', errors='replace').rstrip('\x00').strip()
            # Factor is at fixed position +146
            if name_end + 154 <= len(self.data):
                factor = struct.unpack_from('<d', self.data, name_end + 146)[0]
                if not (1e-15 < abs(factor) < 1e15):
                    factor = 1.0

        # Extract offsets and axes - format varies by version
        data_offset = 0
        x_axis = None
        y_axis = None

        if self.version < self.NEW_FORMAT_VERSION:
            # Old format (< 300, e.g., v250): search for 0x80 marker pattern
            for check in range(100, 180):
                if name_end + check + 7 <= len(self.data):
                    if self.data[name_end + check] == 0x80:
                        addr1 = self._read_u16(name_end + check + 1)
                        padding = self._read_u16(name_end + check + 3)
                        addr2 = self._read_u16(name_end + check + 5)
                        if padding == 0 and 0x0100 <= addr1 <= 0xFFFF:
                            data_offset = addr1
                            if param_type in ("CURVE", "MAP", "MAP_INVERSE") and addr2 > addr1:
                                points = cols if cols > 1 else rows
                                x_axis = AxisInfo(points=points, address=addr2)
                                if param_type in ("MAP", "MAP_INVERSE") and rows > 1:
                                    y_axis = AxisInfo(points=rows, address=addr2 + cols)
                            break

            # Fallback: search for '0b 00' marker pattern
            if data_offset == 0:
                for check in range(100, 160):
                    if name_end + check + 4 <= len(self.data):
                        if self.data[name_end + check] == 0x0b and self.data[name_end + check + 1] == 0x00:
                            addr = self._read_u16(name_end + check + 2)
                            if 0x0100 <= addr <= 0xFFFF:
                                data_offset = addr
                                break

        elif self.version < self.NEWER_FORMAT_VERSION:
            # Intermediate format (300-399)
            if name_end + 172 <= len(self.data):
                data_offset = (self.data[name_end + 164] |
                              (self.data[name_end + 165] << 8) |
                              (self.data[name_end + 166] << 16))
                if param_type in ("CURVE", "MAP", "MAP_INVERSE"):
                    xaxis_offset = (self.data[name_end + 168] |
                                   (self.data[name_end + 169] << 8) |
                                   (self.data[name_end + 170] << 16))
                    x_axis = AxisInfo(points=cols, address=xaxis_offset)
                    if param_type in ("MAP", "MAP_INVERSE"):
                        y_axis = AxisInfo(points=rows, address=xaxis_offset + cols)
        else:
            # New format (>= 400): Full axis extraction
            if name_end + 500 <= len(self.data):
                # Data offset at +162 (after 0x80 marker at +161)
                data_offset = (self.data[name_end + 162] |
                              (self.data[name_end + 163] << 8) |
                              (self.data[name_end + 164] << 16))

                # Validate data offset
                if data_offset > 0x800000:
                    data_offset = 0

                # Extract axis info for CURVE/MAP
                if param_type in ("CURVE", "MAP", "MAP_INVERSE") and data_offset > 0:
                    # Y-axis block starts at +202 (first axis, for MAP it's Y-axis)
                    y_id, y_id_end = self._read_axis_string(name_end + 202)

                    if y_id and param_type in ("MAP", "MAP_INVERSE"):
                        # Y-axis metadata follows ID string
                        # Structure: 12 bytes padding, then data_source, unit, factor, offset, address
                        y_base = y_id_end
                        y_data_source = self._read_u32(y_base + 12) if y_base + 16 <= len(self.data) else 0

                        # Y-axis unit (2 bytes at +16 after ID end, e.g., "°C")
                        y_unit = ""
                        if y_base + 18 <= len(self.data):
                            y_unit_bytes = self.data[y_base + 16:y_base + 18]
                            y_unit = y_unit_bytes.decode('cp1252', errors='replace').rstrip('\x00')

                        # Y-axis factor at +18, offset at +26, address at +38
                        y_factor = 1.0
                        y_offset = 0.0
                        y_addr = 0
                        y_dtype = 3
                        if y_base + 46 <= len(self.data):
                            y_factor = struct.unpack_from('<d', self.data, y_base + 18)[0]
                            y_offset = struct.unpack_from('<d', self.data, y_base + 26)[0]
                            if not (1e-15 < abs(y_factor) < 1e15):
                                y_factor = 1.0
                            if not (-1e10 < y_offset < 1e10):
                                y_offset = 0.0
                            y_addr = self._read_u32(y_base + 38)
                            y_dtype = self._read_u32(y_base + 42)

                        y_axis = AxisInfo(
                            points=rows,
                            address=y_addr,
                            offset=y_offset,
                            unit=y_unit,
                            factor=y_factor,
                            id=y_id,
                            description=y_id,  # Y-axis uses ID as description
                            data_source=y_data_source,
                            data_type=dtype_map.get(y_dtype, "UWORD"),
                        )

                    # X-axis block at fixed positions (description at +328, etc.)
                    x_desc, x_desc_end = self._read_axis_string(name_end + 328)

                    # X-axis unit at +375
                    x_unit_len = self._read_u32(name_end + 375) if name_end + 379 <= len(self.data) else 0
                    x_unit = ""
                    if 0 < x_unit_len < 20:
                        x_unit = self.data[name_end + 379:name_end + 379 + x_unit_len].decode('cp1252', errors='replace').rstrip('\x00')

                    # X-axis factor at +382, address at +402
                    x_factor = 1.0
                    x_addr = 0
                    if name_end + 410 <= len(self.data):
                        x_factor = struct.unpack_from('<d', self.data, name_end + 382)[0]
                        if not (1e-15 < abs(x_factor) < 1e15):
                            x_factor = 1.0
                        x_addr = self._read_u32(name_end + 402)

                    # X-axis ID at +449
                    x_id, _ = self._read_axis_string(name_end + 449)

                    # For CURVE, the first axis block is actually X-axis
                    if param_type == "CURVE":
                        x_axis = AxisInfo(
                            points=cols if cols > 1 else rows,
                            address=x_addr if x_addr else (y_axis.address if y_axis else 0),
                            unit=x_unit if x_unit else (y_axis.unit if y_axis else ""),
                            factor=x_factor if x_factor != 1.0 else (y_axis.factor if y_axis else 1.0),
                            id=x_id if x_id else (y_axis.id if y_axis else ""),
                            description=x_desc if x_desc else (y_axis.description if y_axis else ""),
                        )
                        y_axis = None  # CURVE has no Y-axis
                    else:
                        # MAP/MAP_INVERSE: X-axis from second block
                        x_axis = AxisInfo(
                            points=cols,
                            address=x_addr,
                            unit=x_unit,
                            factor=x_factor,
                            id=x_id,
                            description=x_desc,
                        )

        # Extract description (search backwards from metadata)
        description = self._find_description_before(name_pos)

        # Extract folder_id from metadata[5] (for v300+ files)
        folder_id = 0
        if self.version >= self.NEW_FORMAT_VERSION:
            if meta_start >= 0:
                folder_id = self._read_u32(meta_start + 20)  # metadata[5]
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
            type_code=type_code,
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
