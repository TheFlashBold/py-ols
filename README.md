# py-ols - WinOLS File Reader

A Python library for reading WinOLS `.ols` project files and `.kp` (Kennfeld Pack) map pack files, extracting parameter definitions, CAL offsets, and embedded binary data.

## Features

- Parse OLS files from version 250 to 597+
- **KP (Kennfeld Pack) file support**: Parse map pack files with ZIP-compressed parameters
- Extract parameter definitions with offsets, types, units, and factors
- Extract binary version entries (Original, edit history)
- Extract embedded CAL (calibration) blocks
- Support for multi-version files with edit history (v597+)
- **v250 format support**: Marker-based address extraction for DSG/TCU files

## Installation

```bash
# Copy to your project or add to PYTHONPATH
cp -r py-ols /your/project/
```

## Usage

```python
from ols_reader import read_ols, OLSReader

# Quick read
ols = read_ols("path/to/file.ols")

print(f"Vehicle: {ols.make} {ols.model}")
print(f"OLS Version: {ols.version}")
print(f"Parameters: {len(ols.parameters)}")

# Access parameters with folder IDs
for param in ols.parameters:
    print(f"{param.name}: offset={hex(param.data_offset)}, folder={param.folder_id}")

# Access binary versions
for version in ols.binary_versions:
    if version.version_count > 0:
        # Multi-version file (v597+)
        print(f"{version.name} (idx={version.version_index}/{version.version_count})")
    elif version.is_root:
        print(f"{version.name} (ROOT)")
    else:
        print(f"{version.name} (parent={version.parent_id})")

# Extract binary data from a version
reader = OLSReader("path/to/file.ols")
ols = reader.parse()
for version in ols.binary_versions:
    if version.blob_offset and version.blob_size:
        binary_data = reader.extract_binary(version)
        with open(f"{version.name}.bin", "wb") as f:
            f.write(binary_data)

# Access CAL binary data (Simos-specific)
if ols.cal_block:
    print(f"EPK: {ols.cal_block.epk}")
    with open("cal.bin", "wb") as f:
        f.write(ols.cal_block.data)
```

## KP (Kennfeld Pack) File Support

KP files are map pack files used for sharing map definitions. They use ZIP compression and contain direct file offsets to binary data.

```python
from ols_reader import read_ols

# KP files are automatically detected
ols = read_ols("mappack.kp")

print(f"Is KP file: {ols.is_kp_file}")        # True
print(f"Big-endian data: {ols.big_endian_data}")  # True (common for DSG)

# Addresses are direct file offsets into the corresponding .bin file
for param in ols.parameters:
    print(f"{param.name}: @0x{param.data_offset:x}")
```

### KP File Characteristics

- **Version**: 597 (same as multi-version OLS)
- **Storage**: ZIP-compressed "intern" file containing parameter definitions
- **Addresses**: Direct file offsets (no transformation needed)
- **Data format**: Big-endian (Motorola byte order) for DSG/TCU ECUs
- **No embedded binary**: References external `.bin` files

### Reading KP Data

When reading binary data for KP parameters, use big-endian byte order:

```python
import struct
from pathlib import Path

ols = read_ols("dq250.kp")
bin_data = Path("dq250.bin").read_bytes()

for param in ols.parameters[:5]:
    addr = param.data_offset
    # KP files indicate big-endian data format
    vals = [struct.unpack_from('>H', bin_data, addr + i*2)[0] for i in range(4)]
    print(f"{param.name}: {vals}")
```

## CLI Usage

```bash
python ols_reader.py file.ols
# or
python ols_reader.py file.kp
```

Example output for OLS file:
```
File: DESKTOP-9T8M570_10009.ols
OLS Version: 597
Type: OLS (Project file)
Vehicle: Audi A3 1.8TFSI (2014)
Parameters: 22831

Binary versions (1):
  Original (idx=10/11) (4096 KB) @0x4000000

CAL: SC1CF00 (2051.0 KB)

Sample parameters:
  CURVE UWORD ip_crlc_vp_bpa_ad_dif: @0x41262 *1.53e-05 [xaxis@0x4126e] folder=3
  VALUE UWORD c_crlc_bpa_ad: @0x41357 *0.00390625 folder=3
```

Example output for KP file:
```
File: dq250_mappack.kp
OLS Version: 597
Type: KP (Kennfeld Pack / Map Pack)
Data format: Big-endian
Vehicle:    ()
Parameters: 603
  By type: CURVE: 71, MAP: 37, VALUE: 495

Sample parameters:
  CURVE UWORD HdrPHauptMax_kl: @0x69416 [xaxis@0x6941e]
  MAP   UWORD Ges_NAnHS12Eco_kf: @0x698e4 [xaxis@0x69944]
```

---

# OLS File Format Documentation

WinOLS `.ols` files are proprietary project files used by WinOLS ECU tuning software. This document describes the file structure based on reverse engineering.

## Overview

An OLS file contains:
1. **Header** - Magic number, signature, and vehicle metadata
2. **Parameter Definitions** - Names, descriptions, addresses, and axis info
3. **Display Settings** - UI configuration for maps/tables
4. **Embedded CAL Data** - Optional binary calibration data

## File Header

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| 0x00 | 4 | u32 | Magic number (0x0000000B) |
| 0x04 | 12 | string | Signature "WinOLS File" |
| 0x10 | 2 | u16 | Format version (e.g., 250, 440, 478, 597) |
| 0x12 | 6 | - | Flags/timestamp |
| 0x18 | var | strings | Vehicle metadata (see below) |

### Vehicle Metadata Strings (starting at 0x18)

All strings are length-prefixed (4-byte length + UTF-8 data). Empty strings have length=0.

| Index | Description | Example |
|-------|-------------|---------|
| 0 | Make | "VW", "Audi", "Seat" |
| 1 | Model | "Jetta GLI", "Golf GTI" |
| 2 | Engine code | "2.0 TSI", "CJXB", "CPPA" |
| 3 | Year | "2014" |
| 4 | Fuel type | "Turbo-Petrol", "Diesel" |
| 5 | Displacement | "2.0" |
| 6 | Power | "210.1PS / 154.5KW" |
| 7 | Transmission | "Automatic transmission", "DSG" |
| 8 | Memory type | "Eprom" |
| 9 | ECU manufacturer | "Siemens/Continental", "Temic" |
| 10 | ECU version | "Simos12.2", "Simos18.1" |
| 11 | ECU part number | "06K906070C", "5G0906259" |
| 12 | (reserved) | Usually empty |
| 13 | CAL/SW ID | "SC200E5500000", "SC8F830" |

### Project Info Section (after vehicle metadata)

Following the vehicle metadata strings:

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| var | 4 | u32 | Timestamp (Unix epoch) |
| var+4 | 4 | u32 | Secondary timestamp |
| var+8 | 4 | u32 | Project filename length |
| var+12 | var | string | Project filename (e.g., "VW Golf GTI.ols") |
| var | 4 | u32 | OLS version string length |
| var+4 | var | string | OLS version (e.g., "OLS 5.0 (Windows)") |

### Binary Version Metadata (after project info)

Immediately following the project info section is the binary version metadata:

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| var | 4 | - | Null padding |
| var+4 | 4 | u32 | Version label length |
| var+8 | var | string | Version label (e.g., "0 (Original)") |
| var | - | - | Null padding |
| var | 4 | u32 | Size string length |
| var+4 | var | string | Size/ID string (e.g., "150000") |
| var | 4 | u32 | Flag (typically 1) |
| var+4 | 8 | bytes | Hash/checksum |
| var+12 | 4 | u32 | Flag (typically 1) |
| var+16 | 4 | u32 | Description length |
| var+20 | var | string | Import description (e.g., "Imported from file:...") |

This section contains metadata about the binary versions stored in the file, including import history and checksums.

### Additional Metadata Entries

Following the binary version metadata are additional structured entries:

- Short identifier strings (e.g., "0: ", "0 ()")
- Part numbers (e.g., "0CW300048S")
- Duplicate vehicle info (make, model)
- CAL/SW IDs

### Folder Name Definitions

Category/folder names appear later in the file as length-prefixed strings. Common patterns:

- **Category names**: "Gearbox", "Gear", "Injection", "Torque"
- **Data format types**: "Intel-Hex", "Eprom"
- **Version names**: "Original", "NOCS", "Hexdump"

These folder names correspond to the `folder_id` values in parameter definitions.

### Length-Prefixed Strings

Strings are stored as:
- 4 bytes: length (u32, little-endian)
- N bytes: UTF-8 string data (may contain null terminator)

## Parameter Definitions

Parameters are stored as records throughout the file. Each parameter has:

### Record Structure

```
[Description String]     <- Length-prefixed, search backwards to find
[24 bytes metadata]      <- 6 x u32 values (see below)
[Name String]            <- Length-prefixed
[Post-Name Structure]    <- ~200 bytes of metadata
```

### Parameter Metadata (24 bytes before name)

The 24 bytes before the name length field contain 6 u32 values:

| Index | Offset | Description |
|-------|--------|-------------|
| 0 | -24 | Type indicator (2=VALUE, etc.) |
| 1 | -20 | Type indicator 2 |
| 2 | -16 | Flags |
| 3 | -12 | Flags |
| 4 | -8 | Value (typically 10) |
| 5 | -4 | **Folder ID** (category) |

The folder ID groups parameters into logical categories within WinOLS. Parameters with the same folder ID belong to the same folder/category in the UI.

### Post-Name Structure

The structure varies between OLS format versions. The OLS version is stored at offset 0x10 (u16).

#### OLS Version Ranges

| Version | Format | ECU Types | Notes |
|---------|--------|-----------|-------|
| 250 | Old format (marker-based) | DSG/TCU (DQ200, DQ250, DQ381) | Uses `0b 00` marker for addresses |
| 285-299 | Old format (flag-based) | Simos 10.x, 12.x | Uses `0x80` flag for addresses |
| 300-399 | Intermediate | Various | 24-bit offsets at fixed positions |
| >= 400 | New format | Simos 18.x, 19.x, 22.x | 24-bit offsets, extended metadata |

#### Old Format (version < 300)

##### v250 Format (DSG/TCU files)

The v250 format uses a marker-based approach for address extraction. The `0b 00 [addr]` pattern appears consistently after each parameter's post-name structure:

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| +62 | 2 | u16 | **Columns** |
| +66 | 2 | u16 | **Rows** |
| +100-105 | var | - | Float data (factor, etc.) |
| +105 | 1 | u8 | Flag byte (0x80 for CURVE/MAP) |
| +106-107 | 2 | u16 | Primary address (if 0x80 flag present) |
| **+118-119** | 2 | u16 | **`0b 00` marker** |
| **+120-121** | 2 | u16 | **Data offset** (reliable for all types) |

The primary extraction method for v250:
1. Scan for `0b 00` marker followed by valid address (0x0100-0xFFFF)
2. Search backwards 110-200 bytes to find the parameter name
3. For CURVE/MAP, also check for `0x80 [addr] 00 00 [addr+delta]` pattern for axis info

##### v285+ Format (Simos ECU files)

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| +0 | 18 | - | Zeros/flags |
| +18 | 1 | u8 | UBYTE raw max (0xFF) |
| +34 | 2 | u16 | UWORD raw max (0xFFFF) |
| +50 | 4 | u32 | ULONG raw max (0xFFFFFFFF) |
| +62 | 2 | u16 | **Columns** (number of X-axis points) |
| +66 | 2 | u16 | **Rows** (number of Y-axis points) |
| +78 | 1 | u8 | **Data type indicator** (see below) |
| +86 | 1 | u8 | **Unit length** |
| +90 | var | str | **Unit** (e.g., "rpm", "s", "kg") |
| +92+ | 8 | f64 | **Factor** (conversion: physical = raw * factor) |
| ~+107 | 1 | u8 | Flag (0x80) |
| +108 | 2-3 | u16/u24 | **Data offset** (CAL binary offset) |
| +112 | 2-3 | u16/u24 | **Axis offset** (for CURVE/MAP) |
| +118 | 2 | u16 | **Stride** (bytes per element, e.g., 2 for UWORD) |

#### Intermediate Format (version 300-399)

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| +0 | ~60 | - | Extended metadata/flags |
| +118 | 1 | u8 | **Columns** (single byte) |
| +122 | 1 | u8 | **Rows** (single byte) |
| +163 | 1 | u8 | Flag (0x80) |
| +164-166 | 3 | u24 | **Data offset** (24-bit little-endian) |
| +168-170 | 3 | u24 | **Axis offset** (24-bit little-endian) |

#### New Format (version >= 400)

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| +0 | 18 | - | Zeros/flags |
| +25 | 8 | f64 | UBYTE display max (255.0) |
| +41 | 8 | f64 | UWORD display max (65535.0) |
| +57 | 8 | f64 | ULONG display max (4294967295.0) |
| +73 | 1 | u8 | UBYTE raw max (0xFF) |
| +89 | 2 | u16 | UWORD raw max (0xFFFF) |
| +105 | 4 | u32 | ULONG raw max (0xFFFFFFFF) |
| +117 | 1 | u8 | **Columns** (single byte) |
| +121 | 1 | u8 | **Rows** (single byte) |
| +133 | 1 | u8 | **Data type indicator** (see below) |
| +141 | 1 | u8 | **Unit length** |
| +145 | var | str | **Unit** (e.g., "rpm", "s", "kg") |
| +145+len | 8 | f64 | **Factor** (conversion: physical = raw * factor) |
| +162 | 1 | u8 | Flag (0x80) |
| +163-165 | 3 | u24 | **Data offset** (24-bit little-endian) |
| +167-169 | 3 | u24 | **Axis offset** (24-bit little-endian) |

### Parameter Types

Determined by dimensions:
- **VALUE**: cols=1, rows=1 (scalar)
- **CURVE**: cols>1, rows=1 OR cols=1, rows>1 (1D table)
- **MAP**: cols>1, rows>1 (2D table)

### Data Type Indicators

The data type indicator at +78 (old format) or +133 (new format) encodes the binary data type:

| Value | Type | Size | Description |
|-------|------|------|-------------|
| 0, 1 | UBYTE | 1 | Unsigned 8-bit |
| 2 | SBYTE | 1 | Signed 8-bit |
| 3 | UWORD | 2 | Unsigned 16-bit (most common) |
| 4 | SWORD | 2 | Signed 16-bit |
| 5 | ULONG | 4 | Unsigned 32-bit |
| 6 | SLONG | 4 | Signed 32-bit |

**Note**: Byte order is always little-endian for Simos/Continental ECUs.

### CAL Offset Encoding

The offset encoding differs based on parameter type:

#### VALUE and CURVE (16-bit offsets)

For parameters where the CAL area fits in 64KB:

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| +106 | 2 | u16 | Flag (0x8000) |
| +108 | 2 | u16 | **Data offset** |
| +110 | 2 | u16 | Zero padding |
| +112 | 2 | u16 | **Axis offset** (for CURVE) |

#### MAP (24-bit offsets)

For larger CAL areas exceeding 64KB:

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| +108 | 2 | u16 | Flag (0x8000) |
| +110 | 2 | u16 | Data offset (low 16 bits) |
| +112 | 1 | u8 | Data offset (high 8 bits) |
| +113 | 1 | u8 | Flag |
| +114 | 2 | u16 | X-axis offset (low 16 bits) |
| +116 | 1 | u8 | X-axis offset (high 8 bits) |

To reconstruct 24-bit offset:
```python
data_lo = read_u16(name_end + 110)
data_hi = data[name_end + 112]
data_offset = data_lo | (data_hi << 16)
```

### Data Layout in CAL

For a MAP with cols=8, rows=6:

```
data_offset:       [48 bytes of table data, row-major]
xaxis_offset:      [8 bytes of X-axis values]
yaxis_offset:      [6 bytes of Y-axis values]
```

The axis offset typically follows immediately after data:
- X-axis at: `data_offset + (cols * rows)`
- Y-axis at: `xaxis_offset + cols`

## Parameter Folders/Categories

Parameters in WinOLS can be organized into folders (categories) for logical grouping. Each parameter has a `folder_id` field in its metadata.

### Folder IDs

| Field | Location | Description |
|-------|----------|-------------|
| folder_id | metadata[5] (@-4 from name) | Numeric folder/category ID |

- **Folder IDs** are numeric identifiers (0-1000+)
- Parameters with the same `folder_id` belong to the same logical group
- Folder ID 0 typically indicates unassigned/root-level parameters
- Files can have hundreds of unique folder IDs

### Folder Names

**Status: Unknown location**

Folder NAMES (the user-visible category labels like "Injection", "Torque", etc.) were not found in the OLS files examined. Possible explanations:

1. Names stored in separate WinOLS configuration/database
2. Names generated dynamically from parameter prefixes
3. Names only stored when explicitly set by user
4. Names stored in a format not yet identified

The folder ID to name mapping may be maintained externally by WinOLS.

### Binary Version Folders

Binary version folders (different from parameter folders) ARE stored in the OLS file with the `0x003fffff` marker:

```
[0x003fffff marker]     <- Root folder indicator
[length u32]            <- Name length
[name]                  <- Folder name (e.g., "Daten")
[child_count byte]      <- Number of child IDs
[0xffffff padding]
[child IDs as u16[]]    <- List of child folder IDs
```

**Note**: Folder names for non-root folders (the actual category names like "Injection", "Torque") may not be explicitly stored in the OLS file. The folder IDs reference internal WinOLS categories.

## Binary Versions

WinOLS supports multiple binary versions within a single project file. Each version represents a different variant of the ECU data (e.g., "Original", "Modified", "Stage 1").

### OLS Format Versions

| Version Range | Binary Version Format | Description |
|---------------|----------------------|-------------|
| < 300 | None detected | Oldest format, binary versions not stored |
| 300-399 | Parent ID format | Single version with parent reference |
| 400-596 | Parent ID format | Standard format with blob offset |
| 597+ | Multi-version format | Edit history with version slots |

### Binary Version Types

1. **Root Folders** (Daten, Daten 2, Hexdump): Identified by `0x003fffff` marker
2. **Child Versions** (Original, NOCS): Have parent ID pointing to root
3. **Edit History Versions** (v597+): Multiple version slots for undo/edit history

### Standard Format (v300-596)

#### Root Folder Entry (0x400-0x600)

```
[0x003fffff]            <- Root marker
[length u32]            <- Name length
[name]                  <- e.g., "Daten", "Daten 2"
[child_count byte]      <- Number of children (e.g., 56)
[0xffffff padding]
[child IDs u16[]]       <- List of child folder/version IDs
```

#### Child Version Entry

Child versions contain references to binary blob data stored in the OLS file:

```
[padding 8 bytes]       <- Zeros
[blob_index u32]        <- Index/sequence number of blob
[blob_offset u32]       <- File offset where binary blob starts
[blob_size u32]         <- Binary blob size (e.g., 0x00480000 = 4.5MB)
[timestamp u32]         <- Modification timestamp
[parent_id u32]         <- Parent folder ID (e.g., 1)
[name_length u32]       <- Name length
[name]                  <- e.g., "Original", "NOCS"
[desc_length u32]       <- Description length
[description]           <- e.g., "Importiert aus Datei C:\Users\..."
```

### Multi-Version Format (v597+)

Version 597+ uses a different structure that supports edit history with multiple version slots.

#### Version Entry Structure

```
[zero u32]              <- Always 0
[version_index u32]     <- Index of this version (0-based)
[metadata_offset u32]   <- File offset to version metadata area
[blob_size u32]         <- Size of each blob slot (always 0x400000 = 4MB)
[timestamp u32]         <- Modification timestamp
[version_count u32]     <- Total number of version slots
[name_length u32]       <- Name length
[name]                  <- e.g., "Original"
[desc_length u32]       <- Description length
[description]           <- e.g., "Importiert aus Datei..."
```

#### Version Slot Calculation

In v597+, binary blobs are stored at intervals based on the original binary size:

```
blob_base = aligned_offset after metadata area
actual_blob_offset = blob_base + (version_index * blob_size)
```

The `blob_size` is determined by the original imported binary (e.g., 4MB for typical Simos ECU flash).

Example with 11 version slots (4MB binary):
```
Version 0:  0x1800000  (oldest edit state)
Version 1:  0x1C00000
Version 2:  0x2000000
...
Version 10: 0x4000000  (current "Original" state)
```

#### Edit History

The version slots represent edit history snapshots:
- Each edit operation creates a new version slot with a copy of the binary
- Version names like "V2.11.1", "V2.11.2", "Tune" are stored in a separate version list
- The `version_index` points to the active/current version slot
- `version_count` indicates total history depth (number of undo levels available)

### Binary Blob Storage

Binary blobs (ECU firmware data) are stored contiguously in the OLS file:

- **blob_offset**: Absolute file offset where the blob data starts
- **blob_size**: Size in bytes (typically 4-4.5MB for ECU binaries)
- Blobs contain raw firmware data
- v597+ stores multiple copies for edit history

### Common Version Names

- **"Daten"**, **"Daten 2"** - Root data folders
- **"Original"** - The original/current binary
- **"NOCS"** - No checksum verification version
- **"Tune"**, **"V2.11.x"** - User-created edit versions (v597+)

## Example Parameter Records

### VALUE Parameter (scalar)

```
Name: c_ctr_cyc_st_tran_is
Cols: 1, Rows: 1
Data offset (16-bit): 0x52B8 at +108
No axis data
```

### CURVE Parameter (1D table)

```
Name: ip_ctr_cyc_st
Cols: 8, Rows: 1
Data offset (16-bit): 0x527C at +108
X-axis offset: 0x528C at +112
```

### MAP Parameter (2D table)

```
Name: id_n_accout_min_puc
Cols: 8, Rows: 6
Data offset (24-bit): 0x1C808 at +110/+112
X-axis offset (24-bit): 0x1C838 at +114/+116
```

## KP (Kennfeld Pack) File Format

KP files are a variant of OLS files designed for sharing map definitions. They share the same header as OLS files but store parameter data differently.

### File Structure

```
[OLS Header]                <- Same as standard OLS (magic, signature, version)
[Minimal metadata]          <- Limited vehicle info
[ZIP archive]               <- Starting around offset 0x400-0x500
  └── intern                <- Compressed parameter definitions
```

### Detection

A file is identified as KP if:
1. OLS version >= 597
2. ZIP signature (`PK\x03\x04`) found after header
3. ZIP contains "intern" file

### Intern File Structure

The decompressed "intern" file contains parameter records:

```
[Parameter Record 1]
  [6 x u32 metadata]        <- Type code, dimensions, etc.
  [u32 name_length]
  [name string]
  [~0xa0 bytes post-name]   <- Contains addresses
  [u32 data_offset]         <- Direct file offset (at ~0x9c-0xb0 from name end)
  [u32 axis_offset]         <- Optional, for CURVE/MAP

[Parameter Record 2]
  ...
```

### Address Format

| Offset from name_end | Size | Description |
|---------------------|------|-------------|
| +0x9c to +0xb0 | 4 | Data offset (scan for valid address) |
| +0xa0 to +0xb4 | 4 | Axis offset (4 bytes after data offset) |

Addresses are:
- **Direct file offsets** into the corresponding `.bin` file
- Typically in range 0x60000-0x80000 for DSG/TCU ECUs
- No transformation needed (unlike some OLS formats)

### Data Byte Order

KP files for DSG/TCU ECUs use **big-endian** (Motorola) byte order, unlike Simos ECUs which use little-endian:

```python
# For KP files (DSG/TCU):
value = struct.unpack_from('>H', bin_data, offset)[0]  # Big-endian

# For OLS files (Simos):
value = struct.unpack_from('<H', bin_data, offset)[0]  # Little-endian
```

### Parameter Types in KP

Type codes in metadata[0]:
- **2**: VALUE (scalar)
- **3**: CURVE (1D table)
- **4+**: MAP (2D table)

Dimensions from metadata[4] and metadata[5] for MAPs.

## References

- WinOLS: https://www.evc.de/en/product/ols/software/
