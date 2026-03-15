# py-ols

Python library for reading WinOLS `.ols` and `.kp` (Kennfeld Pack) files.

Extracts parameter definitions, CAL offsets, conversion factors, axis data, and embedded binaries.

## Supported Versions

- v100-285: Old format (marker-based addresses)
- v300-399: Intermediate format (24-bit offsets)
- v400-596: Standard format (full axis extraction)
- v597+: Multi-version format (edit history)
- v597+ KP: Map pack files (ZIP-compressed)
- v804+: WinOLS 5 format

## Usage

```python
from ols_reader import read_ols, OLSReader

ols = read_ols("file.ols")
print(f"{ols.make} {ols.model} — {len(ols.parameters)} params")

for p in ols.parameters:
    print(f"{p.name}: @0x{p.data_offset:x} {p.param_type} {p.cols}x{p.rows} "
          f"factor={p.factor} unit={p.unit}")

# Extract binary data
reader = OLSReader("file.ols")
ols = reader.parse()
for v in ols.binary_versions:
    data = reader.extract_binary(v)
    open(f"{v.name}.bin", "wb").write(data)
```

## CLI

```bash
python ols_reader.py file.ols
```

## Data Classes

- **`OLSFile`** — Parsed file: vehicle metadata, parameters, binary versions, CAL block, folders
- **`Parameter`** — Name, description, type (VALUE/CURVE/MAP), data_type, dims, offset, factor, unit, axes
- **`AxisInfo`** — Points, address, factor, offset, unit, id, description, data_source
- **`BinaryVersion`** — Name, blob_offset, blob_size, EPK, version_index
- **`CALBlock`** — CAS id, EPK, raw data
- **`FolderEntry`** — Folder index, name, description
