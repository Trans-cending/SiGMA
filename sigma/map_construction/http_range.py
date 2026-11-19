"""Seekable HTTP reader for large public files with byte-range support."""

from __future__ import annotations

import io
import re
import struct
import urllib.request
import zipfile
import zlib
from pathlib import Path
from typing import Optional


_CONTENT_RANGE_RE = re.compile(r"bytes\s+(\d+)-(\d+)/(\d+)")


class HTTPRangeReader(io.RawIOBase):
    """Expose an HTTP resource as a read-only seekable binary stream."""

    def __init__(self, url: str, *, timeout: float = 120.0) -> None:
        super().__init__()
        self.url = str(url)
        self.timeout = float(timeout)
        self._position = 0
        request = urllib.request.Request(self.url, method="HEAD")
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            self.url = response.geturl()
            length = response.headers.get("Content-Length")
            accepts_ranges = response.headers.get("Accept-Ranges", "")
        if length is None:
            raise OSError(f"HTTP resource has no Content-Length: {self.url}")
        if "bytes" not in accepts_ranges.lower():
            raise OSError(f"HTTP resource does not advertise byte ranges: {self.url}")
        self._size = int(length)

    @property
    def size(self) -> int:
        return self._size

    def readable(self) -> bool:
        return True

    def seekable(self) -> bool:
        return True

    def tell(self) -> int:
        return self._position

    def seek(self, offset: int, whence: int = io.SEEK_SET) -> int:
        if whence == io.SEEK_SET:
            position = int(offset)
        elif whence == io.SEEK_CUR:
            position = self._position + int(offset)
        elif whence == io.SEEK_END:
            position = self._size + int(offset)
        else:
            raise ValueError(f"Unsupported whence: {whence}")
        if position < 0:
            raise OSError("Negative seek position")
        self._position = min(position, self._size)
        return self._position

    def read(self, size: Optional[int] = -1) -> bytes:
        if self._position >= self._size:
            return b""
        if size is None or size < 0:
            end = self._size - 1
        else:
            if size == 0:
                return b""
            end = min(self._size - 1, self._position + int(size) - 1)

        start = self._position
        request = urllib.request.Request(
            self.url,
            headers={
                "Range": f"bytes={start}-{end}",
                "User-Agent": "mosaic-digital-twin-release/1.0",
            },
        )
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            content_range = response.headers.get("Content-Range", "")
            match = _CONTENT_RANGE_RE.fullmatch(content_range.strip())
            if response.status != 206 or match is None:
                raise OSError(
                    "Server ignored byte range "
                    f"{start}-{end} for {self.url}"
                )
            actual_start, actual_end, total = map(int, match.groups())
            if (actual_start, actual_end, total) != (start, end, self._size):
                raise OSError(
                    "Unexpected Content-Range "
                    f"{content_range!r} for {self.url}"
                )
            data = response.read()
        expected = end - start + 1
        if len(data) != expected:
            raise OSError(
                f"Short HTTP range read: expected {expected}, got {len(data)}"
            )
        self._position = end + 1
        return data

    def readinto(self, buffer: bytearray) -> int:
        data = self.read(len(buffer))
        buffer[: len(data)] = data
        return len(data)


def extract_remote_zip_member(
    url: str,
    member_name: str,
    output: Path,
    *,
    timeout: float = 300.0,
) -> zipfile.ZipInfo:
    """Extract one member with two range reads, regardless of archive size."""
    with HTTPRangeReader(url, timeout=timeout) as remote:
        with zipfile.ZipFile(remote) as archive:
            info = archive.getinfo(member_name)

        remote.seek(info.header_offset)
        header = remote.read(zipfile.sizeFileHeader)
        fields = struct.unpack(zipfile.structFileHeader, header)
        if fields[zipfile._FH_SIGNATURE] != zipfile.stringFileHeader:
            raise zipfile.BadZipFile("Invalid local file header")
        name_size = fields[zipfile._FH_FILENAME_LENGTH]
        extra_size = fields[zipfile._FH_EXTRA_FIELD_LENGTH]
        remote.seek(name_size + extra_size, io.SEEK_CUR)
        compressed = remote.read(info.compress_size)

    if info.compress_type == zipfile.ZIP_STORED:
        payload = compressed
    elif info.compress_type == zipfile.ZIP_DEFLATED:
        payload = zlib.decompress(compressed, -zlib.MAX_WBITS)
    else:
        raise NotImplementedError(
            f"Unsupported ZIP compression method: {info.compress_type}"
        )
    if len(payload) != info.file_size:
        raise zipfile.BadZipFile(
            f"Size mismatch for {member_name}: "
            f"expected {info.file_size}, got {len(payload)}"
        )
    if (zlib.crc32(payload) & 0xFFFFFFFF) != info.CRC:
        raise zipfile.BadZipFile(f"CRC mismatch for {member_name}")

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(payload)
    return info
