# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Self-contained MAVLink v2 codec for the OmniSim hardware-in-the-loop lane.

Stdlib only, deliberately: the interpreter that runs an OmniSim controller is
not guaranteed to have pymavlink, and the HIL loop has to keep working there.
The frames this produces are wire-compatible with real PX4 and ArduPilot, so a
physical autopilot can be swapped in for the simulated one without touching the
transport.

Scope is exactly the HIL message set -- five encoded (sim -> autopilot) and
three decoded (autopilot -> sim). Anything else on the wire is skipped by
framing length rather than parsed, so an unknown message never desyncs the
stream.

The wire rules that are easy to get wrong, and are therefore pinned by tests:

  * Field order is NOT declaration order. MAVLink sorts wire fields by
    descending native element size (8-byte first, then 4, 2, 1), ties broken by
    declaration order.
  * `v2` extension fields are appended after the sorted block, in declaration
    order, and are never reordered -- HIL_GPS ends `... uint8_t id, uint16_t
    yaw`, a 2-byte field after a 1-byte one, which no sort would produce.
  * CRC_EXTRA covers only the non-extension fields. That exclusion is what lets
    a message gain an extension without breaking older peers.
  * v2 trims trailing zero bytes off the payload, and the receiver zero-pads
    back to full length before unpacking.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

__all__ = [
    "MavlinkCodec",
    "DecodedMessage",
    "MavlinkError",
    "crc16_mcrf4xx",
    "MESSAGES_BY_NAME",
    "MESSAGES_BY_ID",
    "CRC_EXTRA",
    "STX_V2",
]


class MavlinkError(ValueError):
    """Raised for a caller mistake: unknown message, unknown field, bad value."""


STX_V2 = 0xFD
HEADER_LEN = 10  # STX, len, incompat, compat, seq, sysid, compid, msgid[3]
CHECKSUM_LEN = 2
SIGNATURE_LEN = 13
IFLAG_SIGNED = 0x01
MAX_PAYLOAD = 255
MAX_FRAME = HEADER_LEN + MAX_PAYLOAD + CHECKSUM_LEN + SIGNATURE_LEN


def crc16_mcrf4xx(data: Iterable[int], crc: int = 0xFFFF) -> int:
    """CRC-16/MCRF4XX, the MAVLink checksum. Init 0xFFFF, no final xor.

    Call it repeatedly, threading `crc` through, to accumulate over several
    byte runs -- that is how CRC_EXTRA is folded in after the payload.
    """
    for b in data:
        tmp = (b ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


# struct code and native element size per MAVLink type name. The element size
# is the sort key -- float[16] sorts as 4, not 64.
_TYPES: Dict[str, Tuple[str, int]] = {
    "char": ("c", 1),
    "int8_t": ("b", 1),
    "uint8_t": ("B", 1),
    "int16_t": ("h", 2),
    "uint16_t": ("H", 2),
    "int32_t": ("i", 4),
    "uint32_t": ("I", 4),
    "int64_t": ("q", 8),
    "uint64_t": ("Q", 8),
    "float": ("f", 4),
    "double": ("d", 8),
}

# Field names the encoder reserves for frame-level overrides. Guarded at import
# so a future message carrying one of these names fails loudly instead of
# silently shadowing the override.
_RESERVED_KWARGS = ("seq", "sysid", "compid")


@dataclass(frozen=True)
class _Field:
    type: str
    name: str
    array_length: int = 0

    @property
    def elem_size(self) -> int:
        return _TYPES[self.type][1]

    @property
    def wire_length(self) -> int:
        return self.elem_size * (self.array_length or 1)

    @property
    def fmt(self) -> str:
        code = _TYPES[self.type][0]
        return "%d%s" % (self.array_length, code) if self.array_length else code


class _MessageDef:
    """One message's wire layout, derived from its declaration order."""

    def __init__(
        self,
        msgid: int,
        name: str,
        fields: Sequence[Tuple[str, str, int]],
        extensions_start: Optional[int] = None,
        published_crc_extra: Optional[int] = None,
    ) -> None:
        self.msgid = msgid
        self.name = name
        self.fields = tuple(_Field(t, n, a) for t, n, a in fields)
        for f in self.fields:
            if f.type not in _TYPES:
                raise MavlinkError("%s.%s: unknown type %r" % (name, f.name, f.type))
            if f.type == "char" and f.array_length:
                # No HIL message needs one; refusing beats shipping an untested
                # str/bytes path that would silently mangle a string field.
                raise MavlinkError("%s.%s: char arrays are not supported" % (name, f.name))
            if f.name in _RESERVED_KWARGS:
                raise MavlinkError("%s.%s collides with a frame override kwarg" % (name, f.name))

        self.extensions_start = len(self.fields) if extensions_start is None else extensions_start
        base = self.fields[: self.extensions_start]
        ext = self.fields[self.extensions_start :]

        # Descending element size; Python's sort is stable, which is exactly the
        # "ties broken by declaration order" rule. Extensions are appended raw.
        self.ordered_fields: Tuple[_Field, ...] = tuple(sorted(base, key=lambda f: -f.elem_size)) + ext

        self.format = "<" + "".join(f.fmt for f in self.ordered_fields)
        self._struct = struct.Struct(self.format)
        self.wire_length = self._struct.size
        self.field_names = tuple(f.name for f in self.fields)

        self.crc_extra = self._derive_crc_extra()
        if published_crc_extra is not None and published_crc_extra != self.crc_extra:
            raise MavlinkError(
                "%s: CRC_EXTRA mismatch -- derived %d, published %d. The field "
                "table and the published dialect disagree; one of them is wrong."
                % (name, self.crc_extra, published_crc_extra)
            )

        if self.wire_length > MAX_PAYLOAD:
            raise MavlinkError("%s: payload %d exceeds 255" % (name, self.wire_length))

    def _derive_crc_extra(self) -> int:
        """CRC_EXTRA per the documented algorithm, so the table is not a guess.

        MCRF4XX over "NAME " then, for each wire-ordered non-extension field,
        "<type> " + "<name> " plus a single array-length byte for arrays; the
        16-bit result is folded to 8. Extension fields are excluded on purpose:
        that is what makes adding one backward compatible.
        """
        crc = crc16_mcrf4xx((self.name + " ").encode("ascii"))
        for f in self.ordered_fields[: self.extensions_start]:
            crc = crc16_mcrf4xx((f.type + " ").encode("ascii"), crc)
            crc = crc16_mcrf4xx((f.name + " ").encode("ascii"), crc)
            if f.array_length:
                crc = crc16_mcrf4xx(bytes((f.array_length,)), crc)
        return (crc & 0xFF) ^ (crc >> 8)

    def pack(self, values: Mapping[str, Any]) -> bytes:
        flat: List[Any] = []
        for f in self.ordered_fields:
            v = values.get(f.name, 0)
            if f.array_length:
                if isinstance(v, (int, float)):
                    v = [v] * f.array_length
                seq = list(v)[: f.array_length]
                seq.extend([0] * (f.array_length - len(seq)))
                flat.extend(seq)
            else:
                flat.append(v)
        try:
            return self._struct.pack(*flat)
        except struct.error as exc:
            raise MavlinkError("%s: cannot pack (%s)" % (self.name, exc)) from exc

    def unpack(self, payload: bytes) -> Dict[str, Any]:
        # A v2 sender trims trailing zeros, and a NEWER sender may append
        # extension fields we do not know about. Pad the first case, clip the
        # second; both are required for forward and backward compatibility.
        if len(payload) < self.wire_length:
            payload = payload + b"\x00" * (self.wire_length - len(payload))
        elif len(payload) > self.wire_length:
            payload = payload[: self.wire_length]
        raw = self._struct.unpack(payload)
        out: Dict[str, Any] = {}
        i = 0
        for f in self.ordered_fields:
            if f.array_length:
                out[f.name] = list(raw[i : i + f.array_length])
                i += f.array_length
            else:
                out[f.name] = raw[i]
                i += 1
        return out


# --------------------------------------------------------------------------
# Message table. Fields are listed in DECLARATION order (as in the MAVLink XML);
# the wire order is derived above. `extensions_start` is the index of the first
# field after <extensions/>. The trailing int is the published CRC_EXTRA, kept
# only as a self-check against the derivation -- the derivation is the source of
# truth and a disagreement raises at import.
# --------------------------------------------------------------------------

_DEFS: Tuple[_MessageDef, ...] = (
    _MessageDef(0, "HEARTBEAT", (
        ("uint8_t", "type", 0),
        ("uint8_t", "autopilot", 0),
        ("uint8_t", "base_mode", 0),
        ("uint32_t", "custom_mode", 0),
        ("uint8_t", "system_status", 0),
        # XML type is uint8_t_mavlink_version; the generator normalises it to
        # uint8_t before the CRC, so uint8_t is what belongs here.
        ("uint8_t", "mavlink_version", 0),
    ), None, 50),

    _MessageDef(2, "SYSTEM_TIME", (
        ("uint64_t", "time_unix_usec", 0),
        ("uint32_t", "time_boot_ms", 0),
    ), None, 137),

    _MessageDef(76, "COMMAND_LONG", (
        ("uint8_t", "target_system", 0),
        ("uint8_t", "target_component", 0),
        ("uint16_t", "command", 0),
        ("uint8_t", "confirmation", 0),
        ("float", "param1", 0),
        ("float", "param2", 0),
        ("float", "param3", 0),
        ("float", "param4", 0),
        ("float", "param5", 0),
        ("float", "param6", 0),
        ("float", "param7", 0),
    ), None, 152),

    _MessageDef(93, "HIL_ACTUATOR_CONTROLS", (
        ("uint64_t", "time_usec", 0),
        ("float", "controls", 16),
        ("uint8_t", "mode", 0),
        ("uint64_t", "flags", 0),
    ), None, 47),

    _MessageDef(107, "HIL_SENSOR", (
        ("uint64_t", "time_usec", 0),
        ("float", "xacc", 0),
        ("float", "yacc", 0),
        ("float", "zacc", 0),
        ("float", "xgyro", 0),
        ("float", "ygyro", 0),
        ("float", "zgyro", 0),
        ("float", "xmag", 0),
        ("float", "ymag", 0),
        ("float", "zmag", 0),
        ("float", "abs_pressure", 0),
        ("float", "diff_pressure", 0),
        ("float", "pressure_alt", 0),
        ("float", "temperature", 0),
        ("uint32_t", "fields_updated", 0),
        ("uint8_t", "id", 0),           # <extensions/>
    ), 15, 108),

    _MessageDef(113, "HIL_GPS", (
        ("uint64_t", "time_usec", 0),
        ("uint8_t", "fix_type", 0),
        ("int32_t", "lat", 0),
        ("int32_t", "lon", 0),
        ("int32_t", "alt", 0),
        ("uint16_t", "eph", 0),
        ("uint16_t", "epv", 0),
        ("uint16_t", "vel", 0),
        ("int16_t", "vn", 0),
        ("int16_t", "ve", 0),
        ("int16_t", "vd", 0),
        ("uint16_t", "cog", 0),
        ("uint8_t", "satellites_visible", 0),
        ("uint8_t", "id", 0),           # <extensions/>
        ("uint16_t", "yaw", 0),
    ), 13, 124),

    _MessageDef(115, "HIL_STATE_QUATERNION", (
        ("uint64_t", "time_usec", 0),
        ("float", "attitude_quaternion", 4),
        ("float", "rollspeed", 0),
        ("float", "pitchspeed", 0),
        ("float", "yawspeed", 0),
        ("int32_t", "lat", 0),
        ("int32_t", "lon", 0),
        ("int32_t", "alt", 0),
        ("int16_t", "vx", 0),
        ("int16_t", "vy", 0),
        ("int16_t", "vz", 0),
        ("uint16_t", "ind_airspeed", 0),
        ("uint16_t", "true_airspeed", 0),
        ("int16_t", "xacc", 0),
        ("int16_t", "yacc", 0),
        ("int16_t", "zacc", 0),
    ), None, 4),
)

MESSAGES_BY_NAME: Dict[str, _MessageDef] = {d.name: d for d in _DEFS}
MESSAGES_BY_ID: Dict[int, _MessageDef] = {d.msgid: d for d in _DEFS}
CRC_EXTRA: Dict[str, int] = {d.name: d.crc_extra for d in _DEFS}


@dataclass(frozen=True)
class DecodedMessage:
    msgid: int
    name: str
    seq: int
    sysid: int
    compid: int
    fields: Dict[str, Any]

    def __getitem__(self, key: str) -> Any:
        return self.fields[key]

    def get(self, key: str, default: Any = None) -> Any:
        return self.fields.get(key, default)


class MavlinkCodec:
    """Encoder and streaming decoder for the HIL message set.

    Not thread-safe: `feed` mutates one buffer and `encode` one sequence
    counter. Use one codec per link, which is the natural HIL shape anyway.
    """

    def __init__(self, sysid: int = 1, compid: int = 1) -> None:
        self.sysid = sysid
        self.compid = compid
        self._buf = bytearray()
        self._seq = 0
        self.stats = {"good": 0, "bad_crc": 0, "unknown": 0, "resync_bytes": 0}

    # -- encode ------------------------------------------------------------

    def encode(
        self,
        msg_name: str,
        *,
        seq: Optional[int] = None,
        sysid: Optional[int] = None,
        compid: Optional[int] = None,
        **fields: Any,
    ) -> bytes:
        """Build one unsigned v2 frame. Unset fields default to zero."""
        try:
            d = MESSAGES_BY_NAME[msg_name]
        except KeyError:
            raise MavlinkError(
                "unknown message %r; this codec carries %s"
                % (msg_name, ", ".join(sorted(MESSAGES_BY_NAME)))
            ) from None

        stray = set(fields) - set(d.field_names)
        if stray:
            raise MavlinkError(
                "%s has no field(s) %s; fields are %s"
                % (msg_name, ", ".join(sorted(stray)), ", ".join(d.field_names))
            )

        payload = d.pack(fields)
        # v2 truncation: drop trailing zero bytes. At least one byte always
        # remains, even for an all-zero payload.
        n = len(payload)
        while n > 1 and payload[n - 1] == 0:
            n -= 1
        payload = payload[:n]

        if seq is None:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF

        msgid = d.msgid
        frame = bytearray((
            STX_V2,
            len(payload),
            0,  # incompat_flags: 0 = unsigned. A non-zero value a peer does not
                # understand means the frame must be dropped, so never set one
                # here speculatively.
            0,  # compat_flags
            seq & 0xFF,
            self.sysid if sysid is None else sysid,
            self.compid if compid is None else compid,
            msgid & 0xFF,
            (msgid >> 8) & 0xFF,
            (msgid >> 16) & 0xFF,
        ))
        frame += payload
        crc = crc16_mcrf4xx(frame[1:])            # len byte through payload end
        crc = crc16_mcrf4xx((d.crc_extra,), crc)  # CRC_EXTRA folded in last
        frame += struct.pack("<H", crc)
        return bytes(frame)

    # -- decode ------------------------------------------------------------

    def feed(self, data: bytes) -> List[DecodedMessage]:
        """Push received bytes, get back every complete valid frame in order.

        Partial frames are held for the next call. Garbage is discarded by
        scanning forward to the next STX; a known message whose checksum fails
        costs one byte of resync, never the rest of the stream.
        """
        buf = self._buf
        buf += data
        out: List[DecodedMessage] = []

        while True:
            if not buf:
                break

            if buf[0] != STX_V2:
                i = buf.find(STX_V2)
                if i < 0:
                    self.stats["resync_bytes"] += len(buf)
                    del buf[:]
                    break
                self.stats["resync_bytes"] += i
                del buf[:i]
                continue

            if len(buf) < HEADER_LEN:
                break

            plen = buf[1]
            siglen = SIGNATURE_LEN if (buf[2] & IFLAG_SIGNED) else 0
            total = HEADER_LEN + plen + CHECKSUM_LEN + siglen
            if len(buf) < total:
                break

            msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            d = MESSAGES_BY_ID.get(msgid)
            if d is None:
                # We cannot checksum a message whose CRC_EXTRA we do not have,
                # so trust the framing length and skip the whole frame. That is
                # what a real MAVLink router does. The trade-off is that a
                # garbage run which happens to look like an unknown message can
                # swallow the bytes after it; a known message never can, because
                # its checksum is verified below.
                self.stats["unknown"] += 1
                del buf[:total]
                continue

            end = HEADER_LEN + plen
            want = buf[end] | (buf[end + 1] << 8)
            crc = crc16_mcrf4xx(buf[1:end])
            crc = crc16_mcrf4xx((d.crc_extra,), crc)
            if crc != want:
                self.stats["bad_crc"] += 1
                self.stats["resync_bytes"] += 1
                del buf[:1]
                continue

            out.append(DecodedMessage(
                msgid=msgid,
                name=d.name,
                seq=buf[4],
                sysid=buf[5],
                compid=buf[6],
                fields=d.unpack(bytes(buf[HEADER_LEN:end])),
            ))
            self.stats["good"] += 1
            del buf[:total]

        return out

    def reset(self) -> None:
        """Drop buffered partial input. Sequence counter and stats survive."""
        del self._buf[:]

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)
