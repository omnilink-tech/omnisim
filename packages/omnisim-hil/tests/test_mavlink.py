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

"""Offline tests for the HIL MAVLink v2 codec.

No simulator, no network, no build. From the repo root:

    python -m pytest packages/omnisim-hil/tests/test_mavlink.py -q

The byte-for-byte tests against pymavlink self-skip when it is not installed;
everything else runs on the stdlib alone.
"""

import random
import struct

import pytest

from omnisim_hil.mavlink import (
    CRC_EXTRA,
    HEADER_LEN,
    MESSAGES_BY_ID,
    MESSAGES_BY_NAME,
    STX_V2,
    MavlinkCodec,
    MavlinkError,
    _MessageDef,
    crc16_mcrf4xx,
)

try:
    from pymavlink.dialects.v20 import common as mavcommon

    HAVE_PYMAVLINK = True
except Exception:  # pragma: no cover - depends on the interpreter, not the code
    mavcommon = None
    HAVE_PYMAVLINK = False

requires_pymavlink = pytest.mark.skipif(
    not HAVE_PYMAVLINK,
    reason=(
        "pymavlink is not installed in this interpreter, so the byte-for-byte "
        "oracle comparison cannot run. The codec itself never imports it. "
        "To run these: pip install pymavlink"
    ),
)


# Every field non-zero, so a round-trip frame carries a full untruncated
# payload and any field-order error shows up as a value landing in the wrong
# slot rather than as a length change. The last WIRE field of each message also
# needs a non-zero top byte, or v2 legitimately trims it and the "nothing was
# truncated" assertion below fails on a correct encoder.
SAMPLES = {
    "HEARTBEAT": dict(
        type=2, autopilot=12, base_mode=81, custom_mode=0x00040001,
        system_status=4, mavlink_version=3,
    ),
    "SYSTEM_TIME": dict(time_unix_usec=1767225600123456, time_boot_ms=0x0A01818D),
    "COMMAND_LONG": dict(
        target_system=1, target_component=190, command=400, confirmation=2,
        param1=1.5, param2=2.25, param3=-3.75, param4=4.5,
        param5=47.3977419, param6=8.5455938, param7=125.0,
    ),
    "HIL_ACTUATOR_CONTROLS": dict(
        time_usec=1234567890123,
        controls=[0.1 * i - 0.5 for i in range(16)],
        mode=129,
        flags=0x0123456789ABCDEF,
    ),
    "HIL_SENSOR": dict(
        time_usec=987654321098,
        xacc=0.125, yacc=-0.25, zacc=-9.8125,
        xgyro=0.0125, ygyro=-0.03125, zgyro=0.5,
        xmag=0.21875, ymag=-0.0625, zmag=0.4375,
        abs_pressure=1013.25, diff_pressure=1.5, pressure_alt=142.75,
        temperature=21.5, fields_updated=0x1FFF, id=3,
    ),
    "HIL_GPS": dict(
        time_usec=555666777888, fix_type=3,
        lat=473977419, lon=85455938, alt=125000,
        eph=121, epv=200, vel=1150, vn=800, ve=-600, vd=-45,
        cog=27000, satellites_visible=14, id=1, yaw=18000,
    ),
    "HIL_STATE_QUATERNION": dict(
        time_usec=111222333444,
        attitude_quaternion=[0.5, 0.5, -0.5, 0.5],
        rollspeed=0.0625, pitchspeed=-0.125, yawspeed=0.25,
        lat=473977419, lon=85455938, alt=125000,
        vx=1200, vy=-340, vz=55,
        ind_airspeed=1250, true_airspeed=1300,
        xacc=100, yacc=-200, zacc=-981,
    ),
}

# The published dialect values, transcribed independently of the derivation in
# mavlink.py. If the derivation and this table ever disagree, one of them is
# wrong and the codec is off the wire.
PUBLISHED_CRC_EXTRA = {
    "HEARTBEAT": 50,
    "SYSTEM_TIME": 137,
    "COMMAND_LONG": 152,
    "HIL_ACTUATOR_CONTROLS": 47,
    "HIL_SENSOR": 108,
    "HIL_GPS": 124,
    "HIL_STATE_QUATERNION": 4,
}

# Golden wire layouts. These are the whole point of the codec: a change here is
# a change to what a real autopilot will read.
GOLDEN_FORMATS = {
    "HEARTBEAT": "<IBBBBB",
    "SYSTEM_TIME": "<QI",
    "COMMAND_LONG": "<fffffffHBBB",
    "HIL_ACTUATOR_CONTROLS": "<QQ16fB",
    "HIL_SENSOR": "<QfffffffffffffIB",
    "HIL_GPS": "<QiiiHHHhhhHBBBH",
    "HIL_STATE_QUATERNION": "<Q4ffffiiihhhHHhhh",
}

ALL = sorted(SAMPLES)


def assert_fields_equal(got, expected):
    for key, want in expected.items():
        have = got[key]
        if isinstance(want, list):
            assert len(have) == len(want), key
            for i, (h, w) in enumerate(zip(have, want)):
                assert h == pytest.approx(w, rel=1e-6, abs=1e-6), "%s[%d]" % (key, i)
        elif isinstance(want, float):
            assert have == pytest.approx(want, rel=1e-6, abs=1e-6), key
        else:
            assert have == want, key


# -- checksum ---------------------------------------------------------------


def test_crc16_mcrf4xx_known_answers():
    # CRC-16/MCRF4XX check value for "123456789" is 0x6F91.
    assert crc16_mcrf4xx(b"123456789") == 0x6F91
    assert crc16_mcrf4xx(b"") == 0xFFFF
    # Accumulating in two runs must equal one run over the concatenation, which
    # is what folding CRC_EXTRA in after the payload relies on.
    assert crc16_mcrf4xx(b"9", crc16_mcrf4xx(b"12345678")) == 0x6F91


# -- layout -----------------------------------------------------------------


@pytest.mark.parametrize("name", ALL)
def test_wire_format_matches_golden(name):
    assert MESSAGES_BY_NAME[name].format == GOLDEN_FORMATS[name]


@pytest.mark.parametrize("name", ALL)
def test_base_fields_sort_by_descending_element_size(name):
    d = MESSAGES_BY_NAME[name]
    sizes = [f.elem_size for f in d.ordered_fields[: d.extensions_start]]
    assert sizes == sorted(sizes, reverse=True)


def test_size_ties_keep_declaration_order():
    # time_usec and flags are both uint64_t; time_usec is declared first and
    # must stay first. controls (float[16]) sorts on its 4-byte ELEMENT, not on
    # its 64-byte total, so it lands after both.
    d = MESSAGES_BY_NAME["HIL_ACTUATOR_CONTROLS"]
    assert [f.name for f in d.ordered_fields] == ["time_usec", "flags", "controls", "mode"]


def test_extension_fields_are_appended_unsorted():
    # HIL_GPS ends with uint8_t id then uint16_t yaw. A 2-byte field after a
    # 1-byte one is impossible under the sort, so this pins that extensions are
    # appended in declaration order instead of being reordered.
    d = MESSAGES_BY_NAME["HIL_GPS"]
    assert [f.name for f in d.ordered_fields[-2:]] == ["id", "yaw"]
    assert [f.elem_size for f in d.ordered_fields[-2:]] == [1, 2]


# -- CRC_EXTRA --------------------------------------------------------------


def test_crc_extra_derivation_matches_published_table():
    assert CRC_EXTRA == PUBLISHED_CRC_EXTRA


def test_crc_extra_excludes_extension_fields():
    # Same fields twice; only the <extensions/> boundary moves. If the
    # derivation counted extensions, adding one would change CRC_EXTRA and
    # break every existing peer -- so the marked variant must keep the
    # published value and the unmarked one must not.
    fields = [(f.type, f.name, f.array_length) for f in MESSAGES_BY_NAME["HIL_GPS"].fields]
    with_ext = _MessageDef(113, "HIL_GPS", fields, extensions_start=13)
    without_ext = _MessageDef(113, "HIL_GPS", fields, extensions_start=None)
    assert with_ext.crc_extra == PUBLISHED_CRC_EXTRA["HIL_GPS"]
    assert without_ext.crc_extra != with_ext.crc_extra


def test_crc_extra_derivation_is_sensitive_to_the_field_table():
    # Guards against a derivation that silently returns a constant.
    fields = [(f.type, f.name, f.array_length) for f in MESSAGES_BY_NAME["HIL_SENSOR"].fields]
    renamed = list(fields)
    renamed[1] = ("float", "xaccel", 0)
    assert _MessageDef(107, "HIL_SENSOR", renamed, 15).crc_extra != PUBLISHED_CRC_EXTRA["HIL_SENSOR"]
    retyped = list(fields)
    retyped[14] = ("uint16_t", "fields_updated", 0)
    assert _MessageDef(107, "HIL_SENSOR", retyped, 15).crc_extra != PUBLISHED_CRC_EXTRA["HIL_SENSOR"]


# -- framing ----------------------------------------------------------------


@pytest.mark.parametrize("name", ALL)
def test_round_trip(name):
    codec = MavlinkCodec(sysid=42, compid=7)
    frame = codec.encode(name, seq=13, **SAMPLES[name])
    got = MavlinkCodec().feed(frame)
    assert len(got) == 1
    msg = got[0]
    assert msg.name == name
    assert msg.msgid == MESSAGES_BY_NAME[name].msgid
    assert (msg.seq, msg.sysid, msg.compid) == (13, 42, 7)
    assert_fields_equal(msg.fields, SAMPLES[name])


@pytest.mark.parametrize("name", ALL)
def test_frame_header_is_well_formed(name):
    d = MESSAGES_BY_NAME[name]
    frame = MavlinkCodec(sysid=1, compid=1).encode(name, seq=200, **SAMPLES[name])
    assert frame[0] == STX_V2
    assert frame[1] == len(frame) - HEADER_LEN - 2
    assert frame[2] == 0 and frame[3] == 0  # unsigned, no compat flags
    assert frame[4] == 200
    assert frame[7] | (frame[8] << 8) | (frame[9] << 16) == d.msgid
    crc = crc16_mcrf4xx(frame[1:-2])
    crc = crc16_mcrf4xx((d.crc_extra,), crc)
    assert struct.unpack("<H", frame[-2:])[0] == crc
    # All-non-zero sample, so nothing should have been truncated.
    assert frame[1] == d.wire_length


def test_sequence_number_auto_increments_and_wraps():
    codec = MavlinkCodec()
    seqs = [codec.encode("HEARTBEAT", **SAMPLES["HEARTBEAT"])[4] for _ in range(258)]
    assert seqs[:3] == [0, 1, 2]
    assert seqs[255:258] == [255, 0, 1]


# -- truncation -------------------------------------------------------------


def test_trailing_zero_fields_shorten_the_frame_and_still_decode():
    full = dict(SAMPLES["HIL_SENSOR"])
    trimmed = dict(full)
    # The last three wire fields: temperature (float), fields_updated (uint32),
    # id (uint8) -- 9 trailing bytes that v2 must drop.
    trimmed.update(temperature=0.0, fields_updated=0, id=0)

    codec = MavlinkCodec()
    long_frame = codec.encode("HIL_SENSOR", **full)
    short_frame = codec.encode("HIL_SENSOR", **trimmed)

    assert len(short_frame) == len(long_frame) - 9
    assert short_frame[1] == MESSAGES_BY_NAME["HIL_SENSOR"].wire_length - 9

    got = MavlinkCodec().feed(short_frame)
    assert len(got) == 1
    assert_fields_equal(got[0].fields, trimmed)


def test_all_zero_payload_keeps_one_byte():
    # v2 trims trailing zeros but never to an empty payload.
    frame = MavlinkCodec().encode("SYSTEM_TIME", time_unix_usec=0, time_boot_ms=0)
    assert frame[1] == 1
    assert len(frame) == HEADER_LEN + 1 + 2
    got = MavlinkCodec().feed(frame)
    assert got[0].fields == {"time_unix_usec": 0, "time_boot_ms": 0}


def test_decoder_zero_pads_a_short_payload():
    # Same message, hand-truncated one byte further than the encoder would go.
    codec = MavlinkCodec()
    frame = bytearray(codec.encode("HIL_GPS", time_usec=7, fix_type=3, lat=1))
    assert frame[1] > 1
    frame[1] -= 1
    payload_end = HEADER_LEN + frame[1]
    body = frame[:payload_end]
    crc = crc16_mcrf4xx(body[1:])
    crc = crc16_mcrf4xx((MESSAGES_BY_NAME["HIL_GPS"].crc_extra,), crc)
    got = MavlinkCodec().feed(bytes(body) + struct.pack("<H", crc))
    assert len(got) == 1
    assert got[0]["time_usec"] == 7 and got[0]["lat"] == 1
    assert got[0]["yaw"] == 0


def test_decoder_clips_a_payload_from_a_newer_sender():
    # A future dialect appends an extension field we do not know about. The
    # extra bytes must be ignored, not shift every field.
    d = MESSAGES_BY_NAME["HIL_SENSOR"]
    codec = MavlinkCodec()
    frame = bytearray(codec.encode("HIL_SENSOR", **SAMPLES["HIL_SENSOR"]))
    body = frame[:-2] + b"\xAA\xBB\xCC\xDD"
    body[1] = d.wire_length + 4
    crc = crc16_mcrf4xx(body[1:])
    crc = crc16_mcrf4xx((d.crc_extra,), crc)
    got = MavlinkCodec().feed(bytes(body) + struct.pack("<H", crc))
    assert len(got) == 1
    assert_fields_equal(got[0].fields, SAMPLES["HIL_SENSOR"])


# -- streaming --------------------------------------------------------------


def build_stream(garbage_fn, seed=20260822):
    """Frames for every message, separated by garbage. Returns (bytes, names)."""
    rng = random.Random(seed)
    codec = MavlinkCodec()
    stream = bytearray()
    for name in ALL:
        stream += garbage_fn(rng, rng.randrange(1, 40))
        stream += codec.encode(name, **SAMPLES[name])
    stream += garbage_fn(rng, 23)
    return bytes(stream), list(ALL)


def _noise_without_stx(rng, n):
    # 0xFD excluded on purpose: this case isolates "discard non-framing bytes".
    # A counterfeit STX is a different failure mode and has its own test below.
    return bytes(rng.randrange(0x00, 0xFD) for _ in range(n))


def test_stream_recovers_every_frame_from_surrounding_garbage():
    stream, expected = build_stream(_noise_without_stx)
    codec = MavlinkCodec()
    got = codec.feed(stream)
    assert [m.name for m in got] == expected
    for msg in got:
        assert_fields_equal(msg.fields, SAMPLES[msg.name])
    assert codec.stats["good"] == len(expected)
    assert codec.stats["resync_bytes"] > 0


def test_stream_survives_byte_at_a_time_delivery():
    stream, expected = build_stream(_noise_without_stx)
    codec = MavlinkCodec()
    got = []
    for i in range(len(stream)):
        got.extend(codec.feed(stream[i : i + 1]))
    assert [m.name for m in got] == expected
    for msg in got:
        assert_fields_equal(msg.fields, SAMPLES[msg.name])


def test_counterfeit_stx_costs_one_byte_of_resync_not_the_stream():
    good = MavlinkCodec()
    a = good.encode("HEARTBEAT", **SAMPLES["HEARTBEAT"])
    b = good.encode("HIL_SENSOR", **SAMPLES["HIL_SENSOR"])
    # A byte run that looks like the start of a HEARTBEAT but checksums wrong.
    fake = bytes([STX_V2, 9, 0, 0, 0, 1, 1, 0, 0, 0]) + b"\x01" * 9 + b"\xDE\xAD"

    codec = MavlinkCodec()
    got = codec.feed(a + fake + b)
    assert [m.name for m in got] == ["HEARTBEAT", "HIL_SENSOR"]
    assert codec.stats["bad_crc"] >= 1


def test_corrupted_payload_is_dropped_and_the_stream_continues():
    codec = MavlinkCodec()
    a = codec.encode("HIL_GPS", **SAMPLES["HIL_GPS"])
    bad = bytearray(codec.encode("HIL_SENSOR", **SAMPLES["HIL_SENSOR"]))
    bad[HEADER_LEN + 3] ^= 0xFF
    c = codec.encode("SYSTEM_TIME", **SAMPLES["SYSTEM_TIME"])

    dec = MavlinkCodec()
    got = dec.feed(bytes(a) + bytes(bad) + bytes(c))
    assert [m.name for m in got] == ["HIL_GPS", "SYSTEM_TIME"]
    assert dec.stats["bad_crc"] >= 1


def test_unknown_msgid_is_skipped_without_desync():
    codec = MavlinkCodec()
    a = codec.encode("HEARTBEAT", **SAMPLES["HEARTBEAT"])
    b = codec.encode("HIL_GPS", **SAMPLES["HIL_GPS"])
    unknown_id = 0x00BEEF
    assert unknown_id not in MESSAGES_BY_ID
    payload = bytes(range(20))
    mystery = bytes([
        STX_V2, len(payload), 0, 0, 5, 1, 1,
        unknown_id & 0xFF, (unknown_id >> 8) & 0xFF, (unknown_id >> 16) & 0xFF,
    ]) + payload + b"\x00\x00"

    dec = MavlinkCodec()
    got = dec.feed(a + mystery + b)
    assert [m.name for m in got] == ["HEARTBEAT", "HIL_GPS"]
    assert dec.stats["unknown"] == 1


def test_signed_frame_is_skipped_by_its_full_length():
    # Signing is not implemented, but the 13-byte trailer must be accounted for
    # or every frame after a signed one would be misparsed.
    codec = MavlinkCodec()
    a = codec.encode("HEARTBEAT", **SAMPLES["HEARTBEAT"])
    signed = bytearray(codec.encode("SYSTEM_TIME", **SAMPLES["SYSTEM_TIME"]))
    signed[2] = 0x01  # MAVLINK_IFLAG_SIGNED
    signed += b"\x00" * 13
    b = codec.encode("HIL_GPS", **SAMPLES["HIL_GPS"])

    dec = MavlinkCodec()
    got = dec.feed(a + bytes(signed) + b)
    # The flipped flag invalidates the checksum, so SYSTEM_TIME is rejected --
    # what matters is that HIL_GPS still parses, i.e. the trailer did not
    # shift the stream.
    assert [m.name for m in got][-1] == "HIL_GPS"


@pytest.mark.parametrize("seed", range(40))
def test_arbitrary_noise_never_corrupts_or_crashes(seed):
    # Garbage here MAY contain 0xFD. A random run that frames up as an unknown
    # msgid is skipped by its declared length and can swallow the bytes after
    # it -- that is real MAVLink behaviour, not a bug, so full recovery is not
    # asserted. What must always hold: no exception, output in order, and every
    # message returned is byte-correct.
    stream, expected = build_stream(
        lambda rng, n: bytes(rng.randrange(0, 256) for _ in range(n)), seed=seed
    )
    got = MavlinkCodec().feed(stream)
    names = [m.name for m in got]

    it = iter(expected)
    assert all(n in it for n in names), "recovered frames out of order"
    for msg in got:
        assert_fields_equal(msg.fields, SAMPLES[msg.name])


def test_buffer_stays_bounded_on_a_pure_garbage_stream():
    codec = MavlinkCodec()
    rng = random.Random(7)
    for _ in range(200):
        assert codec.feed(bytes(rng.randrange(0, 256) for _ in range(512))) == []
        assert codec.pending_bytes < 280  # one maximal frame


# -- caller errors ----------------------------------------------------------


def test_unknown_message_name_is_rejected():
    with pytest.raises(MavlinkError, match="unknown message"):
        MavlinkCodec().encode("HIL_TELEPATHY", x=1)


def test_unknown_field_name_is_rejected():
    with pytest.raises(MavlinkError, match="no field"):
        MavlinkCodec().encode("HEARTBEAT", typo=1)


def test_out_of_range_value_is_rejected():
    with pytest.raises(MavlinkError, match="cannot pack"):
        MavlinkCodec().encode("HEARTBEAT", type=999)


# -- pymavlink oracle -------------------------------------------------------


@requires_pymavlink
@pytest.mark.parametrize("name", ALL)
def test_bytes_match_pymavlink(name):
    mine = MavlinkCodec(sysid=17, compid=42).encode(name, seq=99, **SAMPLES[name])
    mav = mavcommon.MAVLink(None, srcSystem=17, srcComponent=42)
    mav.seq = 99
    theirs = bytes(getattr(mav, name.lower() + "_encode")(**SAMPLES[name]).pack(mav))
    assert mine.hex() == theirs.hex()


@requires_pymavlink
@pytest.mark.parametrize("name", ALL)
def test_truncated_bytes_match_pymavlink(name):
    # Same comparison with a mostly-zero payload, so the truncation rule itself
    # is compared against the reference rather than only the full-length case.
    sparse = {k: (0 if isinstance(v, (int, float)) else [0] * len(v)) for k, v in SAMPLES[name].items()}
    first = next(iter(SAMPLES[name]))
    sparse[first] = SAMPLES[name][first]
    mine = MavlinkCodec(sysid=1, compid=1).encode(name, seq=0, **sparse)
    mav = mavcommon.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.seq = 0
    theirs = bytes(getattr(mav, name.lower() + "_encode")(**sparse).pack(mav))
    assert mine.hex() == theirs.hex()


@requires_pymavlink
def test_crc_extra_matches_pymavlink():
    for name, d in MESSAGES_BY_NAME.items():
        assert d.crc_extra == mavcommon.mavlink_map[d.msgid].crc_extra, name


@requires_pymavlink
@pytest.mark.parametrize("name", ALL)
def test_decodes_frames_produced_by_pymavlink(name):
    mav = mavcommon.MAVLink(None, srcSystem=3, srcComponent=4)
    mav.seq = 5
    frame = bytes(getattr(mav, name.lower() + "_encode")(**SAMPLES[name]).pack(mav))
    got = MavlinkCodec().feed(frame)
    assert len(got) == 1
    assert got[0].name == name
    assert (got[0].seq, got[0].sysid, got[0].compid) == (5, 3, 4)
    assert_fields_equal(got[0].fields, SAMPLES[name])


@requires_pymavlink
def test_pymavlink_decodes_our_frames():
    mav = mavcommon.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.robust_parsing = True
    codec = MavlinkCodec(sysid=1, compid=1)
    stream = b"".join(codec.encode(n, **SAMPLES[n]) for n in ALL)
    got = [m for m in mav.parse_buffer(stream) or [] if m.get_type() != "BAD_DATA"]
    assert [m.get_type() for m in got] == ALL
    for msg in got:
        assert_fields_equal(msg.to_dict(), SAMPLES[msg.get_type()])
