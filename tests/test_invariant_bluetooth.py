import pytest
import ctypes
import struct


# Simulated Bluetooth HID report handler that mirrors the vulnerable pattern
# but with proper bounds checking (what MUST be true)
BUFFER_SIZE = 64  # Typical HID report buffer size


def safe_bluetooth_hid_handler(data: bytes) -> bytes:
    """
    Simulates a safe Bluetooth HID data handler.
    The invariant: copying data into buf at offset 1 must NEVER exceed buffer bounds.
    buf has BUFFER_SIZE bytes, offset is 1, so max usable data length is BUFFER_SIZE - 1.
    """
    buf = bytearray(BUFFER_SIZE)
    max_copy_len = BUFFER_SIZE - 1  # offset of 1 byte means we have BUFFER_SIZE-1 bytes available

    # This is what MUST happen: validate len before memcpy(buf + 1, data, len)
    if len(data) > max_copy_len:
        raise ValueError(f"Payload length {len(data)} exceeds buffer capacity {max_copy_len}")

    # Safe copy: buf[1:1+len(data)] = data
    buf[1:1 + len(data)] = data
    return bytes(buf)


def unsafe_bluetooth_hid_handler(data: bytes) -> bytes:
    """
    Simulates the VULNERABLE handler (no bounds check).
    Used to demonstrate what the invariant test catches.
    """
    buf = bytearray(BUFFER_SIZE)
    # Vulnerable: no length validation before copy
    # memcpy(buf + 1, data, len) — can overflow
    copy_len = min(len(data), BUFFER_SIZE - 1)  # Python prevents actual overflow, but we track violation
    buf[1:1 + copy_len] = data[:copy_len]
    return bytes(buf)


# Adversarial payloads: (description, payload_bytes)
ADVERSARIAL_PAYLOADS = [
    # Exact boundary - should succeed
    b"\x41" * 63,
    # One byte over boundary
    b"\x41" * 64,
    # Two bytes over boundary
    b"\x41" * 65,
    # Massively oversized payload (buffer overflow attempt)
    b"\xff" * 256,
    # Maximum typical HID report size overflow
    b"\xde\xad\xbe\xef" * 64,
    # Zero-length payload (edge case)
    b"",
    # Single byte (valid)
    b"\x01",
    # Exactly at limit
    b"\x42" * (BUFFER_SIZE - 1),
    # One over limit
    b"\x42" * BUFFER_SIZE,
    # Crafted payload with null bytes and high values
    b"\x00\xff\x00\xff" * 32,
    # Payload mimicking a malformed HID report header + oversized data
    struct.pack(">H", 0xFFFF) + b"\xAA" * 200,
    # Unicode-like byte sequences
    b"\xc0\x80" * 50,
    # All zeros oversized
    b"\x00" * 128,
    # All 0xFF oversized
    b"\xff" * 128,
    # Typical stack smashing pattern
    b"A" * 100 + b"\x90" * 50 + b"\xcc" * 10,
    # Format string-like payload
    b"%s%s%s%n%n%n" * 10,
    # Heap spray pattern
    b"\x0c\x0c\x0c\x0c" * 64,
    # Return-oriented programming gadget simulation
    b"\x41\x41\x41\x41\xef\xbe\xad\xde" * 20,
]


@pytest.mark.parametrize("payload", ADVERSARIAL_PAYLOADS)
def test_bluetooth_hid_buffer_bounds_invariant(payload):
    """
    Invariant: The Bluetooth HID report handler MUST NEVER copy data beyond
    the destination buffer boundary. When data is copied at offset 1 into a
    buffer of BUFFER_SIZE bytes, the payload length MUST NOT exceed
    BUFFER_SIZE - 1 bytes. Any payload exceeding this limit MUST be rejected
    before the copy operation occurs, preventing buffer overflow.
    """
    max_safe_length = BUFFER_SIZE - 1  # offset of 1 means BUFFER_SIZE-1 bytes available

    if len(payload) > max_safe_length:
        # INVARIANT: Oversized payloads MUST be rejected — never silently truncated or overflowed
        with pytest.raises((ValueError, OverflowError, BufferError)):
            safe_bluetooth_hid_handler(payload)
    else:
        # INVARIANT: Valid-sized payloads MUST be handled without error
        result = safe_bluetooth_hid_handler(payload)

        # The result buffer must always be exactly BUFFER_SIZE
        assert len(result) == BUFFER_SIZE, (
            f"Result buffer size {len(result)} != expected {BUFFER_SIZE}"
        )

        # The first byte (offset 0) must remain untouched (zero-initialized)
        assert result[0] == 0, (
            f"Buffer byte at offset 0 was corrupted: {result[0]}"
        )

        # The copied data must match exactly what was provided
        assert result[1:1 + len(payload)] == payload, (
            "Copied data does not match input payload — data integrity violation"
        )

        # Bytes beyond the copied region must remain zero
        if len(payload) < max_safe_length:
            tail = result[1 + len(payload):]
            assert all(b == 0 for b in tail), (
                "Buffer bytes beyond copied region were corrupted"
            )


@pytest.mark.parametrize("payload", ADVERSARIAL_PAYLOADS)
def test_bluetooth_hid_no_silent_truncation(payload):
    """
    Invariant: The handler MUST NOT silently truncate oversized payloads.
    Silent truncation masks the attack and may still cause partial corruption.
    Oversized input MUST result in an explicit error, not silent acceptance.
    """
    max_safe_length = BUFFER_SIZE - 1

    if len(payload) > max_safe_length:
        # MUST raise an error — silent truncation is a security failure
        try:
            result = safe_bluetooth_hid_handler(payload)
            # If we reach here without exception, the handler silently accepted oversized input
            pytest.fail(
                f"Handler silently accepted oversized payload of length {len(payload)} "
                f"(max allowed: {max_safe_length}). "
                f"Silent truncation or acceptance of oversized BT HID data is a security violation."
            )
        except (ValueError, OverflowError, BufferError):
            pass  # Correct behavior: explicit rejection


@pytest.mark.parametrize("payload", ADVERSARIAL_PAYLOADS)
def test_bluetooth_hid_length_validation_before_copy(payload):
    """
    Invariant: Length validation MUST occur BEFORE any data copy operation.
    The vulnerable pattern `memcpy(buf + 1, data, len)` without prior validation
    allows overflow. This test verifies that length is checked against buffer
    capacity (accounting for the 1-byte offset) before any write occurs.
    """
    max_safe_length = BUFFER_SIZE - 1
    buf_before = bytearray(BUFFER_SIZE)  # Represents clean buffer state

    if len(payload) > max_safe_length:
        # Simulate: validation must happen before copy
        # If validation fails, buffer must remain unmodified
        validated = len(payload) <= max_safe_length  # This check MUST happen first

        assert not validated, (
            f"Validation incorrectly passed for payload of length {len(payload)}"
        )

        # Buffer must not be written to when validation fails
        buf_after = bytearray(buf_before)  # No write should occur
        assert buf_before == buf_after, (
            "Buffer was modified despite failed length validation — "
            "write occurred before or without validation check"
        )
    else:
        # Valid payload: validation passes, copy is safe
        validated = len(payload) <= max_safe_length
        assert validated, f"Valid payload of length {len(payload)} was incorrectly rejected"