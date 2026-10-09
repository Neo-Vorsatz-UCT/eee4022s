"""
    Module to handle the networking - send the setpoints to the ESP32 over the network via DTLS1.2 (PSK)
    Author: Neo Vorsatz
    Last updated: 7 October 2026
"""

# IMPORTS ================================

# External
import ctypes
import ctypes.util
import math
import socket
import struct
import sys
import time
from typing import Optional

from zeroconf import IPVersion, Zeroconf  # pip install zeroconf

# TYPES ================================

_vp = ctypes.c_void_p # C void pointer
_PSK_CLIENT_CB = ctypes.CFUNCTYPE(ctypes.c_uint, _vp, _vp, _vp, ctypes.c_uint, _vp, ctypes.c_uint) # callback

# CONSTANTS ================================

# Security
PSK = bytes.fromhex("c7676ca068aff229e5ac4e3f8e372103")
PSK_ID = b"eee4022s-braccio-psk"
CIPHER = b"PSK-AES128-GCM-SHA256"

# Target server, found via mDNS
MDNS_HOSTNAME = "eee4022s-braccio" # must match the ESP32's mdns_hostname_set()
MDNS_SERVICE = "_braccio._udp.local." # must match the ESP32's mdns_service_add(NULL, "_braccio", "_udp", ...)
MDNS_TIMEOUT_MS = 2000 # how long to wait for the ESP32 to answer a lookup
ESP32_IPV4_OVERRIDE = None # set to an IPv4 string (e.g. "192.168.0.83") to skip mDNS, for debugging
PORT = 42069 # only used with ESP32_IPV4_OVERRIDE; otherwise the port comes from mDNS

# Setpoints
ANGLE_MIN, ANGLE_MAX = 0.0, 180.0 # the Arduino silently rejects anything outside this range
FRAME = struct.Struct("<H6f")  # 2 bytes of sequencing + 6 float32 setpoints = 26 bytes

# OpenSSL constants, from ssl.h and bio.h
DTLS1_2_VERSION = 0xFEFD
SSL_CTRL_SET_MIN_PROTO_VERSION = 123
SSL_CTRL_SET_MAX_PROTO_VERSION = 124
BIO_NOCLOSE = 0
BIO_CTRL_DGRAM_SET_CONNECTED = 32

# VARIABLES ================================

# Library bindings
_ssl:dict = {}
_crypto:dict = {}

# Session and context
_socket = None # client socket
_context = None # context
_session = None # session
_psk_cb = None # PSK callback - not used, but must avoid garbage collector
_active = False # whether the session is active or not
_next_seq_num:int = 1 # next sequence number for packets
_target:Optional[tuple] = None # last known (ip, port) of the ESP32, from mDNS

# HELPER FUNCTIONS ================================

def _load(name) -> ctypes.CDLL:
    """
    Loads a C library

    Args:
        name (str): Name of library to load
    Raises:
        SystemExit: If the library can't be found
    """
    # Get file path
    path = ctypes.util.find_library(name)
    if path is None:
        sys.exit(f"error: lib{name} not found")

    # Return the library
    return ctypes.CDLL(path)

def _bind(lib:ctypes.CDLL, collection:dict, name:str, restype, *argtypes):
    """
    Binds a function from a C library to Python

    Args:
        lib (CDLL): Library to fetch a function from
        collection (dict[str,Any]): The collection of bindings to save this new binding to
        name (str): Name of function to fetch
        restype (type): Return type of the function
        argtypes (list[type]): List of argument types of the function
    Raises:
        SystemExit: If any of the libraries can't be found
    """
    # Create function
    function = getattr(lib, name) # get function
    function.restype = restype # return type
    function.argtypes = argtypes # argument types

    # Save function
    collection[name] = function

def _create_c_bindings():
    """
    Creates all the necessary C type function bindings
    """
    # Load libraries  (raises SystemExit if libraries can't be found)
    lib_ssl = _load("ssl")
    lib_crypto = _load("crypto")

    # Create bindings for SSL functions
    _bind(lib_ssl, _ssl, "DTLS_client_method", _vp)
    _bind(lib_ssl, _ssl, "SSL_CTX_new", _vp, _vp)
    _bind(lib_ssl, _ssl, "SSL_CTX_free", None, _vp)
    _bind(lib_ssl, _ssl, "SSL_CTX_ctrl", ctypes.c_long, _vp, ctypes.c_int, ctypes.c_long, _vp)
    _bind(lib_ssl, _ssl, "SSL_CTX_set_cipher_list", ctypes.c_int, _vp, ctypes.c_char_p)
    _bind(lib_ssl, _ssl, "SSL_CTX_set_psk_client_callback", None, _vp, _PSK_CLIENT_CB)
    _bind(lib_ssl, _ssl, "SSL_new", _vp, _vp)
    _bind(lib_ssl, _ssl, "SSL_free", None, _vp)
    _bind(lib_ssl, _ssl, "SSL_set_bio", None, _vp, _vp, _vp)
    _bind(lib_ssl, _ssl, "SSL_connect", ctypes.c_int, _vp)
    _bind(lib_ssl, _ssl, "SSL_write", ctypes.c_int, _vp, ctypes.c_char_p, ctypes.c_int)
    _bind(lib_ssl, _ssl, "SSL_shutdown", ctypes.c_int, _vp)
    _bind(lib_ssl, _ssl, "SSL_get_error", ctypes.c_int, _vp, ctypes.c_int)
    _bind(lib_ssl, _ssl, "SSL_get_current_cipher", _vp, _vp)
    _bind(lib_ssl, _ssl, "SSL_CIPHER_get_name", ctypes.c_char_p, _vp)

    # Create bindings for Crypto functions
    _bind(lib_crypto, _crypto, "BIO_new_dgram", _vp, ctypes.c_int, ctypes.c_int)
    _bind(lib_crypto, _crypto, "BIO_ctrl", ctypes.c_long, _vp, ctypes.c_int, ctypes.c_long, _vp)
    _bind(lib_crypto, _crypto, "BIO_ADDR_new", _vp)
    _bind(lib_crypto, _crypto, "BIO_ADDR_free", None, _vp)
    _bind(lib_crypto, _crypto, "BIO_ADDR_rawmake", ctypes.c_int, _vp, ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_ushort)
    _bind(lib_crypto, _crypto, "ERR_get_error", ctypes.c_ulong)
    _bind(lib_crypto, _crypto, "ERR_error_string_n", None, ctypes.c_ulong, ctypes.c_char_p, ctypes.c_size_t)

def _ssl_errors_to_str() -> list[str]:
    """
    Drains OpenSSL's thread-local error queue into readable strings

    Returns:
        list[str]: All of the errors as strings
    """
    # Create buffers
    errors = []
    buffer = ctypes.create_string_buffer(256)

    # Get errors
    while True:
        # Get next error
        err_code = _crypto["ERR_get_error"]()
        if err_code == 0:
            break # no more errors

        # Append error
        _crypto["ERR_error_string_n"](err_code, buffer, len(buffer))
        errors.append(buffer.value.decode(errors="replace"))

    # Return the errors
    return errors

def _print_failure(failure:str, handshake_ret:int=0):
    """
    Prints an error message from the queued SSL errors

    Args:
        failure (str): What failed, causing this error
        handshake_ret (int): Return value of from handshake
    """
    # Global variables
    global _session

    # Get error code
    err_code = None
    if _session and handshake_ret:
        err_code = _ssl["SSL_get_error"](_session, handshake_ret)

    # Get details
    details = "; ".join(_ssl_errors_to_str())
    if len(details) == 0:
        details = "no OpenSSL error queued"

    # Print
    print(f"{failure} failed (ssl_error={err_code}): {details}")

def _close():
    """
    Closes the session, context and socket, safely freeing memory
    """
    # Global variables
    global _socket, _context, _session, _active

    # Close the session
    if _session:
        if _active:
            _ssl["SSL_shutdown"](_session)  # send "close_notify", don't wait for reply
        _ssl["SSL_free"](_session) # free memory
        _session = None
    _active = False

    # Close context
    if _context:
        _ssl["SSL_CTX_free"](_context)
        _context = None

    # Close client socket
    if _socket:
        _socket.close()
        _socket = None

    # Empty the error queue
    _ssl_errors_to_str()

def _discover() -> Optional[tuple]:
    """
    Looks for the ESP32 on the local network via mDNS (DNS-SD service lookup).
    This blocks for up to `MDNS_TIMEOUT_MS`, so it must only be used when (re)connecting, never while sending.

    Returns:
        tuple[str,int] | None: The IPv4 address and port of the ESP32, or None if it didn't answer
    """
    # Query for the service; the instance name defaults to the hostname on the ESP32
    zc = Zeroconf(ip_version=IPVersion.V4Only)
    try:
        info = zc.get_service_info(MDNS_SERVICE, f"{MDNS_HOSTNAME}.{MDNS_SERVICE}", timeout=MDNS_TIMEOUT_MS)
    finally:
        zc.close()

    # Extract the address and port
    if info is None or info.port is None:
        if info is None:
            print("Info is None")
        return None
    addresses = info.parsed_addresses(IPVersion.V4Only)
    if len(addresses) == 0:
        return None
    return addresses[0], info.port

# API FUNCTIONS ================================

def init_net():
    """
    Initializes network module by creating the C bindings to the OpenSSL libraries

    Raises:
       SystemExit: If the network module cannot be initialized
    """
    # Global variables
    global _active

    # Assume session is inactive
    _active = False

    # Create C type function bindings
    _create_c_bindings()

def connect_net() -> bool:
    """
    Attempts to find the ESP32 via mDNS, and then connect to it.
    Any session that was already open is closed first.

    Returns:
        bool: True if the connection was successful, False otherwise
    """
    # Global variables
    global _socket, _context, _session, _psk_cb, _active, _next_seq_num, _target

    # Initialize
    _close() # clean up any previous session
    _next_seq_num = 1
    try:
        # Find the ESP32
        if ESP32_IPV4_OVERRIDE is not None:
            target = (ESP32_IPV4_OVERRIDE, PORT)
        else:
            target = _discover()
            if target is None:
                if _target is None:
                    print("ESP32 not found via mDNS")
                    return False
                target = _target # missed this time; fall back to the last address that worked
                print(f"ESP32 not found via mDNS, trying last known address {target[0]}")
            _target = target
        esp32_ip, esp32_port = target

        # Connect to a UDP socket
        _socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _socket.connect((esp32_ip, esp32_port))

        # Create context
        _context = _ssl["SSL_CTX_new"](_ssl["DTLS_client_method"]())
        if not _context: # if creating the context failed
            _print_failure("SSL_CTX_new")
            raise Exception
        # Set protocol to DTLS v1.2
        if (_ssl["SSL_CTX_ctrl"](_context, SSL_CTRL_SET_MIN_PROTO_VERSION, DTLS1_2_VERSION, None) != 1
                or _ssl["SSL_CTX_ctrl"](_context, SSL_CTRL_SET_MAX_PROTO_VERSION, DTLS1_2_VERSION, None) != 1):
            _print_failure("setting DTLS 1.2 as the protocol version")
            raise Exception
        # Set cipher
        if _ssl["SSL_CTX_set_cipher_list"](_context, CIPHER) != 1:
            _print_failure("SSL_CTX_set_cipher_list")
            raise Exception

        # Give a PSK callback function
        def psk_cb(ssl, hint, id_buffer, id_max_len, psk_buffer, psk_max_len) -> int:
            """
            Writes the PSK identity and value to the given buffer,
            when OpenSSL calls for the PSK during handshakes
            """
            try:
                # Check buffer lengths
                if len(PSK_ID)+1 > id_max_len or len(PSK) > psk_max_len:
                    return 0

                # Copy PSK identity and value over
                ctypes.memmove(id_buffer, PSK_ID+b"\0", len(PSK_ID)+1)
                ctypes.memmove(psk_buffer, PSK, len(PSK))
                return len(PSK)

            except Exception:
                return 0  # unknown failure; abort the handshake
        _psk_cb = _PSK_CLIENT_CB(psk_cb)
        _ssl["SSL_CTX_set_psk_client_callback"](_context, _psk_cb)

        # Create session
        _session = _ssl["SSL_new"](_context)
        if not _session:
            _print_failure("SSL_new")
            raise Exception

        # Create basic input/output for the socket
        bio = _crypto["BIO_new_dgram"](_socket.fileno(), BIO_NOCLOSE) # NOCLOSE: Python keeps ownership of the fd
        if not bio:
            _print_failure("BIO_new_dgram")
            raise Exception
        # Create BIO address
        address = _crypto["BIO_ADDR_new"]()
        _crypto["BIO_ADDR_rawmake"](address, socket.AF_INET, socket.inet_aton(esp32_ip), 4, socket.htons(esp32_port))
        _crypto["BIO_ctrl"](bio, BIO_CTRL_DGRAM_SET_CONNECTED, 0, address)  # the BIO copies the address
        _crypto["BIO_ADDR_free"](address) # free the address
        # Give the BIO to the session
        _ssl["SSL_set_bio"](_session, bio, bio)

        # Handshake (OpenSSL handles cookie exchange and retransmission internally)
        ret = _ssl["SSL_connect"](_session)
        if ret != 1:
            _print_failure("SSL_connect", ret)
            raise Exception
        _active = True

        # Get cipher name (not needed in current implementation)
        name = _ssl["SSL_CIPHER_get_name"](_ssl["SSL_get_current_cipher"](_session))
        cipher = "?"
        if name:
            cipher = name.decode()

    # Catch ordinary failures (but let Ctrl+C through)
    except Exception as e:
        if str(e): # failures reported above raise a bare Exception; others (e.g. socket or mDNS errors) carry a message
            print(f"connecting failed: {e}")
        _close() # cleanup
        return False

    # Indicate that the connection was successful
    return True

def send_setpoints(setpoints) -> bool:
    """
    Sends a DTLS packet to the ESP32 with the given setpoints.<br>
    This function must not be called too frequently, to give the network time to catch up.<br>
    A period of 10 milliseconds should be slow enough

    Args:
        setpoints (Sequence[float]): The setpoints to send, which must be 6 finite numbers in degrees,
            within `ANGLE_MIN` to `ANGLE_MAX`
    Raises:
        ValueError: If the setpoints are not 6 finite numbers within range
    Returns:
        bool: True if the packet was sent, False if there is no active session or the send failed
            (in which case the caller should reconnect)
    """
    # Global variables
    global _next_seq_num

    # Validate input (raises if the values can't be converted to floats)
    if len(setpoints) != 6:
        raise ValueError(f"expected 6 setpoints, got {len(setpoints)}")
    values = [float(setpoint) for setpoint in setpoints]
    for value in values:
        if not (math.isfinite(value) and ANGLE_MIN <= value <= ANGLE_MAX):
            raise ValueError(f"setpoints must be finite and within [{ANGLE_MIN}, {ANGLE_MAX}], got {values}")

    # Can't send without a session
    if not _active:
        return False

    # Send packet
    payload = FRAME.pack(_next_seq_num, *values)
    ret = _ssl["SSL_write"](_session, payload, len(payload))
    if ret != len(payload):
        _print_failure("SSL_write", ret)
        return False

    # Increment sequence number, with wrap-around
    _next_seq_num = (_next_seq_num+1) & 0xFFFF
    return True

def close_net():
    """
    Closes the connection (sending a close_notify to the ESP32) and frees all resources
    """
    _close()

# MAIN ================================

def main():
    # Pick sepoints
    setpoints = [90.0, 90.0, 90.0, 90.0, 90.0, 180.0]

    # Initialization
    init_net()

    try:
        while True:
            # Try to connect
            while not connect_net():
                print("Retrying to connect in 1 second")
                time.sleep(1.0)

            # Continuously send setpoint until sending fails
            while send_setpoints(setpoints):
                time.sleep(0.01)

            # Sending failed
            print("Lost connection, reconnecting in 1 second")
            close_net()
            time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        close_net()

if __name__ == "__main__":
    main()