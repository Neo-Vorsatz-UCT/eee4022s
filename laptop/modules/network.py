"""
    Module to handle the networking - send the setpoints to the ESP32 over the network via DTLS1.2 (_PSK)
    Author: Neo Vorsatz
    Last updated: 14 October 2026
"""

# IMPORTS ================================

# External
import ctypes # interacting with C libraries
import socket # networking with socket of ESP32
import struct # packing data in raw bytes
import sys # aborting program if libraries don't exist
from zeroconf import IPVersion, Zeroconf # mDNS

# TYPES ================================

_vp = ctypes.c_void_p # C void pointer
_PSK_CLIENT_CB = ctypes.CFUNCTYPE(ctypes.c_uint, _vp, _vp, _vp, ctypes.c_uint, _vp, ctypes.c_uint) # callback

# CONSTANTS ================================

# Security
_PSK = bytes.fromhex("c7676ca068aff229e5ac4e3f8e372103")
_PSK_ID = b"eee4022s-braccio-psk"
_CIPHER = b"PSK-AES128-GCM-SHA256"

# Target server
"""
    By default, many Wirewalls block mDNS services (on port 5353).
    To temporarily bypass this on linux, run the command:
    sudo firewall-cmd --zone=public --add-service=mdns
    When you reboot your system, this change is automatically reverted.
"""
_MDNS_HOSTNAME = "eee4022s-braccio"
_MDNS_SERVICE = "_actuation._udp.local."
_MDNS_TIMEOUT_MS = 2000 # timeout for ESP32 to respond to lookup
_ESP32_IPV4_OVERRIDE = "192.168.0.83" # set to `None` to use mDNS, and set to an IPv4 string to override mDNS
_PORT = 42069 # used if overriding mDNS

# OpenSSL constants, from ssl.h and bio.h
_DTLS1_2_VERSION = 0xFEFD
_SSL_CTRL_SET_MIN_PROTO_VERSION = 123
_SSL_CTRL_SET_MAX_PROTO_VERSION = 124
_BIO_NOCLOSE = 0
_BIO_CTRL_DGRAM_SET_CONNECTED = 32

# VARIABLES ================================

# Library bindings
_ssl:dict = {}
_crypto:dict = {}

# Session and context
_target:tuple[str,int] = None # last known (ip, port) of the ESP32, from mDNS
_socket = None # client socket
_context = None # context
_session = None # session
_psk_cb = None # _PSK callback - not used, but must avoid garbage collector
_active = False # whether the session is active or not
_next_seq_num:int = 1 # next sequence number for packets

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
    # Global variables
    global _ssl

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

def _discover() -> tuple[str,int] | None:
    """
    Looks for the ESP32 on the local network via mDNS (DNS-SD service lookup).
    This blocks for up to `_MDNS_TIMEOUT_MS`, so it must only be used when (re)connecting, never while sending
 
    Returns:
        tuple[str,int] | None: The IPv4 address and port of the ESP32, or None if it didn't answer
    """
    # Query for the mDNS service
    zc = Zeroconf(ip_version=IPVersion.V4Only)
    try:
        info = zc.get_service_info(_MDNS_SERVICE, f"{_MDNS_HOSTNAME}.{_MDNS_SERVICE}", timeout=_MDNS_TIMEOUT_MS)
    finally:
        zc.close()
 
    # Validate the address and port
    if info is None or info.port is None:
        return None
    addresses = info.parsed_addresses(IPVersion.V4Only)
    if len(addresses) == 0:
        return None

    # Return
    return addresses[0], info.port

# API FUNCTIONS ================================

def init_net():
    """
    Initializes network module - TODO: elaborate

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
    Attempts to connect to the ESP32.

    Returns:
        bool: True if the connection was successful, False otherwise
    """
    # Global variables
    global _ssl, _crypto, _target, _socket, _context, _session, _psk_cb, _active, _next_seq_num
    
    # Initialize
    close_net() # cleanup a session if currently active
    _next_seq_num = 1
    try:
        # Find the ESP32
        if _ESP32_IPV4_OVERRIDE is not None:
            target = (_ESP32_IPV4_OVERRIDE, _PORT)
        else:
            target = _discover()
            if target is None:
                # If a socket was never found
                if _target is None:
                    print("ESP32 not found via mDNS")
                    return False

                # Fall back to last socket that worked
                target = _target
                print(f"ESP32 not found via mDNS, trying last known address {target[0]}")

            # Update known socket
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
        if (_ssl["SSL_CTX_ctrl"](_context, _SSL_CTRL_SET_MIN_PROTO_VERSION, _DTLS1_2_VERSION, None) != 1
                or _ssl["SSL_CTX_ctrl"](_context, _SSL_CTRL_SET_MAX_PROTO_VERSION, _DTLS1_2_VERSION, None) != 1):
            _print_failure("setting DTLS 1.2 as the protocol version")
            raise Exception
        # Set cipher
        if _ssl["SSL_CTX_set_cipher_list"](_context, _CIPHER) != 1:
            _print_failure("SSL_CTX_set_cipher_list")
            raise Exception

        # Give a _PSK callback function
        def psk_cb(ssl, hint, id_buffer, id_max_len, psk_buffer, psk_max_len) -> int:
            """
            Writes the _PSK identity and value to the given buffer,
            when OpenSSL calls for the _PSK during handshakes
            """
            try:
                # Check buffer lengths
                if len(_PSK_ID)+1 > id_max_len or len(_PSK) > psk_max_len:
                    return 0

                # Copy _PSK identity and value over
                ctypes.memmove(id_buffer, _PSK_ID+b"\0", len(_PSK_ID)+1)
                ctypes.memmove(psk_buffer, _PSK, len(_PSK))
                return len(_PSK)
            
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
        bio = _crypto["BIO_new_dgram"](_socket.fileno(), _BIO_NOCLOSE) # NOCLOSE: Python keeps ownership of the fd
        if not bio:
            _print_failure("BIO_new_dgram")
            raise Exception
        # Create BIO address
        address = _crypto["BIO_ADDR_new"]()
        _crypto["BIO_ADDR_rawmake"](address, socket.AF_INET, socket.inet_aton(esp32_ip), 4, socket.htons(esp32_port))
        _crypto["BIO_ctrl"](bio, _BIO_CTRL_DGRAM_SET_CONNECTED, 0, address)  # the BIO copies the address
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
    
    # Catch exceptions
    except Exception as e:
        if str(e):
            print(f"connecting failed: {e}")
        close_net() # cleanup
        return False

    # Indicate that the connection was successful
    return True

def send_setpoints(setpoints:list[float]):
    """
    Sends a DTLS packet to the ESP32 with the given setpoints.<br>
    This function must not be called too frequently, to give the network time to catch up.<br>
    A period of 10 milliseconds should be slow enough

    Args:
        setpoints (list[float]): The setpoints to send, which must be 6 floats
    """
    # Global variables
    global _ssl, _session, _next_seq_num

    # Validate input
    if len(setpoints) != 6:
        return
    for setpoint in setpoints:
        if not isinstance(setpoint, float):
            return

    # Send packet
    FRAME = struct.Struct("<H6f")  # 2 bytes of sequencing + 6 float32 setpoints = 26 bytes
    payload = FRAME.pack(_next_seq_num, *setpoints)
    ret = _ssl["SSL_write"](_session, payload, len(payload))
    if ret != len(payload):
        _print_failure("SSL_write", ret)

    # Increment sequence number, with wrap-around
    _next_seq_num = (_next_seq_num+1) & 0xFFFF

def close_net():
    """
    Closes the connection by sending close_notify, and safely freeing memory
    """
    # Global variables
    global _ssl, _socket, _context, _session, _active

    # Close the session
    if _session:
        if _active:
            _ssl["SSL_shutdown"](_session)  # send "close_notify", don't wait for reply
            _active = False
        _ssl["SSL_free"](_session) # free memory
        _session = None

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

# MAIN ================================

if __name__ == "__main__":
    # Imports
    import time # timing

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

            # Continuously send setpoint
            while True:
                try:
                    send_setpoints(setpoints)
                    time.sleep(0.01)
                except OSError as e:
                    print(f"Failed to send packet due to error: {e}\nTrying to connect in 1 second")
                    time.sleep(1.0)
                    break

    # Handle exit when using Ctrl+C in the terminal
    except KeyboardInterrupt:
        pass
    finally:
        close_net()
