################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import json
import re
import socket
import subprocess
from pathlib import Path
from typing import Optional


# Path to cached MAC address
CACHE_PATH = Path.home() / ".cache" / "wol_mac_cache.json"


class WolServer:
    """
    Send Wake-on-LAN (WoL) magic packets to a given computer.

    :param hostname: Hostname of the target computer

    :return: MAC address of the target computer, or "" if not found
    """

    @staticmethod
    def get_mac_address(hostname: str) -> str:
        mac_address: str = ""
        mac_address_cache: dict[str, str] = {}

        try:
            # DNS -> IP
            ip_address: str = socket.gethostbyname(hostname)

            # Ping once (silently)
            subprocess.run(
                ["ping", "-c", "1", ip_address],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )

            # Pull ARP entry
            output: str = subprocess.check_output(
                ["ip", "neigh", "show", ip_address],
                text=True,
            ).lower()

            # Extract MAC address
            mac_match: Optional[re.Match[str]] = re.search(
                r"([\da-f]{2}(?::[\da-f]{2}){5})", output
            )
            if mac_match:
                mac_address = mac_match.group(1)
        except Exception:
            # Ignore errors
            pass

        # Load MAC address cache
        try:
            with open(CACHE_PATH, "r") as cache_file:
                mac_address_cache = json.load(cache_file)
        except (OSError, json.JSONDecodeError):
            pass

        # Update MAC address cache
        if mac_address:
            if mac_address != mac_address_cache.get(hostname):
                mac_address_cache[hostname] = mac_address
                with open(CACHE_PATH, "w") as cache_file:
                    json.dump(mac_address_cache, cache_file, indent=2)
        else:
            # If no MAC address was found, check the cache
            mac_address = mac_address_cache.get(hostname, "")

        return mac_address

    @staticmethod
    def send_wol(mac_address: str) -> None:
        """
        Send a WoL magic packet to the specified MAC address.

        :param mac_address: MAC address of the target, in any of these forms:
            - "AA:BB:CC:DD:EE:FF"
            - "AABBCCDDEEFF"
            - "AA-BB-CC-DD-EE-FF"

        :raises ValueError: if the MAC address is malformed
        :raises OSError: on socket errors
        """
        # 1) Normalize: strip out everything but hex digits
        cleaned: str = re.sub(r"[^0-9A-Fa-f]", "", mac_address)
        if len(cleaned) != 12:
            raise ValueError(f"Invalid MAC address '{mac_address}'")

        # 2) Build the payload: 6 × 0xFF followed by 16 × the MAC bytes
        mac_bytes: bytes = bytes.fromhex(cleaned)
        packet: bytes = b"\xFF" * 6 + mac_bytes * 16

        # 3) Send as a UDP broadcast on port 9
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(packet, ("255.255.255.255", 9))
