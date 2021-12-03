################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import re
import subprocess
import time

from oasis_msgs.msg import WiFiScanData as WiFiScanDataMsg
from oasis_msgs.msg import WiFiStationData as WiFiStationDataMsg


# Linux parameters
IW_BINARY = "iw"


# TODO: Move matchers to utility module
class line_matcher:
    def __init__(self, regexp, handler):
        self.regexp = re.compile(regexp)
        self.handler = handler


class LineMatchers:
    @staticmethod
    def handle_new_network(line, result, networks):
        networks.append({})
        # group(1) is the mac address
        networks[-1]["Address"] = [int(b, 16) for b in result.group(1).split(":")]

    @staticmethod
    def handle_ssid(line, result, networks):
        # group(1) is the ssid name
        networks[-1]["SSID"] = result.group(1).encode("ascii", "ignore")

    @staticmethod
    def handle_freq(line, result, networks):
        # group(1) is the frequency in MHz
        networks[-1]["channel"] = int(result.group(1))  # TODO

    @staticmethod
    def handle_signal(line, result, networks):
        # group(1) is the signal strength (dBm)
        networks[-1]["signal"] = float(result.group(1))

    @staticmethod
    def handle_last_seen(line, result, networks):
        # group(1) is the age in ms
        networks[-1]["last_seen"] = int(result.group(1))

    @staticmethod
    def handle_unknown(line, result, networks):
        # group(1) is the key, group(2) is the rest of the line
        networks[-1][result.group(1)] = result.group(2)

    @classmethod
    def get_matchers(cls):
        matchers = []

        # Catch the line 'BSS XX:YY:ZZ:AA:BB:CC(on wlan0)'
        matchers.append(
            line_matcher(
                r"^BSS (([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2}))",
                cls.handle_new_network,
            )
        )

        # Catch the line 'SSID: network name'
        matchers.append(line_matcher(r"\s+SSID: (.+)", cls.handle_ssid))

        # Catch the line 'freq: 2412'
        matchers.append(line_matcher(r"\s+freq: (\d+)", cls.handle_freq))

        # Catch the line 'signal: -71.00 dBm'
        matchers.append(line_matcher(r"\s+signal: -([^ ]+) dBm", cls.handle_signal))

        # Catch the line 'signal: -71.00 dBm'
        matchers.append(
            line_matcher(r"\s+last seen: (\d+) ms ago", cls.handle_last_seen)
        )

        # Catch any other line that looks like this:
        # Key:value
        matchers.append(line_matcher(r"\s+([^:]+): (.+)", cls.handle_unknown))

        return matchers


class WiFiInterface:
    def __init__(self, name):
        self._name = name

    def scan(self):
        scan_start = time.now()

        # Run scan
        proc = subprocess.Popen(
            [IW_BINARY, "dev", self._name, "scan"], stdout=subprocess.PIPE
        )
        stdout, stderr = proc.communicate()

        scan_duration = (time.now() - scan_start).to_sec()

        networks = self._get_networks(stdout)
        if networks:
            scan_msg = WiFiScanDataMsg()
            scan_msg.header.stamp = time.now()
            scan_msg.interface = self._name
            scan_msg.scan_duration = scan_duration

            for network in networks:
                station_msg = WiFiStationDataMsg()
                station_msg.mac_address = network["Address"]
                station_msg.ssid = network["SSID"]
                station_msg.channel = network["channel"]
                station_msg.dbm = network["signal"]
                station_msg.age_ms = network["last_seen"]
                scan_msg.stations.append(station_msg)

            return scan_msg

        return None

    @staticmethod
    def _get_networks(stdout):
        networks = []

        lines = stdout.decode("utf-8").split("\n")

        # read each line of output, testing against the matches above
        # in that order (so that the key:value matcher will be tried last)
        matchers = LineMatchers.get_matchers()
        for line in lines:
            for m in matchers:
                result = m.regexp.match(line)
                if result:
                    m.handler(line, result, networks)
                    break

        return networks
