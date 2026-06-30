################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for Wake-on-LAN manager diagnostics."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Optional

from oasis_control.managers.wol_manager import WolManager


class _Future:
    def __init__(
        self,
        *,
        done: bool,
        result: Optional[object] = None,
        exception: Optional[Exception] = None,
    ) -> None:
        self._done: bool = done
        self._result: Optional[object] = result
        self._exception: Optional[Exception] = exception

    def done(self) -> bool:
        return self._done

    def result(self) -> Optional[object]:
        if self._exception is not None:
            raise self._exception

        return self._result


class _Client:
    def __init__(self, futures: Sequence[_Future]) -> None:
        self._futures: list[_Future] = list(futures)
        self.requests: list[object] = []

    def wait_for_service(self, timeout_sec: Optional[float]) -> bool:
        del timeout_sec
        return True

    def call_async(self, request: object) -> _Future:
        self.requests.append(request)
        return self._futures.pop(0)


class _Logger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def debug(self, message: str) -> None:
        self.messages.append(("debug", message))

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def error(self, message: str) -> None:
        self.messages.append(("error", message))


class _Node:
    def __init__(self, clients: Sequence[_Client]) -> None:
        self._clients: list[_Client] = list(clients)
        self.logger: _Logger = _Logger()

    def get_logger(self) -> _Logger:
        return self.logger

    def create_client(self, srv_type: object, srv_name: str) -> _Client:
        del srv_type, srv_name
        return self._clients.pop(0)


class _MacAddressResponse:
    def __init__(self, mac_address: str) -> None:
        self.mac_address: str = mac_address


class _WoLResponse:
    pass


def test_send_wol_logs_mac_lookup_timeout() -> None:
    node: _Node = _make_node(mac_futures=[_Future(done=False)])
    manager: WolManager = WolManager(node, "megapegasus.local")

    assert manager.initialize()
    assert not manager.send_wol()
    assert (
        "error",
        "Timed out after 5.0s getting MAC address for megapegasus.local",
    ) in node.logger.messages


def test_send_wol_logs_mac_lookup_exception() -> None:
    node: _Node = _make_node(
        mac_futures=[_Future(done=True, exception=RuntimeError("boom"))]
    )
    manager: WolManager = WolManager(node, "megapegasus.local")

    assert manager.initialize()
    assert not manager.send_wol()
    assert (
        "error",
        "get_mac_address service failed for megapegasus.local: boom",
    ) in node.logger.messages


def test_send_wol_logs_mac_lookup_no_response() -> None:
    node: _Node = _make_node(mac_futures=[_Future(done=True, result=None)])
    manager: WolManager = WolManager(node, "megapegasus.local")

    assert manager.initialize()
    assert not manager.send_wol()
    assert (
        "error",
        "get_mac_address service completed with no response for megapegasus.local",
    ) in node.logger.messages


def test_send_wol_logs_empty_mac_address() -> None:
    response: _MacAddressResponse = _MacAddressResponse("")
    node: _Node = _make_node(mac_futures=[_Future(done=True, result=response)])
    manager: WolManager = WolManager(node, "megapegasus.local")

    assert manager.initialize()
    assert not manager.send_wol()
    assert (
        "error",
        "get_mac_address service returned an empty MAC address for megapegasus.local",
    ) in node.logger.messages


def test_send_wol_logs_invalid_mac_address() -> None:
    response: _MacAddressResponse = _MacAddressResponse("not-a-mac")
    node: _Node = _make_node(mac_futures=[_Future(done=True, result=response)])
    manager: WolManager = WolManager(node, "megapegasus.local")

    assert manager.initialize()
    assert not manager.send_wol()
    assert (
        "error",
        "get_mac_address service returned an invalid MAC address "
        "for megapegasus.local: not-a-mac",
    ) in node.logger.messages


def test_send_wol_logs_wol_timeout_with_hostname_and_mac() -> None:
    mac_response: _MacAddressResponse = _MacAddressResponse("aa:bb:cc:dd:ee:ff")
    node: _Node = _make_node(
        mac_futures=[_Future(done=True, result=mac_response)],
        wol_futures=[_Future(done=False)],
    )
    manager: WolManager = WolManager(node, "precision.local")

    assert manager.initialize()
    assert not manager.send_wol()
    assert (
        "error",
        "Timed out after 5.0s sending Wake-on-LAN to "
        "precision.local (aa:bb:cc:dd:ee:ff)",
    ) in node.logger.messages


def test_send_wol_logs_wol_exception_with_hostname_and_mac() -> None:
    mac_response: _MacAddressResponse = _MacAddressResponse("aa:bb:cc:dd:ee:ff")
    node: _Node = _make_node(
        mac_futures=[_Future(done=True, result=mac_response)],
        wol_futures=[_Future(done=True, exception=RuntimeError("boom"))],
    )
    manager: WolManager = WolManager(node, "precision.local")

    assert manager.initialize()
    assert not manager.send_wol()
    assert (
        "error",
        "wol service failed for precision.local (aa:bb:cc:dd:ee:ff): boom",
    ) in node.logger.messages


def test_send_wol_returns_true_on_success() -> None:
    mac_response: _MacAddressResponse = _MacAddressResponse("aa:bb:cc:dd:ee:ff")
    node: _Node = _make_node(
        mac_futures=[_Future(done=True, result=mac_response)],
        wol_futures=[_Future(done=True, result=_WoLResponse())],
    )
    manager: WolManager = WolManager(node, "precision.local")

    assert manager.initialize()
    assert manager.send_wol()


def _make_node(
    *,
    mac_futures: Sequence[_Future],
    wol_futures: Sequence[_Future] = (),
) -> _Node:
    get_mac_client: _Client = _Client(mac_futures)
    wol_client: _Client = _Client(wol_futures)

    return _Node([get_mac_client, wol_client])
