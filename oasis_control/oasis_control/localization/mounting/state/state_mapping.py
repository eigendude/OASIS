################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""State vector block layout for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass

from oasis_control.localization.mounting.state.mounting_state import MountingState


# Block name for IMU-to-body rotation increment
BLOCK_NAME_R_BI: str = "R_BI"

# Block name for magnetometer-to-body rotation increment
BLOCK_NAME_R_BM: str = "R_BM"

# Block name for gravity direction increment
BLOCK_NAME_G_W: str = "g_W"

# Block name for magnetic direction increment
BLOCK_NAME_M_W: str = "m_W"

# Block name for accelerometer bias increment
BLOCK_NAME_B_A: str = "b_a"

# Block name for accelerometer scale/misalignment increment
BLOCK_NAME_A_A: str = "A_a"

# Block name for gyroscope bias increment
BLOCK_NAME_B_G: str = "b_g"

# Block name for magnetometer bias increment
BLOCK_NAME_B_M: str = "b_m"

# Prefix for keyframe attitude block names
BLOCK_NAME_R_WB_PREFIX: str = "R_WB"


class StateMappingError(Exception):
    """Raised when state mapping construction is invalid."""


@dataclass(frozen=True)
class StateBlock:
    """Represents a named block in the parameter vector.

    Attributes:
        name: Block name identifier
        start: Starting index in the parameter vector
        dim: Block dimension
    """

    name: str
    start: int
    dim: int

    def stop(self) -> int:
        """Return the exclusive stop index for the block."""
        return self.start + self.dim

    def sl(self) -> slice:
        """Return the slice covering the block indices."""
        return slice(self.start, self.stop())

    def validate(self) -> None:
        """Validate block indices and dimensions."""
        if not self.name:
            raise StateMappingError("Block name must be non-empty")
        if self.start < 0:
            raise StateMappingError("Block start must be non-negative")
        if self.dim <= 0:
            raise StateMappingError("Block dim must be positive")


@dataclass(frozen=True)
class StateMapping:
    """Deterministic block layout for solver state vectors."""

    _blocks: tuple[StateBlock, ...]
    _keyframe_ids: tuple[int, ...]

    @classmethod
    def from_state(
        cls,
        state: MountingState,
    ) -> StateMapping:
        """Construct a state mapping from a mounting state."""
        if not isinstance(state, MountingState):
            raise StateMappingError("state must be a MountingState")
        blocks: list[StateBlock] = []
        offset: int = 0

        def _add_block(name: str, dim: int) -> None:
            nonlocal offset
            block: StateBlock = StateBlock(name=name, start=offset, dim=dim)
            block.validate()
            blocks.append(block)
            offset += dim

        _add_block(BLOCK_NAME_R_BI, 3)
        _add_block(BLOCK_NAME_R_BM, 3)
        _add_block(BLOCK_NAME_G_W, 3)
        if state.m_W_unit is not None:
            _add_block(BLOCK_NAME_M_W, 3)
        _add_block(BLOCK_NAME_B_A, 3)
        _add_block(BLOCK_NAME_A_A, 9)
        _add_block(BLOCK_NAME_B_G, 3)
        if state.mag.b_m_T is not None:
            _add_block(BLOCK_NAME_B_M, 3)
        keyframe_ids: tuple[int, ...] = state.keyframe_ids()
        for keyframe_id in keyframe_ids:
            _add_block(f"{BLOCK_NAME_R_WB_PREFIX}_{keyframe_id}", 3)
        mapping: StateMapping = cls(
            _blocks=tuple(blocks),
            _keyframe_ids=keyframe_ids,
        )
        mapping.validate()
        return mapping

    def dim(self) -> int:
        """Return the total dimension of the parameter vector."""
        if not self._blocks:
            return 0
        return self._blocks[-1].stop()

    def blocks(self) -> tuple[StateBlock, ...]:
        """Return the blocks in deterministic order."""
        return self._blocks

    def block(self, name: str) -> StateBlock:
        """Return the block with the given name."""
        for block in self._blocks:
            if block.name == name:
                return block
        raise StateMappingError(f"Block {name} not found")

    def has(self, name: str) -> bool:
        """Return True when the mapping contains the named block."""
        return any(block.name == name for block in self._blocks)

    def keyframe_block(self, keyframe_id: int) -> StateBlock:
        """Return the block for a keyframe attitude."""
        name: str = f"{BLOCK_NAME_R_WB_PREFIX}_{keyframe_id}"
        return self.block(name)

    def validate(self) -> None:
        """Validate the layout and keyframe ordering."""
        offset: int = 0
        for block in self._blocks:
            block.validate()
            if block.start != offset:
                raise StateMappingError("Blocks must be contiguous")
            offset = block.stop()
        if self._blocks and offset != self.dim():
            raise StateMappingError("Block layout does not match dimension")
        keyframe_blocks: list[str] = [
            block.name
            for block in self._blocks
            if block.name.startswith(f"{BLOCK_NAME_R_WB_PREFIX}_")
        ]
        expected: list[str] = [
            f"{BLOCK_NAME_R_WB_PREFIX}_{keyframe_id}"
            for keyframe_id in self._keyframe_ids
        ]
        if keyframe_blocks != expected:
            raise StateMappingError("Keyframe blocks do not match state order")
        if len(set(self._keyframe_ids)) != len(self._keyframe_ids):
            raise StateMappingError("Keyframe ids must be unique")
