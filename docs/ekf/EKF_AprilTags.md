# EKF AprilTag Design

This document defines the AprilTag portion of the localization EKF.

The AprilTag path is landmark-based and image-space. It does not fuse pre-solved
camera poses as if they were independent global pose measurements. Instead, it
uses shared EKF state, shared covariance, fixed camera extrinsics, and
per-corner reprojection residuals to couple base pose and landmark pose.

This document is the focused measurement and landmark-map contract for the
AprilTag portion of localization. General EKF behavior belongs in `EKF.md`,
and EKF code layout belongs in `EKF_Model.md`.

---

## 1. Inputs and frames

The detector input is:

- `apriltags` (`apriltag_msgs/AprilTagDetectionArray`)

Camera intrinsics input is:

- `camera_info` (`sensor_msgs/CameraInfo`)

Frame policy:

- `camera_link` is the canonical camera frame for image-space AprilTag
  measurements and camera intrinsics
- `camera_info` intrinsics are required before AprilTag updates can run
- `apriltags.header.frame_id == "camera_link"` by policy
- `camera_info.header.frame_id == "camera_link"` by policy
- `T_BC` is the fixed transform from `camera_link` to `base_link`
- the current design assumes zero translation between `camera_link` and
  `imu_link`
- equivalently, camera and IMU origins are treated as co-located by current
  hardware and design policy
- this is a current design simplification, not a generic requirement of the
  estimator
- the camera may still have a fixed rotational offset relative to `base_link`
- the world frame is global and tag landmarks live in `world`

This document is the authoritative camera-measurement policy for the current
EKF design. `EKF.md` keeps only the concise runtime contract.

Detection key:

- `TagKey := (family, id)`

This key is the canonical identity for map lookup, deterministic sorting, and
landmark state ownership.

---

## 2. Landmark representation

Each initialized landmark is stored as:

- `T_WTag(TagKey)`

with a full `6 x 6` covariance block.

Policy:

- tags are world-frame landmarks
- landmark covariance remains full
- landmark covariance is never diagonalized by convenience

This matters because AprilTag updates can couple translation, rotation,
and base pose in ways that a diagonal approximation would lose.

---

## 3. Measurement model

The AprilTag update is based on image corner reprojection error, not on
pre-solved pose fusion.

For each detection:

1. identify `TagKey := (family, id)`
2. recover the current camera pose from `T_WB` and fixed `T_BC`
3. predict the tag corner pixels from `T_WTag(TagKey)` and `camera_info`
   intrinsics in `camera_link`
4. compare predicted and observed corner pixels
5. apply the residual through the shared EKF state

The residual lives in image space because that keeps the camera geometry,
landmark uncertainty, fixed camera extrinsics, and base-pose uncertainty in one
consistent model.

Reprojection policy:

- `camera_link` is the canonical frame for the measurement
- `apriltags.header.frame_id` and `camera_info.header.frame_id` are both
  required to resolve to `camera_link`
- AprilTag reprojection uses intrinsics from `camera_info` together with fixed
  camera extrinsics from `T_BC`
- the current co-located camera and IMU origin assumption is policy, not an
  automatic inference from tags

---

## 4. Detection ordering

Detections from one `apriltags` message are applied sequentially in
deterministic sorted order by `(family, id)`.

This ordering should be stable across runs and replays so that:

- fixed-lag replay is deterministic
- update diagnostics are reproducible
- new landmark initialization order is predictable

If two detections share the same key in one message, the packet should be
rejected or normalized by explicit policy rather than relying on container
iteration order.

---

## 5. New landmark initialization

When `TagKey` is not yet in the map, initialization may bootstrap from the
detection geometry using homography, PnP, or an equivalent camera-space solve.

That bootstrap is only the initializer.

Once the landmark exists, ongoing fusion remains image-space and corner-based.
The estimator should not switch to fusing a separate per-tag pose product as if
it were the primary measurement.

---

## 6. Gating and noise policy

The detector provides quality hints that may affect update acceptance.

Useful fields include:

- `decision_margin`
- `homography`

Recommended use:

- use `decision_margin` as an input to gating or noise inflation
- use homography-derived quality checks when deciding whether initialization is
  trustworthy
- keep the gating policy deterministic and diagnosable

Covariance policy:

- preserve full covariance in the EKF update
- preserve cross-covariance with base pose and landmarks
- do not diagonalize the update path for convenience

---

## 7. Outputs

The outward-facing system may still publish one AprilTag reporting result per
image.

Desired behavior:

- publish one map or pose reporting result for each image or `apriltags` message
- if no tags are visible, still publish the per-image result with an
  appropriate status and an empty detection set

These per-image products are reporting outputs only. The estimator itself
remains a single shared-state EKF with one joint map and one joint covariance.

---

## 8. Implementation notes

The AprilTag path should remain tightly coupled to:

- `camera_info` intrinsics
- fixed `T_BC`
- current base state
- landmark state indexed by `TagKey`

This keeps the implementation aligned with the rest of the localization spec:

- camera geometry in `camera_link`
- world landmarks in `world`
- deterministic replayable updates
- uncertainty preserved as full covariance throughout
