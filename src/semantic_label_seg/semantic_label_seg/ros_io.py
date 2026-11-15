"""ROS message conversion without ros_numpy/cv_bridge or fixed point strides."""

from copy import deepcopy

import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField

_FIELD_TYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


def cloud_view(message):
    """Read an organized cloud, including row padding and big endian fields."""
    names, formats, offsets = [], [], []
    byteorder = ">" if message.is_bigendian else "<"
    for field in message.fields:
        if field.name in names or field.datatype not in _FIELD_TYPES or field.count < 1:
            raise ValueError("unsupported or duplicate PointCloud2 field")
        base = np.dtype(byteorder + _FIELD_TYPES[field.datatype])
        names.append(field.name)
        formats.append(base if field.count == 1 else (base, (field.count,)))
        offsets.append(field.offset)
        if (
            field.offset < 0
            or field.offset + base.itemsize * field.count > message.point_step
        ):
            raise ValueError("PointCloud2 field extends beyond point_step")
    if message.height < 1 or message.point_step < 1:
        raise ValueError("PointCloud2 needs positive height and point_step")
    if message.row_step < message.width * message.point_step:
        raise ValueError("PointCloud2 row_step is too small")
    if len(message.data) < message.row_step * message.height:
        raise ValueError("PointCloud2 data buffer is truncated")
    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": message.point_step,
        }
    )
    return np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=message.data,
        strides=(message.row_step, message.point_step),
    )


def cloud_to_arrays(message):
    view = cloud_view(message)
    for field in ("x", "y", "z"):
        if field not in view.dtype.names or view[field].ndim != 2:
            raise ValueError("PointCloud2 requires scalar x,y,z fields")
    xyz = np.stack([view[f].reshape(-1) for f in ("x", "y", "z")], axis=1).astype(
        np.float32
    )
    intensity = (
        view["intensity"].reshape(-1).astype(np.float32)
        if "intensity" in view.dtype.names
        else np.zeros(len(xyz), dtype=np.float32)
    )
    return xyz, intensity


def with_labels(message, labels, confidence, instance_ids=None):
    """Append explicit labels while retaining all other fields and original XYZ.

    Existing semantic fields are replaced; other fields including intensity,
    ring, time and vector-valued fields retain their bytes and original offsets.
    Output rows are packed, point ordering/organization and Header are preserved.
    """
    cloud_view(message)  # validate input before accessing its bytes
    n = message.width * message.height
    values = {"label": np.asarray(labels), "confidence": np.asarray(confidence)}
    if instance_ids is not None:
        values["instance_id"] = np.asarray(instance_ids)
    for name, value in values.items():
        if value.size != n:
            raise ValueError(f"{name} must contain exactly {n} elements")
        if name != "confidence" and (
            value.dtype.kind not in "ui"
            or (value > np.iinfo(np.uint32).max).any()
            or (value < 0).any()
        ):
            raise ValueError(f"{name} must contain uint32-compatible integers")
    score = values["confidence"]
    if not np.isfinite(score).all() or (score < 0).any() or (score > 1).any():
        raise ValueError("confidence must be finite in [0,1]")
    fields = [deepcopy(f) for f in message.fields if f.name not in values]
    point_step = message.point_step
    for name in values:
        datatype = PointField.FLOAT32 if name == "confidence" else PointField.UINT32
        fields.append(
            PointField(name=name, offset=point_step, datatype=datatype, count=1)
        )
        point_step += 4
    raw = np.zeros((message.height, message.width, point_step), dtype=np.uint8)
    original = np.ndarray(
        (message.height, message.width, message.point_step),
        dtype=np.uint8,
        buffer=message.data,
        strides=(message.row_step, message.point_step, 1),
    )
    raw[:, :, : message.point_step] = original
    result = PointCloud2(
        header=deepcopy(message.header),
        height=message.height,
        width=message.width,
        fields=fields,
        is_bigendian=message.is_bigendian,
        point_step=point_step,
        row_step=point_step * message.width,
        data=raw.tobytes(),
        is_dense=message.is_dense,
    )
    # Fill using a writable view before assigning its final byte buffer.
    dtype_names = list(values)
    byteorder = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": dtype_names,
            "formats": [
                byteorder + ("f4" if name == "confidence" else "u4")
                for name in dtype_names
            ],
            "offsets": [message.point_step + i * 4 for i in range(len(values))],
            "itemsize": point_step,
        }
    )
    output = raw.reshape(-1, point_step).view(dtype).reshape(-1)
    for name, value in values.items():
        output[name] = value.reshape(-1)
    result.data = raw.tobytes()
    return result


def xyz_cloud(points, header):
    points = np.asarray(points, dtype="<f4")
    if points.ndim == 2 and points.shape[1] == 3:
        points = points[None]
    if points.ndim != 3 or points.shape[2] != 3:
        raise ValueError("expected Nx3 or HxWx3 points")
    height, width, _ = points.shape
    return PointCloud2(
        header=deepcopy(header),
        height=height,
        width=width,
        fields=[
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(("x", "y", "z"))
        ],
        is_bigendian=False,
        point_step=12,
        row_step=12 * width,
        data=np.ascontiguousarray(points).tobytes(),
        is_dense=bool(np.isfinite(points).all()),
    )


def image_to_array(message):
    formats = {
        "bgr8": ("u1", 3),
        "rgb8": ("u1", 3),
        "mono8": ("u1", 1),
        "16UC1": ("u2", 1),
        "32FC1": ("f4", 1),
        "32SC1": ("i4", 1),
    }
    if message.encoding not in formats:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    code, channels = formats[message.encoding]
    dtype = np.dtype((">" if message.is_bigendian else "<") + code)
    if (
        message.height < 1
        or message.width < 1
        or message.step < message.width * channels * dtype.itemsize
    ):
        raise ValueError("invalid Image dimensions or step")
    if len(message.data) < message.step * message.height:
        raise ValueError("Image data buffer is truncated")
    shape = (
        (message.height, message.width, channels)
        if channels > 1
        else (message.height, message.width)
    )
    strides = (
        (message.step, channels * dtype.itemsize, dtype.itemsize)
        if channels > 1
        else (message.step, dtype.itemsize)
    )
    return np.ndarray(shape, dtype=dtype, buffer=message.data, strides=strides).astype(
        dtype.newbyteorder("="), copy=True
    )


def array_to_image(array, encoding, header):
    formats = {"32SC1": "<i4", "32FC1": "<f4", "16UC1": "<u2"}
    value = np.asarray(array)
    if value.ndim != 2 or encoding not in formats:
        raise ValueError("expected a supported single-channel HxW image")
    if encoding == "32SC1" and (value > np.iinfo(np.int32).max).any():
        raise ValueError("label exceeds the signed 32-bit ROS image representation")
    value = np.ascontiguousarray(value, dtype=formats[encoding])
    return Image(
        header=deepcopy(header),
        height=value.shape[0],
        width=value.shape[1],
        encoding=encoding,
        is_bigendian=False,
        step=value.shape[1] * value.dtype.itemsize,
        data=value.tobytes(),
    )
