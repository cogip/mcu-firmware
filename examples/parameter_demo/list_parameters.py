#!/usr/bin/env python3
"""Configure the parameters of parameter_demo from their protobuf schema, as a host tool would.

The firmware sends the identity of each schema it exposes (PB_ParameterSchema): the FNV-1a hash of
the schema message full name and the schema version. The host finds the schema message in its
descriptors (PB_ParameterDemo.proto), which give the parameters, their types, units and
descriptions, and addresses a parameter with the FNV-1a hash of its field name, in the messages of
PB_ParameterCommands.proto.

Note: run it after building the example: the build generates the `*_pb2.py` modules next to it.
"""

import sys
from pathlib import Path
from types import ModuleType

sys.path.insert(0, str(Path(__file__).parent))

import PB_ParameterCommands_pb2
import PB_ParameterDemo_pb2
import PB_ParameterOptions_pb2
from google.protobuf.descriptor import Descriptor, FieldDescriptor

FNV1A_OFFSET = 0x811C9DC5
FNV1A_PRIME = 0x01000193

# PB_ParameterValue field holding each value type
VALUE_FIELDS = {
    FieldDescriptor.TYPE_FLOAT: "float_value",
    FieldDescriptor.TYPE_DOUBLE: "double_value",
    FieldDescriptor.TYPE_INT32: "int32_value",
    FieldDescriptor.TYPE_UINT32: "uint32_value",
    FieldDescriptor.TYPE_INT64: "int64_value",
    FieldDescriptor.TYPE_UINT64: "uint64_value",
    FieldDescriptor.TYPE_BOOL: "bool_value",
}


def fnv1a_hash(key: str) -> int:
    """Compute the 32-bit FNV-1a hash of a key, like `_key_hash` in KeyHash.hpp.

    Args:
        key: Key to hash.

    Returns:
        32-bit hash.
    """
    value = FNV1A_OFFSET
    for byte in key.encode():
        value = ((value ^ byte) * FNV1A_PRIME) & 0xFFFFFFFF
    return value


def schema_version(schema: Descriptor) -> int:
    """Get the version of a schema message.

    Args:
        schema: Schema message descriptor.

    Returns:
        Version from the `parameter_schema` option.
    """
    return schema.GetOptions().Extensions[PB_ParameterOptions_pb2.parameter_schema].version


def known_schemas(*modules: ModuleType) -> dict[int, Descriptor]:
    """Index the schema messages known by the host by schema id.

    Args:
        modules: Generated `*_pb2` modules declaring schema messages.

    Returns:
        Schema message descriptors, by FNV-1a hash of their full name.
    """
    schemas = {}
    for module in modules:
        for message in module.DESCRIPTOR.message_types_by_name.values():
            if message.GetOptions().HasExtension(PB_ParameterOptions_pb2.parameter_schema):
                schemas[fnv1a_hash(message.full_name)] = message
    return schemas


def find_schema(identity: PB_ParameterCommands_pb2.PB_ParameterSchema, schemas: dict[int, Descriptor]) -> Descriptor:
    """Find the schema message matching a schema exposed by the firmware.

    Args:
        identity: Schema identity sent by the firmware.
        schemas: Schema messages known by the host, by schema id.

    Returns:
        Schema message descriptor.

    Raises:
        ValueError: If the host does not know this schema or has another version of it.
    """
    schema = schemas.get(identity.id)
    if schema is None:
        raise ValueError(f"unknown schema 0x{identity.id:08x}")
    if schema_version(schema) != identity.version:
        raise ValueError(f"{schema.full_name}: firmware version {identity.version}, host {schema_version(schema)}")
    return schema


def parameter_meta(field: FieldDescriptor) -> PB_ParameterOptions_pb2.PB_ParameterMeta:
    """Get the description of a parameter.

    Args:
        field: Parameter field.

    Returns:
        Content of the `parameter_meta` option, empty if not set.
    """
    return field.GetOptions().Extensions[PB_ParameterOptions_pb2.parameter_meta]


def describe_field(field: FieldDescriptor) -> str:
    """Describe a parameter.

    Args:
        field: Parameter field.

    Returns:
        One line description.
    """
    meta = parameter_meta(field)
    details = [f"key=0x{fnv1a_hash(field.name):08x}"]
    if meta.unit:
        details.append(f"unit={meta.unit}")
    description = f" - {meta.description}" if meta.description else ""
    return f"{field.name:<24} {VALUE_FIELDS[field.type].removesuffix('_value'):<6} {' '.join(details)}{description}"


def build_set_request(
    schema: Descriptor, name: str, value: float | int | bool
) -> PB_ParameterCommands_pb2.PB_ParameterSetRequest:
    """Build the request setting a parameter.

    Args:
        schema: Schema message descriptor.
        name: Parameter name.
        value: New value, of the parameter type.

    Returns:
        Request to send to the firmware.
    """
    field = schema.fields_by_name[name]
    request = PB_ParameterCommands_pb2.PB_ParameterSetRequest(key_hash=fnv1a_hash(name))
    setattr(request.value, VALUE_FIELDS[field.type], value)
    return request


def main() -> None:
    """Find the schema exposed by the firmware, list its parameters, then build set requests."""
    schemas = known_schemas(PB_ParameterDemo_pb2)

    # Schema identity sent by the firmware in reply to a schema request (see the demo output)
    demo = PB_ParameterDemo_pb2.PB_ParameterDemo.DESCRIPTOR
    identity = PB_ParameterCommands_pb2.PB_ParameterSchema(id=fnv1a_hash(demo.full_name), version=schema_version(demo))

    schema = find_schema(identity, schemas)
    print(f"Schema {schema.full_name} (id 0x{identity.id:08x}) version {identity.version}: {len(schema.fields)} parameters")
    for field in schema.fields:
        print(f"  {describe_field(field)}")

    print()
    for name, value in (("max_angular_velocity", 360.0), ("enable_logging", False)):
        request = build_set_request(schema, name, value)
        print(f"Set {name} = {value}: {request.SerializeToString().hex()} ({request.ByteSize()} bytes)")


if __name__ == "__main__":
    main()
