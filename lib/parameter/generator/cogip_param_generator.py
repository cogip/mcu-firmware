#!/usr/bin/env python3
"""Protoc plugin generating the identity and keys of parameter schema messages.

A parameter schema message carries the `parameter_schema` option (PB_ParameterOptions.proto)
and declares one field per firmware parameter, described for host tools by `parameter_meta`.
For each `<File>.proto` holding such messages, the plugin checks the schema and generates
`<File>_parameters.hpp` with a `ParameterSchema<Message>` specialization: schema id (FNV-1a hash
of the message full name) and version, and parameter keys (FNV-1a hash of the field names).
"""

import sys
from dataclasses import dataclass
from pathlib import PurePosixPath

from google.protobuf import descriptor_pool, message_factory
from google.protobuf.compiler import plugin_pb2
from google.protobuf.descriptor_pb2 import DescriptorProto, FieldDescriptorProto, FileDescriptorProto

SCHEMA_OPTION = "parameter_schema"
FNV1A_OFFSET = 0x811C9DC5
FNV1A_PRIME = 0x01000193

# Value types of PB_ParameterValue (PB_ParameterCommands.proto)
VALUE_TYPES = {
    FieldDescriptorProto.TYPE_FLOAT,
    FieldDescriptorProto.TYPE_DOUBLE,
    FieldDescriptorProto.TYPE_INT32,
    FieldDescriptorProto.TYPE_UINT32,
    FieldDescriptorProto.TYPE_INT64,
    FieldDescriptorProto.TYPE_UINT64,
    FieldDescriptorProto.TYPE_BOOL,
}


class SchemaError(ValueError):
    """Raised when a parameter schema message is invalid."""


@dataclass
class Schema:
    """Parameter schema message.

    Attributes:
        name: Message full name, also the schema name.
        cpp_name: Message C++ type name.
        version: Schema version from the `parameter_schema` option.
        keys: Parameter keys (field names), in declaration order.
    """

    name: str
    cpp_name: str
    version: int
    keys: list[str]


class OptionReader:
    """Read the custom options of the request files.

    Note: protoc sends options with unknown extensions, so they are re-parsed with the classes
    of a pool holding every request file.
    """

    def __init__(self, request: plugin_pb2.CodeGeneratorRequest) -> None:
        """Initialize the option reader.

        Args:
            request: Request sent by protoc.
        """
        self._pool = descriptor_pool.DescriptorPool()
        for file_proto in request.proto_file:
            self._pool.Add(file_proto)
        self._schema_extension = None
        self._message_options_class = None
        try:
            self._schema_extension = self._pool.FindExtensionByName(SCHEMA_OPTION)
        except KeyError:
            return
        self._message_options_class = message_factory.GetMessageClass(
            self._pool.FindMessageTypeByName("google.protobuf.MessageOptions")
        )

    def schema_version(self, message_proto: DescriptorProto) -> int | None:
        """Read the `parameter_schema` option of a message.

        Args:
            message_proto: Message to inspect.

        Returns:
            Schema version, or None if the message has no `parameter_schema` option.
        """
        option = self._read_option(message_proto, self._message_options_class, self._schema_extension)
        return None if option is None else option.version

    @staticmethod
    def _read_option(proto, options_class, extension):
        """Read a custom option from the options of a descriptor.

        Args:
            proto: Descriptor whose options are read.
            options_class: Options message class of the pool.
            extension: Custom option extension.

        Returns:
            Option value, or None if the descriptor does not set it.
        """
        if extension is None or not proto.HasField("options"):
            return None
        options = options_class.FromString(proto.options.SerializeToString())
        if not options.HasExtension(extension):
            return None
        return options.Extensions[extension]


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


def check_field(schema_name: str, field_proto: FieldDescriptorProto) -> None:
    """Check a parameter field.

    Args:
        schema_name: Name of the schema message, for error messages.
        field_proto: Parameter field.

    Raises:
        SchemaError: If the field breaks a schema rule.
    """
    location = f"{schema_name}.{field_proto.name}"
    if field_proto.type not in VALUE_TYPES:
        raise SchemaError(f"{location}: unsupported type, use float, double, (u)int32, (u)int64 or bool")
    if field_proto.label == FieldDescriptorProto.LABEL_REPEATED:
        raise SchemaError(f"{location}: repeated fields are not supported")


def parse_schema(file_proto: FileDescriptorProto, message_proto: DescriptorProto, version: int) -> Schema:
    """Check a schema message and extract its parameter keys.

    Args:
        file_proto: File declaring the message.
        message_proto: Schema message.
        version: Schema version.

    Returns:
        Parsed schema.

    Raises:
        SchemaError: If the message breaks a schema rule.
    """
    name = f"{file_proto.package}.{message_proto.name}" if file_proto.package else message_proto.name
    if version == 0:
        raise SchemaError(f"{name}: parameter_schema version must be >= 1")
    if not message_proto.field:
        raise SchemaError(f"{name}: no parameter field")

    for field_proto in message_proto.field:
        check_field(name, field_proto)
    keys = [field_proto.name for field_proto in message_proto.field]
    return Schema(name=name, cpp_name=name.replace(".", "::"), version=version, keys=keys)


def check_hashes(schemas: list[Schema]) -> None:
    """Check schema ids and key hashes are unique across all schemas of the build.

    Note: schemas and parameters are addressed by hash, so two names with the same hash cannot be
    told apart.

    Args:
        schemas: Schemas of the build.

    Raises:
        SchemaError: If two schemas or two parameters share a hash.
    """
    ids: dict[int, str] = {}
    for schema in schemas:
        schema_id = fnv1a_hash(schema.name)
        if schema_id in ids:
            raise SchemaError(f"{schema.name}: schema id 0x{schema_id:08x} collides with {ids[schema_id]}")
        ids[schema_id] = schema.name

    hashes: dict[int, str] = {}
    for schema in schemas:
        for key in schema.keys:
            location = f"{schema.name}.{key}"
            key_hash = fnv1a_hash(key)
            if key_hash in hashes:
                raise SchemaError(f"{location}: key hash 0x{key_hash:08x} collides with {hashes[key_hash]}")
            hashes[key_hash] = location


def render_header(proto_name: str, schemas: list[Schema]) -> str:
    """Render the C++ header of the schemas declared in one .proto file.

    Args:
        proto_name: Name of the .proto file.
        schemas: Schemas declared in this file.

    Returns:
        Header content.
    """
    lines = [
        f"// Generated by protoc-gen-cogip-param from {proto_name}. Do not edit.",
        "",
        "#pragma once",
        "",
        "#include <cstdint>",
        "",
        '#include "KeyHash.hpp"',
        '#include "parameter/ParameterSchema.hpp"',
        "",
        f'#include "{PurePosixPath(proto_name).with_suffix(".hpp")}"',
        "",
        "namespace cogip {",
        "",
        "namespace parameter {",
    ]
    for schema in schemas:
        lines += ["", *_render_schema(schema)]
    lines += [
        "",
        "} // namespace parameter",
        "",
        "} // namespace cogip",
        "",
    ]
    return "\n".join(lines)


def generate(request: plugin_pb2.CodeGeneratorRequest, response: plugin_pb2.CodeGeneratorResponse) -> None:
    """Generate the headers of every schema message of the files to generate.

    Args:
        request: Request sent by protoc.
        response: Response filled with the generated files.

    Raises:
        SchemaError: If a schema message is invalid.
    """
    option_reader = OptionReader(request)
    files_to_generate = set(request.file_to_generate)
    schemas_by_file: dict[str, list[Schema]] = {}
    for file_proto in request.proto_file:
        if file_proto.name not in files_to_generate:
            continue
        for message_proto in file_proto.message_type:
            version = option_reader.schema_version(message_proto)
            if version is None:
                continue
            schemas_by_file.setdefault(file_proto.name, []).append(
                parse_schema(file_proto, message_proto, version)
            )

    check_hashes([schema for schemas in schemas_by_file.values() for schema in schemas])

    for file_name, schemas in schemas_by_file.items():
        generated = response.file.add()
        generated.name = f"{PurePosixPath(file_name).with_suffix('')}_parameters.hpp"
        generated.content = render_header(file_name, schemas)


def main() -> None:
    """Run the plugin: read the request on stdin, write the response on stdout."""
    request = plugin_pb2.CodeGeneratorRequest.FromString(sys.stdin.buffer.read())
    response = plugin_pb2.CodeGeneratorResponse()
    response.supported_features = plugin_pb2.CodeGeneratorResponse.FEATURE_PROTO3_OPTIONAL
    try:
        generate(request, response)
    except SchemaError as exc:
        response.error = str(exc)
    sys.stdout.buffer.write(response.SerializeToString())


def _render_schema(schema: Schema) -> list[str]:
    """Render the `ParameterSchema` specialization of one schema message.

    Args:
        schema: Schema to render.

    Returns:
        Header lines.
    """
    lines = [
        f"/// @brief Parameter schema of {schema.name} (version {schema.version}).",
        f"template <> struct ParameterSchema<{schema.cpp_name}>",
        "{",
        "    /// Schema id: FNV-1a hash of the schema message full name",
        f'    static constexpr uint32_t ID = cogip::utils::hash_key("{schema.name}");',
        f"    static constexpr uint32_t VERSION = {schema.version}; ///< Schema version",
        "",
        "    /// @brief Parameter keys: FNV-1a hash of the field names, as computed by host tools.",
        "    struct Keys",
        "    {",
    ]
    for key in schema.keys:
        lines.append(f'        static constexpr uint32_t {key.upper()} = cogip::utils::hash_key("{key}");')
    lines += [
        "    };",
        "};",
    ]
    return lines


if __name__ == "__main__":
    main()
