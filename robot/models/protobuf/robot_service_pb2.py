# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: protobuf/robot_service.proto
# Protobuf Python Version: 5.29.0
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    29,
    0,
    '',
    'protobuf/robot_service.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from protobuf import common_pb2 as protobuf_dot_common__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x1cprotobuf/robot_service.proto\x12\x02pb\x1a\x15protobuf/common.proto\" \n\x0cRegistration\x12\x10\n\x08robot_id\x18\x01 \x01(\t\"\x8f\x01\n\x0cRobotMessage\x12(\n\x0cregistration\x18\x01 \x01(\x0b\x32\x10.pb.RegistrationH\x00\x12#\n\x07\x63ommand\x18\x02 \x01(\x0b\x32\x10.pb.RobotCommandH\x00\x12%\n\x08response\x18\x03 \x01(\x0b\x32\x11.pb.RobotResponseH\x00\x42\t\n\x07payload2A\n\x0cRobotService\x12\x31\n\x07\x43onnect\x12\x10.pb.RobotMessage\x1a\x10.pb.RobotMessage(\x01\x30\x01\x42\x1eZ\x1cturtlesim-grpc/server/modelsb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'protobuf.robot_service_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  _globals['DESCRIPTOR']._loaded_options = None
  _globals['DESCRIPTOR']._serialized_options = b'Z\034turtlesim-grpc/server/models'
  _globals['_REGISTRATION']._serialized_start=59
  _globals['_REGISTRATION']._serialized_end=91
  _globals['_ROBOTMESSAGE']._serialized_start=94
  _globals['_ROBOTMESSAGE']._serialized_end=237
  _globals['_ROBOTSERVICE']._serialized_start=239
  _globals['_ROBOTSERVICE']._serialized_end=304
# @@protoc_insertion_point(module_scope)
