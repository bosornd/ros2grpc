# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: calc.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\ncalc.proto\x12\x04\x63\x61lc\"!\n\x07Request\x12\n\n\x02n1\x18\x01 \x01(\x05\x12\n\n\x02n2\x18\x02 \x01(\x05\"\x15\n\x08Response\x12\t\n\x01r\x18\x01 \x01(\x05\x32/\n\x05\x41\x64\x64\x65r\x12&\n\x03\x61\x64\x64\x12\r.calc.Request\x1a\x0e.calc.Response\"\x00\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'calc_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _globals['_REQUEST']._serialized_start=20
  _globals['_REQUEST']._serialized_end=53
  _globals['_RESPONSE']._serialized_start=55
  _globals['_RESPONSE']._serialized_end=76
  _globals['_ADDER']._serialized_start=78
  _globals['_ADDER']._serialized_end=125
# @@protoc_insertion_point(module_scope)
