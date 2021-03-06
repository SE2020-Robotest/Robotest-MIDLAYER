# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: msg.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='msg.proto',
  package='msg',
  syntax='proto3',
  serialized_options=b'\n\010msg.grpcP\001',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\tmsg.proto\x12\x03msg\"#\n\x05Point\x12\x0c\n\x04posx\x18\x01 \x01(\x01\x12\x0c\n\x04posy\x18\x02 \x01(\x01\"_\n\nRBPosition\x12\x17\n\x03pos\x18\x01 \x01(\x0b\x32\n.msg.Point\x12\r\n\x05\x61ngle\x18\x02 \x01(\x01\x12\n\n\x02vx\x18\x03 \x01(\x01\x12\n\n\x02vy\x18\x04 \x01(\x01\x12\x11\n\ttimestamp\x18\x05 \x01(\x01\"E\n\x06RBPath\x12\x17\n\x03pos\x18\x01 \x03(\x0b\x32\n.msg.Point\x12\x11\n\tstarttime\x18\x02 \x01(\x01\x12\x0f\n\x07\x65ndtime\x18\x03 \x01(\x01\"\x19\n\tVoiceData\x12\x0c\n\x04\x66ile\x18\x01 \x01(\x0c\",\n\x08VoiceStr\x12\r\n\x05voice\x18\x01 \x01(\t\x12\x11\n\ttimestamp\x18\x02 \x01(\x01\"\x15\n\x06LogStr\x12\x0b\n\x03log\x18\x01 \x01(\t\"u\n\x05\x42lock\x12\x1d\n\x04type\x18\x01 \x01(\x0e\x32\x0f.msg.Block.Type\x12\t\n\x01w\x18\x02 \x01(\x01\x12\t\n\x01h\x18\x03 \x01(\x01\x12\x17\n\x03pos\x18\x04 \x01(\x0b\x32\n.msg.Point\"\x1e\n\x04Type\x12\x08\n\x04\x43UBE\x10\x00\x12\x0c\n\x08\x43YLINDER\x10\x01\"H\n\x03Map\x12\x11\n\troomwidth\x18\x01 \x01(\x01\x12\x12\n\nroomheight\x18\x02 \x01(\x01\x12\x1a\n\x06\x62locks\x18\x03 \x03(\x0b\x32\n.msg.Block\"_\n\nControlCmd\x12$\n\x03\x63md\x18\x01 \x01(\x0e\x32\x17.msg.ControlCmd.CtrlCmd\"+\n\x07\x43trlCmd\x12\t\n\x05START\x10\x00\x12\x08\n\x04STOP\x10\x01\x12\x0b\n\x07\x43ONNECT\x10\x02\"\x86\x01\n\x05\x44rive\x12%\n\x08\x64rivecmd\x18\x01 \x01(\x0e\x32\x13.msg.Drive.DriveCmd\"V\n\x08\x44riveCmd\x12\t\n\x05\x46RONT\x10\x00\x12\x08\n\x04\x42\x41\x43K\x10\x01\x12\x08\n\x04LEFT\x10\x02\x12\t\n\x05RIGHT\x10\x03\x12\r\n\tCLOCKWISE\x10\x04\x12\x11\n\rANTICLOCKWISE\x10\x05\"[\n\x08Response\x12$\n\x06status\x18\x01 \x01(\x0e\x32\x14.msg.Response.Status\")\n\x06Status\x12\x06\n\x02OK\x10\x00\x12\t\n\x05\x45RROR\x10\x01\x12\x0c\n\x08\x46INISHED\x10\x02\x32\x99\x03\n\x0bMsgServices\x12$\n\tConfigMap\x12\x08.msg.Map\x1a\r.msg.Response\x12/\n\rRobotPosition\x12\x0f.msg.RBPosition\x1a\r.msg.Response\x12\'\n\tRobotPath\x12\x0b.msg.RBPath\x1a\r.msg.Response\x12\x30\n\rSendVoiceFile\x12\x0e.msg.VoiceData\x1a\r.msg.Response(\x01\x12+\n\x0bVoiceResult\x12\r.msg.VoiceStr\x1a\r.msg.Response\x12!\n\x03Log\x12\x0b.msg.LogStr\x1a\r.msg.Response\x12\x30\n\x0e\x43ontrolCommand\x12\x0f.msg.ControlCmd\x1a\r.msg.Response\x12-\n\rRobotFinished\x12\r.msg.Response\x1a\r.msg.Response\x12\'\n\nDriveRobot\x12\n.msg.Drive\x1a\r.msg.ResponseB\x0c\n\x08msg.grpcP\x01\x62\x06proto3'
)



_BLOCK_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='msg.Block.Type',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='CUBE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CYLINDER', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=406,
  serialized_end=436,
)
_sym_db.RegisterEnumDescriptor(_BLOCK_TYPE)

_CONTROLCMD_CTRLCMD = _descriptor.EnumDescriptor(
  name='CtrlCmd',
  full_name='msg.ControlCmd.CtrlCmd',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='START', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STOP', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CONNECT', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=564,
  serialized_end=607,
)
_sym_db.RegisterEnumDescriptor(_CONTROLCMD_CTRLCMD)

_DRIVE_DRIVECMD = _descriptor.EnumDescriptor(
  name='DriveCmd',
  full_name='msg.Drive.DriveCmd',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FRONT', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BACK', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LEFT', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RIGHT', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CLOCKWISE', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ANTICLOCKWISE', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=658,
  serialized_end=744,
)
_sym_db.RegisterEnumDescriptor(_DRIVE_DRIVECMD)

_RESPONSE_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='msg.Response.Status',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='FINISHED', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=796,
  serialized_end=837,
)
_sym_db.RegisterEnumDescriptor(_RESPONSE_STATUS)


_POINT = _descriptor.Descriptor(
  name='Point',
  full_name='msg.Point',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='posx', full_name='msg.Point.posx', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='posy', full_name='msg.Point.posy', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=18,
  serialized_end=53,
)


_RBPOSITION = _descriptor.Descriptor(
  name='RBPosition',
  full_name='msg.RBPosition',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='pos', full_name='msg.RBPosition.pos', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='angle', full_name='msg.RBPosition.angle', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vx', full_name='msg.RBPosition.vx', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vy', full_name='msg.RBPosition.vy', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='msg.RBPosition.timestamp', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=55,
  serialized_end=150,
)


_RBPATH = _descriptor.Descriptor(
  name='RBPath',
  full_name='msg.RBPath',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='pos', full_name='msg.RBPath.pos', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='starttime', full_name='msg.RBPath.starttime', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='endtime', full_name='msg.RBPath.endtime', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=152,
  serialized_end=221,
)


_VOICEDATA = _descriptor.Descriptor(
  name='VoiceData',
  full_name='msg.VoiceData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='file', full_name='msg.VoiceData.file', index=0,
      number=1, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=223,
  serialized_end=248,
)


_VOICESTR = _descriptor.Descriptor(
  name='VoiceStr',
  full_name='msg.VoiceStr',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='voice', full_name='msg.VoiceStr.voice', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='msg.VoiceStr.timestamp', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=250,
  serialized_end=294,
)


_LOGSTR = _descriptor.Descriptor(
  name='LogStr',
  full_name='msg.LogStr',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='log', full_name='msg.LogStr.log', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=296,
  serialized_end=317,
)


_BLOCK = _descriptor.Descriptor(
  name='Block',
  full_name='msg.Block',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='msg.Block.type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='w', full_name='msg.Block.w', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='h', full_name='msg.Block.h', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pos', full_name='msg.Block.pos', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _BLOCK_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=319,
  serialized_end=436,
)


_MAP = _descriptor.Descriptor(
  name='Map',
  full_name='msg.Map',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='roomwidth', full_name='msg.Map.roomwidth', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='roomheight', full_name='msg.Map.roomheight', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='blocks', full_name='msg.Map.blocks', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=438,
  serialized_end=510,
)


_CONTROLCMD = _descriptor.Descriptor(
  name='ControlCmd',
  full_name='msg.ControlCmd',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='cmd', full_name='msg.ControlCmd.cmd', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _CONTROLCMD_CTRLCMD,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=512,
  serialized_end=607,
)


_DRIVE = _descriptor.Descriptor(
  name='Drive',
  full_name='msg.Drive',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='drivecmd', full_name='msg.Drive.drivecmd', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _DRIVE_DRIVECMD,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=610,
  serialized_end=744,
)


_RESPONSE = _descriptor.Descriptor(
  name='Response',
  full_name='msg.Response',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='msg.Response.status', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _RESPONSE_STATUS,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=746,
  serialized_end=837,
)

_RBPOSITION.fields_by_name['pos'].message_type = _POINT
_RBPATH.fields_by_name['pos'].message_type = _POINT
_BLOCK.fields_by_name['type'].enum_type = _BLOCK_TYPE
_BLOCK.fields_by_name['pos'].message_type = _POINT
_BLOCK_TYPE.containing_type = _BLOCK
_MAP.fields_by_name['blocks'].message_type = _BLOCK
_CONTROLCMD.fields_by_name['cmd'].enum_type = _CONTROLCMD_CTRLCMD
_CONTROLCMD_CTRLCMD.containing_type = _CONTROLCMD
_DRIVE.fields_by_name['drivecmd'].enum_type = _DRIVE_DRIVECMD
_DRIVE_DRIVECMD.containing_type = _DRIVE
_RESPONSE.fields_by_name['status'].enum_type = _RESPONSE_STATUS
_RESPONSE_STATUS.containing_type = _RESPONSE
DESCRIPTOR.message_types_by_name['Point'] = _POINT
DESCRIPTOR.message_types_by_name['RBPosition'] = _RBPOSITION
DESCRIPTOR.message_types_by_name['RBPath'] = _RBPATH
DESCRIPTOR.message_types_by_name['VoiceData'] = _VOICEDATA
DESCRIPTOR.message_types_by_name['VoiceStr'] = _VOICESTR
DESCRIPTOR.message_types_by_name['LogStr'] = _LOGSTR
DESCRIPTOR.message_types_by_name['Block'] = _BLOCK
DESCRIPTOR.message_types_by_name['Map'] = _MAP
DESCRIPTOR.message_types_by_name['ControlCmd'] = _CONTROLCMD
DESCRIPTOR.message_types_by_name['Drive'] = _DRIVE
DESCRIPTOR.message_types_by_name['Response'] = _RESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Point = _reflection.GeneratedProtocolMessageType('Point', (_message.Message,), {
  'DESCRIPTOR' : _POINT,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.Point)
  })
_sym_db.RegisterMessage(Point)

RBPosition = _reflection.GeneratedProtocolMessageType('RBPosition', (_message.Message,), {
  'DESCRIPTOR' : _RBPOSITION,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.RBPosition)
  })
_sym_db.RegisterMessage(RBPosition)

RBPath = _reflection.GeneratedProtocolMessageType('RBPath', (_message.Message,), {
  'DESCRIPTOR' : _RBPATH,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.RBPath)
  })
_sym_db.RegisterMessage(RBPath)

VoiceData = _reflection.GeneratedProtocolMessageType('VoiceData', (_message.Message,), {
  'DESCRIPTOR' : _VOICEDATA,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.VoiceData)
  })
_sym_db.RegisterMessage(VoiceData)

VoiceStr = _reflection.GeneratedProtocolMessageType('VoiceStr', (_message.Message,), {
  'DESCRIPTOR' : _VOICESTR,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.VoiceStr)
  })
_sym_db.RegisterMessage(VoiceStr)

LogStr = _reflection.GeneratedProtocolMessageType('LogStr', (_message.Message,), {
  'DESCRIPTOR' : _LOGSTR,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.LogStr)
  })
_sym_db.RegisterMessage(LogStr)

Block = _reflection.GeneratedProtocolMessageType('Block', (_message.Message,), {
  'DESCRIPTOR' : _BLOCK,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.Block)
  })
_sym_db.RegisterMessage(Block)

Map = _reflection.GeneratedProtocolMessageType('Map', (_message.Message,), {
  'DESCRIPTOR' : _MAP,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.Map)
  })
_sym_db.RegisterMessage(Map)

ControlCmd = _reflection.GeneratedProtocolMessageType('ControlCmd', (_message.Message,), {
  'DESCRIPTOR' : _CONTROLCMD,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.ControlCmd)
  })
_sym_db.RegisterMessage(ControlCmd)

Drive = _reflection.GeneratedProtocolMessageType('Drive', (_message.Message,), {
  'DESCRIPTOR' : _DRIVE,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.Drive)
  })
_sym_db.RegisterMessage(Drive)

Response = _reflection.GeneratedProtocolMessageType('Response', (_message.Message,), {
  'DESCRIPTOR' : _RESPONSE,
  '__module__' : 'msg_pb2'
  # @@protoc_insertion_point(class_scope:msg.Response)
  })
_sym_db.RegisterMessage(Response)


DESCRIPTOR._options = None

_MSGSERVICES = _descriptor.ServiceDescriptor(
  name='MsgServices',
  full_name='msg.MsgServices',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=840,
  serialized_end=1249,
  methods=[
  _descriptor.MethodDescriptor(
    name='ConfigMap',
    full_name='msg.MsgServices.ConfigMap',
    index=0,
    containing_service=None,
    input_type=_MAP,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='RobotPosition',
    full_name='msg.MsgServices.RobotPosition',
    index=1,
    containing_service=None,
    input_type=_RBPOSITION,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='RobotPath',
    full_name='msg.MsgServices.RobotPath',
    index=2,
    containing_service=None,
    input_type=_RBPATH,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SendVoiceFile',
    full_name='msg.MsgServices.SendVoiceFile',
    index=3,
    containing_service=None,
    input_type=_VOICEDATA,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='VoiceResult',
    full_name='msg.MsgServices.VoiceResult',
    index=4,
    containing_service=None,
    input_type=_VOICESTR,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='Log',
    full_name='msg.MsgServices.Log',
    index=5,
    containing_service=None,
    input_type=_LOGSTR,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='ControlCommand',
    full_name='msg.MsgServices.ControlCommand',
    index=6,
    containing_service=None,
    input_type=_CONTROLCMD,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='RobotFinished',
    full_name='msg.MsgServices.RobotFinished',
    index=7,
    containing_service=None,
    input_type=_RESPONSE,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='DriveRobot',
    full_name='msg.MsgServices.DriveRobot',
    index=8,
    containing_service=None,
    input_type=_DRIVE,
    output_type=_RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_MSGSERVICES)

DESCRIPTOR.services_by_name['MsgServices'] = _MSGSERVICES

# @@protoc_insertion_point(module_scope)
