#
# Autogenerated by Thrift Compiler (0.9.1)
#
# DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
#
#  options string: py
#

from thrift.Thrift import TType, TMessageType, TException, TApplicationException

from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol, TProtocol
try:
  from thrift.protocol import fastbinary
except:
  fastbinary = None



class TAppDataRaw:
  """
  ****************************************************************************
   interface objects for the frAIburg webapp
  ****************************************************************************

  Attributes:
   - raw_data
  """

  thrift_spec = (
    None, # 0
    (1, TType.STRING, 'raw_data', None, None, ), # 1
  )

  def __init__(self, raw_data=None,):
    self.raw_data = raw_data

  def read(self, iprot):
    if iprot.__class__ == TBinaryProtocol.TBinaryProtocolAccelerated and isinstance(iprot.trans, TTransport.CReadableTransport) and self.thrift_spec is not None and fastbinary is not None:
      fastbinary.decode_binary(self, iprot.trans, (self.__class__, self.thrift_spec))
      return
    iprot.readStructBegin()
    while True:
      (fname, ftype, fid) = iprot.readFieldBegin()
      if ftype == TType.STOP:
        break
      if fid == 1:
        if ftype == TType.STRING:
          self.raw_data = iprot.readString();
        else:
          iprot.skip(ftype)
      else:
        iprot.skip(ftype)
      iprot.readFieldEnd()
    iprot.readStructEnd()

  def write(self, oprot):
    if oprot.__class__ == TBinaryProtocol.TBinaryProtocolAccelerated and self.thrift_spec is not None and fastbinary is not None:
      oprot.trans.write(fastbinary.encode_binary(self, (self.__class__, self.thrift_spec)))
      return
    oprot.writeStructBegin('TAppDataRaw')
    if self.raw_data is not None:
      oprot.writeFieldBegin('raw_data', TType.STRING, 1)
      oprot.writeString(self.raw_data)
      oprot.writeFieldEnd()
    oprot.writeFieldStop()
    oprot.writeStructEnd()

  def validate(self):
    if self.raw_data is None:
      raise TProtocol.TProtocolException(message='Required field raw_data is unset!')
    return


  def __repr__(self):
    L = ['%s=%r' % (key, value)
      for key, value in self.__dict__.iteritems()]
    return '%s(%s)' % (self.__class__.__name__, ', '.join(L))

  def __eq__(self, other):
    return isinstance(other, self.__class__) and self.__dict__ == other.__dict__

  def __ne__(self, other):
    return not (self == other)

class TImageParams:
  """
  Attributes:
   - height
   - width
   - name
  """

  thrift_spec = (
    None, # 0
    (1, TType.I16, 'height', None, None, ), # 1
    (2, TType.I16, 'width', None, None, ), # 2
    (3, TType.STRING, 'name', None, None, ), # 3
  )

  def __init__(self, height=None, width=None, name=None,):
    self.height = height
    self.width = width
    self.name = name

  def read(self, iprot):
    if iprot.__class__ == TBinaryProtocol.TBinaryProtocolAccelerated and isinstance(iprot.trans, TTransport.CReadableTransport) and self.thrift_spec is not None and fastbinary is not None:
      fastbinary.decode_binary(self, iprot.trans, (self.__class__, self.thrift_spec))
      return
    iprot.readStructBegin()
    while True:
      (fname, ftype, fid) = iprot.readFieldBegin()
      if ftype == TType.STOP:
        break
      if fid == 1:
        if ftype == TType.I16:
          self.height = iprot.readI16();
        else:
          iprot.skip(ftype)
      elif fid == 2:
        if ftype == TType.I16:
          self.width = iprot.readI16();
        else:
          iprot.skip(ftype)
      elif fid == 3:
        if ftype == TType.STRING:
          self.name = iprot.readString();
        else:
          iprot.skip(ftype)
      else:
        iprot.skip(ftype)
      iprot.readFieldEnd()
    iprot.readStructEnd()

  def write(self, oprot):
    if oprot.__class__ == TBinaryProtocol.TBinaryProtocolAccelerated and self.thrift_spec is not None and fastbinary is not None:
      oprot.trans.write(fastbinary.encode_binary(self, (self.__class__, self.thrift_spec)))
      return
    oprot.writeStructBegin('TImageParams')
    if self.height is not None:
      oprot.writeFieldBegin('height', TType.I16, 1)
      oprot.writeI16(self.height)
      oprot.writeFieldEnd()
    if self.width is not None:
      oprot.writeFieldBegin('width', TType.I16, 2)
      oprot.writeI16(self.width)
      oprot.writeFieldEnd()
    if self.name is not None:
      oprot.writeFieldBegin('name', TType.STRING, 3)
      oprot.writeString(self.name)
      oprot.writeFieldEnd()
    oprot.writeFieldStop()
    oprot.writeStructEnd()

  def validate(self):
    if self.height is None:
      raise TProtocol.TProtocolException(message='Required field height is unset!')
    if self.width is None:
      raise TProtocol.TProtocolException(message='Required field width is unset!')
    return


  def __repr__(self):
    L = ['%s=%r' % (key, value)
      for key, value in self.__dict__.iteritems()]
    return '%s(%s)' % (self.__class__.__name__, ', '.join(L))

  def __eq__(self, other):
    return isinstance(other, self.__class__) and self.__dict__ == other.__dict__

  def __ne__(self, other):
    return not (self == other)
