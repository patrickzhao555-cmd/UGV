# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ugv_sensor_sync:msg/UwbRange.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UwbRange(type):
    """Metaclass of message 'UwbRange'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ugv_sensor_sync')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ugv_sensor_sync.msg.UwbRange')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__uwb_range
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__uwb_range
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__uwb_range
            cls._TYPE_SUPPORT = module.type_support_msg__msg__uwb_range
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__uwb_range

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UwbRange(metaclass=Metaclass_UwbRange):
    """Message class 'UwbRange'."""

    __slots__ = [
        '_header',
        '_esp32_time_us',
        '_range_m',
        '_range_stddev_m',
        '_anchor_id',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'esp32_time_us': 'uint64',
        'range_m': 'double',
        'range_stddev_m': 'double',
        'anchor_id': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.esp32_time_us = kwargs.get('esp32_time_us', int())
        self.range_m = kwargs.get('range_m', float())
        self.range_stddev_m = kwargs.get('range_stddev_m', float())
        self.anchor_id = kwargs.get('anchor_id', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.esp32_time_us != other.esp32_time_us:
            return False
        if self.range_m != other.range_m:
            return False
        if self.range_stddev_m != other.range_stddev_m:
            return False
        if self.anchor_id != other.anchor_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def esp32_time_us(self):
        """Message field 'esp32_time_us'."""
        return self._esp32_time_us

    @esp32_time_us.setter
    def esp32_time_us(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'esp32_time_us' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'esp32_time_us' field must be an unsigned integer in [0, 18446744073709551615]"
        self._esp32_time_us = value

    @builtins.property
    def range_m(self):
        """Message field 'range_m'."""
        return self._range_m

    @range_m.setter
    def range_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'range_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._range_m = value

    @builtins.property
    def range_stddev_m(self):
        """Message field 'range_stddev_m'."""
        return self._range_stddev_m

    @range_stddev_m.setter
    def range_stddev_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range_stddev_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'range_stddev_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._range_stddev_m = value

    @builtins.property
    def anchor_id(self):
        """Message field 'anchor_id'."""
        return self._anchor_id

    @anchor_id.setter
    def anchor_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'anchor_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'anchor_id' field must be an unsigned integer in [0, 4294967295]"
        self._anchor_id = value
