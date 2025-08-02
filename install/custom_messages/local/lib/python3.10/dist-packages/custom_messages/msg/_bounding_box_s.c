// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_messages:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "custom_messages/msg/detail/bounding_box__struct.h"
#include "custom_messages/msg/detail/bounding_box__functions.h"

bool custom_messages__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * custom_messages__msg__point__convert_to_py(void * raw_ros_message);
bool custom_messages__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * custom_messages__msg__point__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool custom_messages__msg__bounding_box__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("custom_messages.msg._bounding_box.BoundingBox", full_classname_dest, 45) == 0);
  }
  custom_messages__msg__BoundingBox * ros_message = _ros_message;
  {  // low_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "low_left");
    if (!field) {
      return false;
    }
    if (!custom_messages__msg__point__convert_from_py(field, &ros_message->low_left)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // top_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "top_right");
    if (!field) {
      return false;
    }
    if (!custom_messages__msg__point__convert_from_py(field, &ros_message->top_right)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_messages__msg__bounding_box__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BoundingBox */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_messages.msg._bounding_box");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BoundingBox");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_messages__msg__BoundingBox * ros_message = (custom_messages__msg__BoundingBox *)raw_ros_message;
  {  // low_left
    PyObject * field = NULL;
    field = custom_messages__msg__point__convert_to_py(&ros_message->low_left);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "low_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // top_right
    PyObject * field = NULL;
    field = custom_messages__msg__point__convert_to_py(&ros_message->top_right);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "top_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
