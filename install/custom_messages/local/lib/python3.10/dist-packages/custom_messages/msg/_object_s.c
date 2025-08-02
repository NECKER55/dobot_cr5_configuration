// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_messages:msg/Object.idl
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
#include "custom_messages/msg/detail/object__struct.h"
#include "custom_messages/msg/detail/object__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "custom_messages/msg/detail/circumference__functions.h"
// end nested array functions include
bool custom_messages__msg__bounding_box__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * custom_messages__msg__bounding_box__convert_to_py(void * raw_ros_message);
bool custom_messages__msg__circumference__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * custom_messages__msg__circumference__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool custom_messages__msg__object__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
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
    assert(strncmp("custom_messages.msg._object.Object", full_classname_dest, 34) == 0);
  }
  custom_messages__msg__Object * ros_message = _ros_message;
  {  // target
    PyObject * field = PyObject_GetAttrString(_pymsg, "target");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->target = (Py_True == field);
    Py_DECREF(field);
  }
  {  // shape
    PyObject * field = PyObject_GetAttrString(_pymsg, "shape");
    if (!field) {
      return false;
    }
    if (!custom_messages__msg__bounding_box__convert_from_py(field, &ros_message->shape)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // possible_trajectories
    PyObject * field = PyObject_GetAttrString(_pymsg, "possible_trajectories");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'possible_trajectories'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!custom_messages__msg__Circumference__Sequence__init(&(ros_message->possible_trajectories), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create custom_messages__msg__Circumference__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    custom_messages__msg__Circumference * dest = ros_message->possible_trajectories.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!custom_messages__msg__circumference__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_messages__msg__object__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Object */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_messages.msg._object");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Object");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_messages__msg__Object * ros_message = (custom_messages__msg__Object *)raw_ros_message;
  {  // target
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->target ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shape
    PyObject * field = NULL;
    field = custom_messages__msg__bounding_box__convert_to_py(&ros_message->shape);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "shape", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // possible_trajectories
    PyObject * field = NULL;
    size_t size = ros_message->possible_trajectories.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    custom_messages__msg__Circumference * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->possible_trajectories.data[i]);
      PyObject * pyitem = custom_messages__msg__circumference__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "possible_trajectories", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
