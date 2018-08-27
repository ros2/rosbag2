// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Python.h>
#include <string>
#include <vector>

#include "rosbag2_transport/rosbag2_transport.hpp"

static PyObject *
rosbag2_transport_record_topics(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * listObj; /* the list of strings */
  if (!PyArg_ParseTuple(args, "O", &listObj)) {
    return NULL;
  }

  PyObject *iterator = PyObject_GetIter(listObj);
  if (iterator == NULL) {
    return NULL;
  }

  std::vector<std::string> topic_vector;
  PyObject *item;
  while ((item = PyIter_Next(iterator))) {
    topic_vector.push_back(std::string(PyUnicode_AsUTF8(item)));

    Py_DECREF(item);
  }
  Py_DECREF(iterator);

  if (topic_vector.size() == 0) {
    return NULL;
  }
  rosbag2_transport::Rosbag2Transport transport;
  transport.record(topic_vector);

  Py_RETURN_NONE;
}

/// Define the public methods of this module
static PyMethodDef rosbag2_transport_methods[] = {
  {
    "record_topics", rosbag2_transport_record_topics, METH_VARARGS,
    "Record topics"
  },
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rosbag2_transport__doc__,
  "Python module for rosbag2 transport");

/// Define the Python module
static struct PyModuleDef _rosbag2_transport_module = {
  PyModuleDef_HEAD_INIT,
  "_rosbag2_transport",
  rosbag2_transport__doc__,
  -1,   /* -1 means that the module keeps state in global variables */
  rosbag2_transport_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rosbag2_transport_py(void)
{
  return PyModule_Create(&_rosbag2_transport_module);
}
