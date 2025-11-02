// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/Add.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__ADD__STRUCT_H_
#define INTERFACES__SRV__DETAIL__ADD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Add in the package interfaces.
typedef struct interfaces__srv__Add_Request
{
  int16_t a;
  int16_t b;
} interfaces__srv__Add_Request;

// Struct for a sequence of interfaces__srv__Add_Request.
typedef struct interfaces__srv__Add_Request__Sequence
{
  interfaces__srv__Add_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__Add_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Add in the package interfaces.
typedef struct interfaces__srv__Add_Response
{
  int16_t sum;
} interfaces__srv__Add_Response;

// Struct for a sequence of interfaces__srv__Add_Response.
typedef struct interfaces__srv__Add_Response__Sequence
{
  interfaces__srv__Add_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__Add_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__ADD__STRUCT_H_
