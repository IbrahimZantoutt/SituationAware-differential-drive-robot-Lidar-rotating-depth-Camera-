// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_interfaces:action/Nav.idl
// generated code does not contain a copyright notice

#ifndef ACTION_INTERFACES__ACTION__DETAIL__NAV__STRUCT_H_
#define ACTION_INTERFACES__ACTION__DETAIL__NAV__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_Goal
{
  float target_x;
  float target_y;
} action_interfaces__action__Nav_Goal;

// Struct for a sequence of action_interfaces__action__Nav_Goal.
typedef struct action_interfaces__action__Nav_Goal__Sequence
{
  action_interfaces__action__Nav_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_Result
{
  rosidl_runtime_c__String status;
  float final_x;
  float final_y;
} action_interfaces__action__Nav_Result;

// Struct for a sequence of action_interfaces__action__Nav_Result.
typedef struct action_interfaces__action__Nav_Result__Sequence
{
  action_interfaces__action__Nav_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_Feedback
{
  float current_x;
  float current_y;
  float distance_remaining;
} action_interfaces__action__Nav_Feedback;

// Struct for a sequence of action_interfaces__action__Nav_Feedback.
typedef struct action_interfaces__action__Nav_Feedback__Sequence
{
  action_interfaces__action__Nav_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_interfaces/action/detail/nav__struct.h"

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_interfaces__action__Nav_Goal goal;
} action_interfaces__action__Nav_SendGoal_Request;

// Struct for a sequence of action_interfaces__action__Nav_SendGoal_Request.
typedef struct action_interfaces__action__Nav_SendGoal_Request__Sequence
{
  action_interfaces__action__Nav_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_interfaces__action__Nav_SendGoal_Response;

// Struct for a sequence of action_interfaces__action__Nav_SendGoal_Response.
typedef struct action_interfaces__action__Nav_SendGoal_Response__Sequence
{
  action_interfaces__action__Nav_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_interfaces__action__Nav_GetResult_Request;

// Struct for a sequence of action_interfaces__action__Nav_GetResult_Request.
typedef struct action_interfaces__action__Nav_GetResult_Request__Sequence
{
  action_interfaces__action__Nav_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_interfaces/action/detail/nav__struct.h"

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_GetResult_Response
{
  int8_t status;
  action_interfaces__action__Nav_Result result;
} action_interfaces__action__Nav_GetResult_Response;

// Struct for a sequence of action_interfaces__action__Nav_GetResult_Response.
typedef struct action_interfaces__action__Nav_GetResult_Response__Sequence
{
  action_interfaces__action__Nav_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_interfaces/action/detail/nav__struct.h"

/// Struct defined in action/Nav in the package action_interfaces.
typedef struct action_interfaces__action__Nav_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_interfaces__action__Nav_Feedback feedback;
} action_interfaces__action__Nav_FeedbackMessage;

// Struct for a sequence of action_interfaces__action__Nav_FeedbackMessage.
typedef struct action_interfaces__action__Nav_FeedbackMessage__Sequence
{
  action_interfaces__action__Nav_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__Nav_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_INTERFACES__ACTION__DETAIL__NAV__STRUCT_H_
