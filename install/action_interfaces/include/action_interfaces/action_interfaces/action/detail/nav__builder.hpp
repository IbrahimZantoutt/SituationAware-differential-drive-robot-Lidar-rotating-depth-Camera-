// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_interfaces:action/Nav.idl
// generated code does not contain a copyright notice

#ifndef ACTION_INTERFACES__ACTION__DETAIL__NAV__BUILDER_HPP_
#define ACTION_INTERFACES__ACTION__DETAIL__NAV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_interfaces/action/detail/nav__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_Goal_target_y
{
public:
  explicit Init_Nav_Goal_target_y(::action_interfaces::action::Nav_Goal & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::Nav_Goal target_y(::action_interfaces::action::Nav_Goal::_target_y_type arg)
  {
    msg_.target_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_Goal msg_;
};

class Init_Nav_Goal_target_x
{
public:
  Init_Nav_Goal_target_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_Goal_target_y target_x(::action_interfaces::action::Nav_Goal::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_Nav_Goal_target_y(msg_);
  }

private:
  ::action_interfaces::action::Nav_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_Goal>()
{
  return action_interfaces::action::builder::Init_Nav_Goal_target_x();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_Result_final_y
{
public:
  explicit Init_Nav_Result_final_y(::action_interfaces::action::Nav_Result & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::Nav_Result final_y(::action_interfaces::action::Nav_Result::_final_y_type arg)
  {
    msg_.final_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_Result msg_;
};

class Init_Nav_Result_final_x
{
public:
  explicit Init_Nav_Result_final_x(::action_interfaces::action::Nav_Result & msg)
  : msg_(msg)
  {}
  Init_Nav_Result_final_y final_x(::action_interfaces::action::Nav_Result::_final_x_type arg)
  {
    msg_.final_x = std::move(arg);
    return Init_Nav_Result_final_y(msg_);
  }

private:
  ::action_interfaces::action::Nav_Result msg_;
};

class Init_Nav_Result_status
{
public:
  Init_Nav_Result_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_Result_final_x status(::action_interfaces::action::Nav_Result::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Nav_Result_final_x(msg_);
  }

private:
  ::action_interfaces::action::Nav_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_Result>()
{
  return action_interfaces::action::builder::Init_Nav_Result_status();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_Feedback_distance_remaining
{
public:
  explicit Init_Nav_Feedback_distance_remaining(::action_interfaces::action::Nav_Feedback & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::Nav_Feedback distance_remaining(::action_interfaces::action::Nav_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_Feedback msg_;
};

class Init_Nav_Feedback_current_y
{
public:
  explicit Init_Nav_Feedback_current_y(::action_interfaces::action::Nav_Feedback & msg)
  : msg_(msg)
  {}
  Init_Nav_Feedback_distance_remaining current_y(::action_interfaces::action::Nav_Feedback::_current_y_type arg)
  {
    msg_.current_y = std::move(arg);
    return Init_Nav_Feedback_distance_remaining(msg_);
  }

private:
  ::action_interfaces::action::Nav_Feedback msg_;
};

class Init_Nav_Feedback_current_x
{
public:
  Init_Nav_Feedback_current_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_Feedback_current_y current_x(::action_interfaces::action::Nav_Feedback::_current_x_type arg)
  {
    msg_.current_x = std::move(arg);
    return Init_Nav_Feedback_current_y(msg_);
  }

private:
  ::action_interfaces::action::Nav_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_Feedback>()
{
  return action_interfaces::action::builder::Init_Nav_Feedback_current_x();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_SendGoal_Request_goal
{
public:
  explicit Init_Nav_SendGoal_Request_goal(::action_interfaces::action::Nav_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::Nav_SendGoal_Request goal(::action_interfaces::action::Nav_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_SendGoal_Request msg_;
};

class Init_Nav_SendGoal_Request_goal_id
{
public:
  Init_Nav_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_SendGoal_Request_goal goal_id(::action_interfaces::action::Nav_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Nav_SendGoal_Request_goal(msg_);
  }

private:
  ::action_interfaces::action::Nav_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_SendGoal_Request>()
{
  return action_interfaces::action::builder::Init_Nav_SendGoal_Request_goal_id();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_SendGoal_Response_stamp
{
public:
  explicit Init_Nav_SendGoal_Response_stamp(::action_interfaces::action::Nav_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::Nav_SendGoal_Response stamp(::action_interfaces::action::Nav_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_SendGoal_Response msg_;
};

class Init_Nav_SendGoal_Response_accepted
{
public:
  Init_Nav_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_SendGoal_Response_stamp accepted(::action_interfaces::action::Nav_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Nav_SendGoal_Response_stamp(msg_);
  }

private:
  ::action_interfaces::action::Nav_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_SendGoal_Response>()
{
  return action_interfaces::action::builder::Init_Nav_SendGoal_Response_accepted();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_GetResult_Request_goal_id
{
public:
  Init_Nav_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_interfaces::action::Nav_GetResult_Request goal_id(::action_interfaces::action::Nav_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_GetResult_Request>()
{
  return action_interfaces::action::builder::Init_Nav_GetResult_Request_goal_id();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_GetResult_Response_result
{
public:
  explicit Init_Nav_GetResult_Response_result(::action_interfaces::action::Nav_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::Nav_GetResult_Response result(::action_interfaces::action::Nav_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_GetResult_Response msg_;
};

class Init_Nav_GetResult_Response_status
{
public:
  Init_Nav_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_GetResult_Response_result status(::action_interfaces::action::Nav_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Nav_GetResult_Response_result(msg_);
  }

private:
  ::action_interfaces::action::Nav_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_GetResult_Response>()
{
  return action_interfaces::action::builder::Init_Nav_GetResult_Response_status();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_Nav_FeedbackMessage_feedback
{
public:
  explicit Init_Nav_FeedbackMessage_feedback(::action_interfaces::action::Nav_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::Nav_FeedbackMessage feedback(::action_interfaces::action::Nav_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::Nav_FeedbackMessage msg_;
};

class Init_Nav_FeedbackMessage_goal_id
{
public:
  Init_Nav_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_FeedbackMessage_feedback goal_id(::action_interfaces::action::Nav_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Nav_FeedbackMessage_feedback(msg_);
  }

private:
  ::action_interfaces::action::Nav_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::Nav_FeedbackMessage>()
{
  return action_interfaces::action::builder::Init_Nav_FeedbackMessage_goal_id();
}

}  // namespace action_interfaces

#endif  // ACTION_INTERFACES__ACTION__DETAIL__NAV__BUILDER_HPP_
