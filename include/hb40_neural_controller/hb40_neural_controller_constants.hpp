#ifndef HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_CONSTS_HPP_
#define HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_CONSTS_HPP_

#include <cstddef>
#include <array>
#include <utility>

namespace hb40_neural_controller
{
constexpr size_t TENSOR_SIZE = 48;
constexpr float SCALED_FACTOR = 0.25;

enum class Joints
{
  FR_j0, FR_j1, FR_j2,
  FL_j0, FL_j1, FL_j2,
  RL_j0, RL_j1, RL_j2,
  RR_j0, RR_j1, RR_j2,
  SPINE_0,
  SIZE
};

enum class Legs
{
  FR, FL, RL, RR,
  SIZE
};

using JointsOrder = std::make_index_sequence<static_cast<size_t>(Joints::SIZE)>;
using LegsOrder = std::make_index_sequence<static_cast<size_t>(Legs::SIZE)>;

template<typename Seq>
struct ReorderToTensor;

template<size_t... Is>
struct ReorderToTensor<std::index_sequence<Is...>>
{
  using type = std::index_sequence<static_cast<size_t>(Joints::SPINE_0),
      static_cast<size_t>(Joints::RL_j0),
      static_cast<size_t>(Joints::RL_j1),
      static_cast<size_t>(Joints::RL_j2),
      static_cast<size_t>(Joints::RR_j0),
      static_cast<size_t>(Joints::RR_j1),
      static_cast<size_t>(Joints::RR_j2),
      static_cast<size_t>(Joints::FR_j0),
      static_cast<size_t>(Joints::FR_j1),
      static_cast<size_t>(Joints::FR_j2),
      static_cast<size_t>(Joints::FL_j0),
      static_cast<size_t>(Joints::FL_j1),
      static_cast<size_t>(Joints::FL_j2)>;
};
template<typename Seq>
struct ReorderLegsToTensor;

template<size_t... Is>
struct ReorderLegsToTensor<std::index_sequence<Is...>>
{
  using type = std::index_sequence<static_cast<size_t>(Legs::FR),
      static_cast<size_t>(Legs::RR),
      static_cast<size_t>(Legs::FR),
      static_cast<size_t>(Legs::FL)>;
};

// it must be in same order as in the neural network and in the ReorderToTensor
enum class NetworkJoints
{
  FR_j0, FR_j1, FR_j2,
  FL_j0, FL_j1, FL_j2,
  RR_j0, RR_j1, RR_j2,
  RL_j0, RL_j1, RL_j2,
  SPINE_0,
};

enum class NetworkLegs
{
  FR, FL, RL, RR
};

template<typename Seq>
struct ReorderToMsg;

template<size_t... Is>
struct ReorderToMsg<std::index_sequence<Is...>>
{
  using type = std::index_sequence<static_cast<size_t>(NetworkJoints::FR_j0),
      static_cast<size_t>(NetworkJoints::FR_j1),
      static_cast<size_t>(NetworkJoints::FR_j2),
      static_cast<size_t>(NetworkJoints::FL_j0),
      static_cast<size_t>(NetworkJoints::FL_j1),
      static_cast<size_t>(NetworkJoints::FL_j2),
      static_cast<size_t>(NetworkJoints::RL_j0),
      static_cast<size_t>(NetworkJoints::RL_j1),
      static_cast<size_t>(NetworkJoints::RL_j2),
      static_cast<size_t>(NetworkJoints::RR_j0),
      static_cast<size_t>(NetworkJoints::RR_j1),
      static_cast<size_t>(NetworkJoints::RR_j2),
      static_cast<size_t>(NetworkJoints::SPINE_0)>;
};

template<typename T, size_t... Is>
std::array<T, sizeof...(Is)> reorder(
  const std::array<T, sizeof...(Is)> & arr,
  std::index_sequence<Is...>)
{
  return {arr[Is] ...};
}

template<typename T, size_t... Is>
std::array<T, sizeof...(Is)> reorderVectorToArray(
  const std::vector<T> & vec,
  std::index_sequence<Is...>)
{
  return {vec[Is] ...};
}
template<typename T, size_t... Is>
std::vector<T> reorderVector(
  const std::vector<T> & vec,
  std::index_sequence<Is...>)
{
  return {vec[Is] ...};
}

}  // namespace hb40_neural_controller

#endif  // HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_CONSTS_HPP_
