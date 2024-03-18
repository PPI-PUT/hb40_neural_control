#ifndef HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_CONSTS_HPP_
#define HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_CONSTS_HPP_

#include <cstddef>

namespace hb40_neural_controller
{
constexpr float SCALED_FACTOR = 0.25;

constexpr size_t _HB40_FR_ABDUCTOR = 0;
constexpr size_t _HB40_FR_HIP = 1;
constexpr size_t _HB40_FR_KNEE = 2;
constexpr size_t _HB40_FL_ABDUCTOR = 3;
constexpr size_t _HB40_FL_HIP = 4;
constexpr size_t _HB40_FL_KNEE = 5;
constexpr size_t _HB40_RL_ABDUCTOR = 6;
constexpr size_t _HB40_RL_HIP = 7;
constexpr size_t _HB40_RL_KNEE = 8;
constexpr size_t _HB40_RR_ABDUCTOR = 9;
constexpr size_t _HB40_RR_HIP = 10;
constexpr size_t _HB40_RR_KNEE = 11;
constexpr size_t _HB40_SPINE_0 = 12;

constexpr size_t MAB_FR = 0;
constexpr size_t MAB_FL = 1;
constexpr size_t MAB_RL = 2;
constexpr size_t MAB_RR = 3;


// HARDCODING IS MY NEW FAVOURITE THING
// USE ANTI-CLOCKWISE ORDER... OR NOT
constexpr size_t FR_CONTACT = 1; // CLOCKWISE ORDER? NOPE
constexpr size_t FL_CONTACT = 0; // WATCH OUT MAGIC SHIFT
constexpr size_t RL_CONTACT = 2; // CLOCKIWSE ORDER? NOPE
constexpr size_t RR_CONTACT = 3; // PLEASE CHECK TWICE

constexpr size_t FR_CYCLE = 0; // WATCH OUT MAGIC SHIFT
constexpr size_t FL_CYCLE = 1; // WATCH OUT MAGIC SHIFT
constexpr size_t RL_CYCLE = 3; // ORDER?
constexpr size_t RR_CYCLE = 2; // PLEASE CHECK TWICE

// HERE WE GO AGAIN, NETWORK JOINT NAME ARE DIFFERENT SO ...
// TU SIE ZATEGUJE
constexpr size_t FR_HIP = 0;
constexpr size_t FR_THIGH = 1;
constexpr size_t FR_CALF = 2;
constexpr size_t FL_HIP = 3;
constexpr size_t FL_THIGH = 4;
constexpr size_t FL_CALF = 5;
constexpr size_t RR_HIP = 6;
constexpr size_t RR_THIGH = 7;
constexpr size_t RR_CALF = 8;
constexpr size_t RL_HIP = 9;
constexpr size_t RL_THIGH = 10;
constexpr size_t RL_CALF = 11;
// DID YOU KNOW THAT I LOVE HARDCODING?
// MAB JOINTS NAME ARE DIFFERNT FROM THE UNITREE ONES
// SO I HAVE TO HARDCODE IT TOO
// UNITREE - MAB
// hip - abductor
// thigh - hip
// calf - knee
constexpr size_t MAB_FR_HIP = _HB40_FR_ABDUCTOR;
constexpr size_t MAB_FR_THIGH = _HB40_FR_HIP;
constexpr size_t MAB_FR_CALF = _HB40_FR_KNEE;
constexpr size_t MAB_FL_HIP = _HB40_FL_ABDUCTOR;
constexpr size_t MAB_FL_THIGH = _HB40_FL_HIP;
constexpr size_t MAB_FL_CALF = _HB40_FL_KNEE;
constexpr size_t MAB_RR_HIP = _HB40_RR_ABDUCTOR;
constexpr size_t MAB_RR_THIGH = _HB40_RR_HIP;
constexpr size_t MAB_RR_CALF = _HB40_RR_KNEE;
constexpr size_t MAB_RL_HIP = _HB40_RL_ABDUCTOR;
constexpr size_t MAB_RL_THIGH = _HB40_RL_HIP;
constexpr size_t MAB_RL_CALF = _HB40_RL_KNEE;

// I WILL GIVE YOU THAT, FUN IS FOR FREE
// float state[53] = {
//   fr_hip_pos, fr_thigh_pos, fr_calf_pos,
//   fl_hip_pos, fl_thigh_pos, fl_calf_pos,
//   rr_hip_pos, rr_thigh_pos, rr_calf_pos,
//   rl_hip_pos, rl_thigh_pos, rl_calf_pos,
//   roll_vel, pitch_vel, yaw_vel,
//   fr_hip_vel, fr_thigh_vel, fr_calf_vel,
//   fl_hip_vel, fl_thigh_vel, fl_calf_vel,
//   rr_hip_vel, rr_thigh_vel, rr_calf_vel,
//   rl_hip_vel, rl_thigh_vel, rl_calf_vel,
//   goal_x_vel, goal_y_vel, goal_yaw_vel,
//   fl_contact, fr_contact, rl_contact, rr_contact,
//   gravity_vec_1, gravity_vec_2, gravity_vec_3,
//   last_action_0, last_action_1, last_action_2,
//   last_action_3, last_action_4, last_action_5,
//   last_action_6, last_action_7, last_action_8,
//   last_action_9, last_action_10, last_action_11,
//   fr_cycles_since_last_contact, fl_cycles_since_last_contact,
//   rr_cycles_since_last_contact, rl_cycles_since_last_contact
// };

// EYE BLEEDING, I KNOW
// IM NOT SORRY, THIS IS FASTEST WAY TO DO IT
// I WILL FIX IT LATER, WE WILL FIX IT LATER
}  // namespace hb40_neural_controller
#endif  // HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_CONSTS_HPP_
