#pragma once
#ifndef PINOCCHIO_UTILS_HPP
#define PINOCCHIO_UTILS_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace pinocchio_utils
{
inline std::vector<int> build_joint_reorder_map(
  const std::vector<std::string> & joint_names,
  const pinocchio::Model & model)
{
  // Step 1: Build compact Pinocchio joint order (0…n-1)
  std::unordered_map<std::string, int> pin_joint_space_index;

  int joint_space_idx = 0;

  for (pinocchio::JointIndex j = 1; j < model.njoints; ++j)
  {
      const std::string &name = model.names[j];

      // Only include joints that appear in joint_names
      if (std::find(joint_names.begin(),
                    joint_names.end(),
                    name) != joint_names.end())
      {
          pin_joint_space_index[name] = joint_space_idx++;
      }
  }

  // Step 2: Build permutation map (new_index = perm[old_index])
  std::vector<int> perm(joint_names.size());

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
      const auto &name = joint_names[i];

      if (pin_joint_space_index.find(name) == pin_joint_space_index.end())
      {
          throw std::runtime_error(
              "Joint not found in Pinocchio model: " + name);
      }

      perm[i] = pin_joint_space_index[name];
  }

  return perm;
}

inline Eigen::VectorXd reorder_joint_to_pinocchio(
  const Eigen::VectorXd & x,
  const std::vector<int> & map)
{
  if (x.size() != static_cast<int>(map.size()))
  {
      throw std::runtime_error("Vector size does not match map size");
  }

  Eigen::VectorXd x_reordered(x.size());

  for (size_t i = 0; i < map.size(); ++i)
  {
      x_reordered[map[i]] = x[i];
  }

  return x_reordered;
}

inline Eigen::VectorXd reorder_pinocchio_to_joint(
  const Eigen::VectorXd & x_pinocchio,
  const std::vector<int> & map)
{
  if (x_pinocchio.size() != static_cast<int>(map.size()))
  {
      throw std::runtime_error("Vector size does not match map size");
  }

  Eigen::VectorXd x_reordered(x_pinocchio.size());

  for (size_t i = 0; i < map.size(); ++i)
  {
      x_reordered[i] = x_pinocchio[map[i]];  // Read from Pinocchio position map[i]
  }

  return x_reordered;
}

inline std::vector<int> build_contact_reorder_map(
  std::vector<std::string>& contact_names,
  const pinocchio::Model & model)
{
  struct ContactInfo
  {
      int old_index;
      int parent_joint_index;
  };

  std::vector<ContactInfo> contacts;

  for (size_t i = 0; i < contact_names.size(); ++i)
  {
      const std::string &name = contact_names[i];

      if (!model.existFrame(name))
          throw std::runtime_error("Frame not found: " + name);

      auto fid = model.getFrameId(name);
      auto parent_joint = model.frames[fid].parent;

      contacts.push_back({static_cast<int>(i),
                          static_cast<int>(parent_joint)});
  }

  // Sort by Pinocchio joint order
  std::sort(contacts.begin(), contacts.end(),
            [](const ContactInfo &a, const ContactInfo &b)
            {
                return a.parent_joint_index < b.parent_joint_index;
            });

  // Build permutation (perm[new] = old)
  std::vector<int> perm(contacts.size());

  for (size_t new_idx = 0; new_idx < contacts.size(); ++new_idx)
  {
      perm[new_idx] = contacts[new_idx].old_index;
  }

  // 🔥 Reorder contact_names in-place
  std::vector<std::string> reordered_names(contact_names.size());

  for (size_t new_idx = 0; new_idx < perm.size(); ++new_idx)
  {
      reordered_names[new_idx] = contact_names[perm[new_idx]];
  }

  contact_names = reordered_names;

  return perm;
}

inline Eigen::VectorXd reorder_contact_to_pinocchio(
  const Eigen::VectorXd & x,
  const std::vector<int> & map)
{
  if (x.size() != static_cast<int>(map.size()))
      throw std::runtime_error("Size mismatch in applyPermutation");

  Eigen::VectorXd x_reordered(x.size());

  for (size_t i = 0; i < map.size(); ++i)
  {
      x_reordered[i] = x[map[i]];
  }

  return x_reordered;
}

inline Eigen::VectorXd reorder_pinocchio_to_contact(
  const Eigen::VectorXd & x_pinocchio,
  const std::vector<int> & map)
{
  if (x_pinocchio.size() != static_cast<int>(map.size()))
      throw std::runtime_error("Size mismatch in reorder_pinocchio_to_contact");

  Eigen::VectorXd x_reordered(x_pinocchio.size());

  for (size_t i = 0; i < map.size(); ++i)
  {
      x_reordered[map[i]] = x_pinocchio[i];  // Read from Pinocchio order, write to contact order
  }

  return x_reordered;
}

inline bool is_floating_base(const pinocchio::Model & model)
{
  if (model.njoints <= 1)
  {
    return false;
  }

  const auto& root_joint = model.joints[1];
  const std::string root_joint_name = root_joint.shortname();
  return root_joint_name == "JointModelFreeFlyer" ||
         root_joint_name == "JointModelPlanar" ||
         root_joint_name == "JointModelTranslation" ||
         root_joint_name == "JointModelSpherical";
}

} // namespace pinocchio_utils


#endif // PINOCCHIO_UTILS_HPP
