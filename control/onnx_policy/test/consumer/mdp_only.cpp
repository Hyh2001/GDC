#include "onnx_policy/mdp/action/actions.hpp"
#include "onnx_policy/mdp/observation/observations.hpp"
int main()
{
  onnx_policy::action::JointPositionActionTerm term({"", 1, {2}, {1}});
  term.process_actions({3});
  return term.processed_actions() == std::vector<double>{7} ? 0 : 1;
}
