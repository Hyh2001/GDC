"""Optional fixture regeneration; requires numpy and onnx, not used by tests."""

from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def main():
    root = Path(__file__).parent / "fixtures"
    root.mkdir(exist_ok=True)

    def value(name, shape, dtype=TensorProto.FLOAT):
        return helper.make_tensor_value_info(name, dtype, shape)

    def tensor(name, values, dtype=np.float32):
        return numpy_helper.from_array(np.array(values, dtype=dtype), name)

    def save(name, nodes, inputs, outputs, initializers=()):
        model = helper.make_model(
            helper.make_graph(nodes, name, inputs, outputs, list(initializers)),
            opset_imports=[helper.make_opsetid("", 13)],
        )
        model.ir_version = 8
        onnx.checker.check_model(model)
        onnx.save(model, root / name)

    save(
        "mlp.onnx", [helper.make_node("MatMul", ["obs", "W"], ["act"])],
        [value("obs", [1, 3])], [value("act", [1, 2])],
        [tensor("W", [[1, 0], [1, 0], [0, 1]])],
    )
    save(
        "controller.onnx", [helper.make_node("MatMul", ["obs", "W"], ["act"])],
        [value("obs", [1, 4])], [value("act", [1, 2])],
        [tensor("W", [[1, 0], [0, 1], [1, 0], [0, 1]])],
    )
    for name, size in [("rnn.onnx", 2), ("controller_rnn.onnx", 4)]:
        # The +1 makes even zero-input warm-up change memory, testing reset.
        nodes = [
            helper.make_node("MatMul", ["obs", "W"], ["base"]),
            helper.make_node("Squeeze", ["h_in", "axes"], ["memory"]),
            helper.make_node("Add", ["base", "memory"], ["act"]),
            helper.make_node("Add", ["act", "one"], ["next_memory"]),
            helper.make_node("Unsqueeze", ["next_memory", "axes"], ["h_out"]),
        ]
        save(
            name, nodes,
            [value("h_in", [1, 1, 2]), value("obs", [1, size])],
            [value("h_out", [1, 1, 2]), value("act", [1, 2])],
            [tensor("W", np.vstack([np.eye(2)] * (size // 2))),
             tensor("axes", [0], np.int64), tensor("one", [1])],
        )
    for name, shape, dtype in [
        ("dynamic.onnx", ["batch", "features"], TensorProto.FLOAT),
        ("double.onnx", [1, 2], TensorProto.DOUBLE),
        ("rank.onnx", [2], TensorProto.FLOAT),
    ]:
        save(name, [helper.make_node("Identity", ["obs"], ["act"])],
             [value("obs", shape, dtype)], [value("act", shape, dtype)])
    save(
        "extra.onnx", [helper.make_node("Add", ["obs", "other"], ["act"])],
        [value("obs", [1, 2]), value("other", [1, 2])], [value("act", [1, 2])],
    )
    save(
        "overflow.onnx", [helper.make_node("Mul", ["obs", "scale"], ["act"])],
        [value("obs", [1, 2])], [value("act", [1, 2])], [tensor("scale", [100])],
    )


if __name__ == "__main__":
    main()
