# Problem instances in `chstone` and `MachSuite`

## Source / construction
> C applications from the HLS benchmark suites CHStone and MachSuite are compiled with Clang (we use LLVM/Clang version 3.3, optimisation preset `-O2` with loop unrolling disabled, and perform exhaustive inlining) to LLVM-IR.
Nymble constructs per-loop control-data-flow graphs (CDFG), where the original control flow is replaced by multiplexers and predicated operations.
Nested loops become special operations in the graph, thus making it possible to modulo schedule all loops in the application instead of being limited to the most deeply nested ones.
Nymble also constructs edges to retain the sequential order amongst memory access operations where needed, as determined by LLVM's dependence analysis.
We use operator latencies and physical delays from the Bambu HLS framework's extensive operator library for a Xilinx `xc7vx690` device.
For each operator, we choose the lowest-latency variant that is estimated to achieve a frequency of at least 250 MHz.
The edges to limit operator chaining are constructed with a simple path-based approach similar and aim to enforce a maximum cycle time of 5 ns.
The MSP instances use the following resource types: memory load (2 available)/store (1), nested loop (1), integer division (8), floating-point addition (4), FP subtraction (4), FP multiplication (4), other FP operations (2 each).

(from the paper that JO submitted to FPL'18, so don't copy verbatim)

## Chaining edges
The dependence graphs in the `*.200MHz.graphml` files contain precomputed edges that limit the maximum cycle time to 5ns, i.e. 200 MHz. The graphs in the other `graphml` files contain only edges with delay=1 from chainable ops (delay=0) to non-chainable ops (delay>0).
