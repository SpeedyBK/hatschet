# Problem instances in `chstone` and `MachSuite`

## Source / construction (excerpt from [1])
> C applications from the HLS benchmark suites CHStone [2] and MachSuite [3] are compiled with Clang to LLVM-IR. We use LLVM/Clang version 3.3, optimisation preset `-O2` with loop unrolling disabled, and perform exhaustive inlining.

> Nymble [4] constructs per-loop control-data-flow graphs (CDFG), where the original control flow is replaced by multiplexers and predicated operations. Nested loops become special operations in the graph, thus making it possible to modulo schedule all loops in the application instead of being limited to the most deeply nested ones.
Nymble also constructs edges to retain the sequential order amongst memory access operations where needed, as determined by LLVM's dependence analysis.

> We use operator latencies and physical delays from the Bambu HLS framework's [5] extensive operator library for a Xilinx xc7vx690 device. For each operator, we choose the lowest-latency variant that is estimated to achieve a frequency of at least 250 MHz. The edges to limit operator chaining are constructed with a simple path-based approach similar to [6] and aim to enforce a maximum cycle time of 5 ns.

> The MSP instances use the following resource types: memory load (2 available)/store (1), nested loop (1), integer division (8), floating-point addition (4), FP subtraction (4), FP multiplication (4), other FP operations (2 each).

## Chaining edges
The dependence graphs in the `*.200MHz.graphml` files contain precomputed edges that limit the maximum cycle time to 5ns, i.e. 200 MHz. The graphs in the other `graphml` files contain only edges with *delay=1* from chainable ops (*delay=0*) to non-chainable ops (*delay>0*).

## References

[1] J. Oppermann, M. Reuter-Oppermann, L. Sommer, O. Sinnen, and A. Koch, "Dependence Graph Preprocessing for Faster Exact Modulo Scheduling in High-level Synthesis" in 28th International Conference on Field Programmable Logic and Applications, FPL 2018, Dublin, Ireland, August 27-31, 2018.

[2] Y. Hara, H. Tomiyama, S. Honda, and H. Takada, "Proposal and quantitative analysis of the chstone benchmark program suite for practical c-based high-level synthesis" JIP, vol. 17, pp. 242-254, 2009.

[3] B. Reagen, R. Adolf, Y. S. Shao, G. Wei, and D. M. Brooks, "Machsuite: Benchmarks for accelerator design and customized architectures" in 2014 IEEE International Symposium on Workload Characterization, IISWC 2014, Raleigh, NC, USA, October 26-28, 2014.

[4] J. Huthmann, B. Liebig, J. Oppermann, and A. Koch, "Hardware/software co-compilation with the nymble system" in 2013 8th International Workshop on Reconﬁgurable and Communication-Centric Systems-on-Chip (ReCoSoC), Darmstadt, Germany, July 10-12, 2013.

[5] C. Pilato and F. Ferrandi, "Bambu: A modular framework for the high level synthesis of memory-intensive applications" in 23rd International Conference on Field Programmable Logic and Applications, FPL 2013, Porto, Portugal, September 2-4, 2013.

[6] C. Hwang, J. Lee, and Y. Hsu, "A formal approach to the scheduling problem in high level synthesis" IEEE Trans. on CAD of Integrated Circuits and Systems, vol. 10, no. 4, pp. 464-475, 1991.
