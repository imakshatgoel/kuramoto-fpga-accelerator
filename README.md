# Kuramoto FPGA Accelerator

Verilog hardware that solves the Kuramoto oscillator equations in parallel and uses the result to approximate the Max-Cut graph problem.

This was a research project taken under Prof. Debanjan Bhowmik from the Department of Electrical Engineering, IIT Bombay.

*October 2025 - November 2025*

## How it works

Each node of the graph is an oscillator with a phase θᵢ. At every time step, each phase is updated with:

```
dθᵢ/dt = Σⱼ Kᵢⱼ · sin(θⱼ − θᵢ) + 2·K_SHIL · sin(2θᵢ)
θᵢ ← θᵢ − Δt · dθᵢ/dt
```

Connected oscillators push toward opposite phases, and the SHIL term settles every phase at 0 or π. At the end, the nodes near 0 form one group and the nodes near π form the other.

The design has one computational unit per oscillator, and the units are connected by a shared bus:

1. In slot k, unit k puts its phase on the bus.
2. Every neighbour of node k computes sin(θₖ − θᵢ) with its CORDIC core and adds the result to its running sum. Unit k uses the same slot to compute its own sin(2θₖ).
3. After N slots, all units update their phases in the same clock cycle.

**Parameters:** N = 10, Kᵢⱼ = 0.5 per edge, K_SHIL = 0.5, Δt = 2⁻⁸, θᵢ(0) = 2πi/N + π/5, and 1000 Euler steps. All values are 32-bit fixed point (Q3.29).

## Speedup

One step takes N bus slots of 37 clock cycles each, so a step takes **37N + 2 cycles**. That grows linearly with N. A single unit doing the same work one sine at a time would need N² sine evaluations per step, which is about 34N² cycles.

| N | Parallel design (measured) | Serial (estimate) | Speedup |
|---|---|---|---|
| 4 | 150 cycles | 544 cycles | 3.6× |
| 6 | 224 cycles | 1,224 cycles | 5.5× |
| 8 | 298 cycles | 2,176 cycles | 7.3× |
| 10 | 372 cycles | 3,400 cycles | 9.1× |

The speedup grows roughly in proportion to N. On an Intel MAX 10 10M50 FPGA the design runs at up to 84.9 MHz, so 1000 steps for N = 10 take about 4.4 ms.

## Verification

- `testbench.v` runs six graphs: the default 10-node graph, Möbius ladders with 4, 6, 8 and 10 nodes, and a complete 10-node graph. After every step it checks the hardware against a floating-point model. The largest difference is 1.9 × 10⁻⁹ rad.
- `tb_cordic.v` checks the CORDIC sine over 4,104 angles. The largest error is 2.2 × 10⁻⁸.
- Both testbenches pass in ModelSim and in Icarus Verilog.

## Files

| File | Contents |
|---|---|
| `kuramoto_solver.v` | Top level: controller FSM, shared bus and partition readout. `N`, `ITERATIONS` and the adjacency matrix `ADJ` are parameters. |
| `computational_unit.v` | One oscillator: phase register, CORDIC control and Euler update |
| `cordic_sine.v` | 32-iteration CORDIC sine |
| `testbench.v`, `tb_cordic.v` | Self-checking testbenches |
| `Kuramoto.qpf`, `Kuramoto.qsf` | Quartus Prime 18.1 project |
| `Prof-Reasearch Paper.pdf` | Background reading |

## Running

**ModelSim:**

```
vlib work
vlog cordic_sine.v computational_unit.v kuramoto_solver.v testbench.v tb_cordic.v
vsim -c testbench -do "run -all; quit -f"
```

**Icarus Verilog:**

```
iverilog -o tb.vvp testbench.v kuramoto_solver.v computational_unit.v cordic_sine.v
vvp -n tb.vvp
```

Each run ends with `RESULT: PASS` or `RESULT: FAIL`.

## Notes

- The design fits a MAX 10 10M50, using 32% of its logic. It is too large for the 10M08 that the project file targets. It has not been tested on a physical board yet.
- With these fixed parameters, the phases mostly settle near where they started, so the cut is often not the best possible. It found the optimal cut on 2 of the 6 test graphs. Ramping K_SHIL up from zero during the run would help.
