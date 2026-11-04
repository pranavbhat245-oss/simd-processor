# 4-Lane SIMD Processor with a UVM Verification Environment

A GPU-inspired 4-lane SIMD datapath in Verilog, verified with a self-checking
testbench and a UVM environment in Questa (Intel FPGA Starter Edition).
Every operation has the same fixed latency: a transaction accepted at clock N
produces its result with `valid_out` at clock N+10.

## Design (`rtl/simd_processor.v`)

- Four independent 8-bit lanes behind one centralized decode stage (SIMT-style dispatch).
- 16 operations: ADD, SUB, MULT, DIV, MAC, DOT, SAD, SQRT, GCD, POPCOUNT,
  saturating add, ReLU, Leaky ReLU, MIN, MAX, ABS.
- Ready/valid interface with whole-pipeline stall (`ready_out` back-pressure).
- 4-bit lane mask: inactive lanes are excluded from DOT / MAC / SAD reductions.
- Signed and unsigned modes.
- Four exception codes: divide-by-zero, overflow, underflow, invalid square root.
- Fixed 10-cycle latency for all 16 operations.

## Verification

| Part | Where | What it does |
|---|---|---|
| Plain testbench | `tb/plain/` | Self-checking Verilog testbench: latency, data, mask, exceptions, reduction, stall hold, reset |
| Reference model | `tb/uvm/simd_ref_pkg.sv` | Golden model in pure SystemVerilog, written independently of the RTL |
| UVM environment | `tb/uvm/` | Agent (driver, monitor, sequencer), scoreboard, coverage collector, tests |
| Assertions | `tb/uvm/simd_if.sv` | SVA protocol checks: output hold and no accept during stall, reduction/valid alignment, lane-valid consistency, no flags when idle |
| Model sanity bench | `sim/ref_check_tb.sv` | Non-UVM bench that checks the reference model against the DUT |

The scoreboard checks every transaction for: 10-cycle latency, control signals,
lane mask, lane data, exception flags and codes, masked lanes staying quiet, and
reduction results. It also fails a test that compares zero transactions, so a
test that does nothing cannot pass.

### Results

| Test | Transactions compared | Errors | Functional coverage |
|---|---|---|---|
| `simd_smoke_test` | 4,613 | 0 | 97.9% (mask bins not exercised) |
| `simd_mask_test` | 720 | 0 | lane mask 15/15 |
| `simd_full_test` | 5,333 | 0 | **100% (658/658 bins)** |

`simd_random_test`, `simd_backpressure_test`, `simd_reset_test` and
`simd_exhaustive_test` are included; their results are not recorded here yet.

![Latency waveform](docs/images/waveform_latency.png)
![Scoreboard summary](docs/images/scoreboard_summary.png)
![Coverage report](docs/images/coverage_report.png)


## How to run

Requires Questa (tested on Intel FPGA Starter Edition 2025, UVM 1.1d built in).

```
cd sim
vsim -c -do run_questa.do          # batch run, edit TEST at the top of the script
vsim    -do run_questa.do          # GUI run: set GUI 1 in the script
```

Or by hand:

```
vlib work
vlog ../rtl/simd_processor.v
vlog -sv -L mtiUvm +incdir+../tb/uvm ../tb/uvm/simd_ref_pkg.sv ../tb/uvm/simd_if.sv ../tb/uvm/simd_pkg.sv ../tb/uvm/tb_top.sv
vsim -voptargs=+acc -L mtiUvm work.tb_top +UVM_TESTNAME=simd_full_test
add wave /tb_top/sif/*
run -all
```

Keep the `.svh` files in the same folder as `simd_pkg.sv`; it includes them.

## Limitations

- Starter Edition has no licence for constrained-random (`randomize()`) or
  covergroups. Stimulus uses `$urandom`, and functional coverage is implemented
  with hand-written counter bins. The structure follows UVM practice, but this
  is not true constrained-random verification.
- "100% coverage" means every bin I defined was hit. It is not a claim that the
  design is fully verified. Code coverage has not been measured.
- Known design behaviours, not changed: signed -128 / -1 returns +128 with no
  flag; unsigned ADD and SUB wrap to 8 bits while signed returns 9 bits;
  signed saturating add flags OVERFLOW (not UNDERFLOW) on negative saturation;
  a transaction with `mask = 0` is never accepted (`ready_in` depends on the mask).

## Acknowledgements

The RTL was designed and written by me. The UVM environment was developed with
AI assistance (Claude) while I was learning UVM, and the code documentation,
README and repository structure were also prepared with AI assistance.

Author: Pranav Bhat · [GitHub](https://github.com/pranavbhat245) · [LinkedIn](https://linkedin.com/in/pranavbhat245)
