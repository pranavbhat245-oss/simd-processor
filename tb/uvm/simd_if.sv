`timescale 1ns/1ps
// ============================================================================
// simd_if : signal bundle + clocking blocks + protocol assertions
//   +define+SIMD_NO_SVA  to compile out the assertions
// ============================================================================
interface simd_if (input logic clk);

  // ---- DUT inputs (driven procedurally: UVM driver via drv_cb, reset by tasks) ----
  logic              rst;
  logic              valid_in;
  logic              ready_out;
  logic              signed_mode;
  logic              relu_en;
  logic [3:0]        opcode;
  logic [3:0]        mask;
  logic [3:0][7:0]   a;
  logic [3:0][7:0]   b;
  logic [15:0]       c;

  // ---- DUT outputs (nets, driven by the DUT ports) ----
  wire               ready_in;
  wire [3:0][15:0]   lane_result;
  wire [3:0]         lane_valid_out;
  wire [18:0]        dot_result;
  wire [18:0]        mac_result;
  wire [18:0]        sad_result;
  wire               reduction_valid_out;
  wire               valid_out;
  wire [3:0]         opcode_out;
  wire               signed_mode_out;
  wire [3:0]         div_by_zero;
  wire [3:0]         overflow;
  wire [3:0]         underflow;
  wire [3:0]         sqrt_invalid;
  wire [3:0]         exception_out;
  wire [3:0][2:0]    exception_code;

  int unsigned       sva_fail_cnt = 0;

  initial begin
    valid_in = 1'b0;  ready_out = 1'b1;  signed_mode = 1'b0;  relu_en = 1'b0;
    opcode = '0;  mask = 4'hF;  a = '0;  b = '0;  c = '0;
    rst = 1'b1;
    repeat (4) @(posedge clk);
    #1 rst = 1'b0;
  end

  // Asynchronous reset pulse, held for 'cycles' clock edges.
  task automatic pulse_reset(int unsigned cycles);
    rst = 1'b1;
    repeat (cycles) @(posedge clk);
    #1 rst = 1'b0;
  endtask

  // Driver view: drives inputs 1 ns after the edge, samples just before it.
  clocking drv_cb @(posedge clk);
    default input #1step output #1;
    output valid_in, ready_out, signed_mode, relu_en, opcode, mask, a, b, c;
    input  ready_in, valid_out;
  endclocking

  // Monitor view: everything is an input, sampled just before the edge.
  clocking mon_cb @(posedge clk);
    default input #1step;
    input valid_in, ready_in, ready_out, signed_mode, relu_en, opcode, mask, a, b, c;
    input lane_result, lane_valid_out, dot_result, mac_result, sad_result;
    input reduction_valid_out, valid_out, opcode_out, signed_mode_out;
    input div_by_zero, overflow, underflow, sqrt_invalid, exception_out, exception_code;
  endclocking

`ifndef SIMD_NO_SVA
  // While the output is stalled the whole pipeline must hold still.
  a_hold_outputs : assert property (@(posedge clk) disable iff (rst)
      (valid_out && !ready_out) |=>
        (valid_out && $stable(lane_result) && $stable(lane_valid_out) &&
         $stable(opcode_out) && $stable(signed_mode_out) &&
         $stable(dot_result) && $stable(mac_result) && $stable(sad_result) &&
         $stable(reduction_valid_out) && $stable(exception_code)))
    else begin sva_fail_cnt++; $error("SVA a_hold_outputs: outputs changed while stalled"); end

  // No new transaction may be accepted while the output is stalled.
  a_no_accept_when_stalled : assert property (@(posedge clk) disable iff (rst)
      (valid_out && !ready_out) |-> !ready_in)
    else begin sva_fail_cnt++; $error("SVA a_no_accept_when_stalled: ready_in high during stall"); end

  // reduction_valid_out must be aligned with valid_out (catches the N+11 bug).
  a_red_aligned : assert property (@(posedge clk) disable iff (rst)
      valid_out |-> (reduction_valid_out == (opcode_out inside {4'd4, 4'd5, 4'd14})))
    else begin sva_fail_cnt++; $error("SVA a_red_aligned: reduction_valid_out not aligned with valid_out"); end

  a_red_implies_valid : assert property (@(posedge clk) disable iff (rst)
      reduction_valid_out |-> valid_out)
    else begin sva_fail_cnt++; $error("SVA a_red_implies_valid: reduction_valid_out without valid_out"); end

  // Per-lane valid bits must agree with the transaction-level valid.
  a_lane_valid_consistent : assert property (@(posedge clk) disable iff (rst)
      valid_out == (|lane_valid_out))
    else begin sva_fail_cnt++; $error("SVA a_lane_valid_consistent: lane_valid_out vs valid_out"); end

  // Nothing may be flagged when no transaction is at the output.
  a_quiet_when_idle : assert property (@(posedge clk) disable iff (rst)
      !valid_out |-> ((div_by_zero | overflow | underflow | sqrt_invalid | exception_out) == 4'b0))
    else begin sva_fail_cnt++; $error("SVA a_quiet_when_idle: exception flags while !valid_out"); end
`endif

endinterface
