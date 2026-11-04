`timescale 1ns/1ps
module tb_top;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import simd_pkg::*;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  simd_if sif (.clk(clk));

  SimdTop #(.WIDTH(8), .RED_W(19)) dut (
    .clk(clk), .rst(sif.rst),
    .valid_in(sif.valid_in), .ready_in(sif.ready_in), .ready_out(sif.ready_out),
    .signed_mode(sif.signed_mode), .relu_en(sif.relu_en),
    .opcode(sif.opcode), .mask(sif.mask),
    .a0(sif.a[0]), .a1(sif.a[1]), .a2(sif.a[2]), .a3(sif.a[3]),
    .b0(sif.b[0]), .b1(sif.b[1]), .b2(sif.b[2]), .b3(sif.b[3]),
    .c(sif.c),
    .lane0_result(sif.lane_result[0]), .lane1_result(sif.lane_result[1]),
    .lane2_result(sif.lane_result[2]), .lane3_result(sif.lane_result[3]),
    .lane_valid_out(sif.lane_valid_out),
    .dot_result(sif.dot_result), .mac_result(sif.mac_result), .sad_result(sif.sad_result),
    .reduction_valid_out(sif.reduction_valid_out), .valid_out(sif.valid_out),
    .opcode_out(sif.opcode_out), .signed_mode_out(sif.signed_mode_out),
    .div_by_zero(sif.div_by_zero), .overflow(sif.overflow), .underflow(sif.underflow),
    .sqrt_invalid(sif.sqrt_invalid), .exception_out(sif.exception_out),
    .exception_code0(sif.exception_code[0]), .exception_code1(sif.exception_code[1]),
    .exception_code2(sif.exception_code[2]), .exception_code3(sif.exception_code[3])
  );

  initial begin
    uvm_config_db#(virtual simd_if)::set(null, "*", "vif", sif);
    run_test();
  end
endmodule
