`timescale 1ns/1ps

// ============================================================================
// 4-Lane SIMD Processor - Verilog-2001
// ============================================================================
// Fixed transaction latency:
//   accepted at clock N  -> valid_out at clock N+10
// when ready_out remains high.
//
// FPGA-oriented:
//   * clock-enable (ce), no fabric-generated gated clocks
//   * a*b multiplier so synthesis can infer FPGA DSP resources
//   * performance/utilization counters are intentionally NOT in RTL
//
// Operations:
// ADD, SUB, MULT, DIV, MAC, DOT, RELU, LRELU,
// MIN, MAX, ABS, SATADD, SQRT, GCD, SAD, POPCOUNT
// ============================================================================

`ifndef SIMD_PROCESSOR_FIXED_V
`define SIMD_PROCESSOR_FIXED_V

// --------------------------------------------------------------------------
// Opcode constants
// --------------------------------------------------------------------------
`define OP_ADD      4'b0000
`define OP_SUB      4'b0001
`define OP_MULT     4'b0010
`define OP_DIV      4'b0011
`define OP_MAC      4'b0100
`define OP_DOT      4'b0101
`define OP_RELU     4'b0110
`define OP_LRELU    4'b0111
`define OP_MIN      4'b1000
`define OP_MAX      4'b1001
`define OP_ABS      4'b1010
`define OP_SATADD   4'b1011
`define OP_SQRT     4'b1100
`define OP_GCD      4'b1101
`define OP_SAD      4'b1110
`define OP_POPCOUNT 4'b1111

// Exception codes
`define EX_NONE     3'b000
`define EX_DIVZERO  3'b001
`define EX_OVERFLOW 3'b010
`define EX_UNDER    3'b011
`define EX_INVALID  3'b100

// ============================================================================
// DivUnit
// ============================================================================
// 8-bit radix-2 restoring division.
// Stage 0: input capture / magnitude conversion.
// Stages 1..8: one quotient-bit iteration per stage.
// Stages 9..10: output alignment.
// Total input-to-output latency = 10 clocks.
// ============================================================================
module DivUnit #(
    parameter WIDTH = 8
)(
    input                    clk,
    input                    rst,
    input                    ce,
    input                    valid_in,
    input                    signed_mode,
    input  [WIDTH-1:0]       a,
    input  [WIDTH-1:0]       b,
    output [2*WIDTH-1:0]     result,
    output                   valid_out,
    output                   div_by_zero
);

    reg [WIDTH-1:0] dividend_pipe [0:WIDTH];
    reg [WIDTH-1:0] divisor_pipe  [0:WIDTH];
    reg [WIDTH:0]   rem_pipe      [0:WIDTH];
    reg [WIDTH-1:0] quot_pipe     [0:WIDTH];
    reg             sign_pipe     [0:WIDTH];
    reg             dz_pipe       [0:WIDTH];
    reg             valid_pipe    [0:WIDTH];

    reg [WIDTH-1:0] quot_d9, quot_d10;
    reg             sign_d9, sign_d10;
    reg             dz_d9, dz_d10;
    reg             valid_d9, valid_d10;

    wire [WIDTH-1:0] mag_a;
    wire [WIDTH-1:0] mag_b;
    wire             quotient_sign;

    assign mag_a = (signed_mode && a[WIDTH-1]) ? (~a + 1'b1) : a;
    assign mag_b = (signed_mode && b[WIDTH-1]) ? (~b + 1'b1) : b;
    assign quotient_sign = signed_mode && (a[WIDTH-1] ^ b[WIDTH-1]);

    integer i;
    reg [WIDTH:0] rem_shift;
    reg [WIDTH:0] rem_next;
    reg [WIDTH-1:0] q_next;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i <= WIDTH; i = i + 1) begin
                dividend_pipe[i] <= 0;
                divisor_pipe[i]  <= 0;
                rem_pipe[i]      <= 0;
                quot_pipe[i]     <= 0;
                sign_pipe[i]     <= 0;
                dz_pipe[i]       <= 0;
                valid_pipe[i]    <= 0;
            end
            quot_d9   <= 0;
            quot_d10  <= 0;
            sign_d9   <= 0;
            sign_d10  <= 0;
            dz_d9     <= 0;
            dz_d10    <= 0;
            valid_d9  <= 0;
            valid_d10 <= 0;
        end
        else if (ce) begin
            dividend_pipe[0] <= mag_a;
            divisor_pipe[0]  <= mag_b;
            rem_pipe[0]      <= 0;
            quot_pipe[0]     <= 0;
            sign_pipe[0]     <= quotient_sign;
            dz_pipe[0]       <= valid_in && (b == {WIDTH{1'b0}});
            valid_pipe[0]    <= valid_in;

            for (i = 0; i < WIDTH; i = i + 1) begin
                rem_shift = {rem_pipe[i][WIDTH-1:0], dividend_pipe[i][WIDTH-1]};
                rem_next  = rem_pipe[i];
                q_next    = {quot_pipe[i][WIDTH-2:0], 1'b0};

                if (!dz_pipe[i]) begin
                    if (rem_shift >= {1'b0, divisor_pipe[i]}) begin
                        rem_next = rem_shift - {1'b0, divisor_pipe[i]};
                        q_next[0] = 1'b1;
                    end
                    else begin
                        rem_next = rem_shift;
                        q_next[0] = 1'b0;
                    end
                end

                dividend_pipe[i+1] <= {dividend_pipe[i][WIDTH-2:0], 1'b0};
                divisor_pipe[i+1]  <= divisor_pipe[i];
                rem_pipe[i+1]      <= rem_next;
                quot_pipe[i+1]     <= q_next;
                sign_pipe[i+1]     <= sign_pipe[i];
                dz_pipe[i+1]       <= dz_pipe[i];
                valid_pipe[i+1]    <= valid_pipe[i];
            end

            quot_d9   <= quot_pipe[WIDTH];
            quot_d10  <= quot_d9;
            sign_d9   <= sign_pipe[WIDTH];
            sign_d10  <= sign_d9;
            dz_d9     <= dz_pipe[WIDTH];
            dz_d10    <= dz_d9;
            valid_d9  <= valid_pipe[WIDTH];
            valid_d10 <= valid_d9;
        end
    end

    wire [2*WIDTH-1:0] q_unsigned;
    wire [2*WIDTH-1:0] q_signed;

    assign q_unsigned = {{WIDTH{1'b0}}, quot_d10};
    assign q_signed   = (~q_unsigned) + 1'b1;

    assign result      = sign_d10 ? q_signed : q_unsigned;
    assign valid_out   = valid_d10;
    assign div_by_zero = valid_d10 && dz_d10;

endmodule

// ============================================================================
// GcdUnit
// ============================================================================
// Binary GCD (Stein) with two reduction steps per pipeline stage.
// Exhaustive analysis for all 8-bit input pairs gives a maximum of 14
// two-step iterations for this formulation, so 7 compute stages are enough.
// Stage 0 = magnitude capture.
// Stages 1..7 = two binary-GCD reductions per stage.
// Stages 8..10 = result alignment.
// ============================================================================
module GcdUnit #(
    parameter WIDTH = 8
)(
    input                    clk,
    input                    rst,
    input                    ce,
    input                    valid_in,
    input                    signed_mode,
    input  [WIDTH-1:0]       a,
    input  [WIDTH-1:0]       b,
    output [2*WIDTH-1:0]     result,
    output                   valid_out
);

    localparam STAGES = 7;

    reg [WIDTH-1:0] a_pipe [0:STAGES];
    reg [WIDTH-1:0] b_pipe [0:STAGES];
    reg [3:0]       shift_pipe [0:STAGES];
    reg             v_pipe [0:STAGES];

    reg [WIDTH-1:0] result_d8, result_d9, result_d10;
    reg             valid_d8, valid_d9, valid_d10;

    wire [WIDTH-1:0] mag_a;
    wire [WIDTH-1:0] mag_b;

    assign mag_a = (signed_mode && a[WIDTH-1]) ? (~a + 1'b1) : a;
    assign mag_b = (signed_mode && b[WIDTH-1]) ? (~b + 1'b1) : b;

    function [2*WIDTH+3:0] gcd_two_steps;
        input [WIDTH-1:0] x_in;
        input [WIDTH-1:0] y_in;
        input [3:0]       shift_in;
        reg [WIDTH-1:0] x;
        reg [WIDTH-1:0] y;
        reg [3:0] common_shift;
        integer k;
        begin
            x = x_in;
            y = y_in;
            common_shift = shift_in;

            for (k = 0; k < 2; k = k + 1) begin
                if ((x == 0) || (y == 0)) begin
                    x = x;
                    y = y;
                end
                else if ((!x[0]) && (!y[0])) begin
                    x = x >> 1;
                    y = y >> 1;
                    common_shift = common_shift + 1'b1;
                end
                else if (!x[0]) begin
                    x = x >> 1;
                end
                else if (!y[0]) begin
                    y = y >> 1;
                end
                else if (x >= y) begin
                    x = (x - y) >> 1;
                end
                else begin
                    y = (y - x) >> 1;
                end
            end

            gcd_two_steps = {x, y, common_shift};
        end
    endfunction

    integer i;
    reg [2*WIDTH+3:0] fn_pack;
    reg [WIDTH-1:0] fn_x, fn_y;
    reg [3:0] fn_shift;
    reg [WIDTH-1:0] gcd_final;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i <= STAGES; i = i + 1) begin
                a_pipe[i] <= 0;
                b_pipe[i] <= 0;
                shift_pipe[i] <= 0;
                v_pipe[i] <= 0;
            end
            result_d8  <= 0;
            result_d9  <= 0;
            result_d10 <= 0;
            valid_d8   <= 0;
            valid_d9   <= 0;
            valid_d10  <= 0;
        end
        else if (ce) begin
            a_pipe[0] <= mag_a;
            b_pipe[0] <= mag_b;
            shift_pipe[0] <= 0;
            v_pipe[0] <= valid_in;

            for (i = 0; i < STAGES; i = i + 1) begin
                fn_pack = gcd_two_steps(a_pipe[i], b_pipe[i], shift_pipe[i]);
                fn_x = fn_pack[2*WIDTH+3:WIDTH+4];
                fn_y = fn_pack[WIDTH+3:4];
                fn_shift = fn_pack[3:0];

                a_pipe[i+1] <= fn_x;
                b_pipe[i+1] <= fn_y;
                shift_pipe[i+1] <= fn_shift;
                v_pipe[i+1] <= v_pipe[i];
            end

            if (b_pipe[STAGES] == 0)
                gcd_final = a_pipe[STAGES];
            else
                gcd_final = b_pipe[STAGES];

            result_d8 <= gcd_final << shift_pipe[STAGES];
            result_d9 <= result_d8;
            result_d10 <= result_d9;
            valid_d8 <= v_pipe[STAGES];
            valid_d9 <= valid_d8;
            valid_d10 <= valid_d9;
        end
    end

    assign result = {{WIDTH{1'b0}}, result_d10};
    assign valid_out = valid_d10;

endmodule

// ============================================================================
// SqrtUnit
// ============================================================================
// 8-bit integer sqrt using four root-bit decisions.
// Stage 0 = input capture.
// Stages 1..4 = try 8,4,2,1 root bits.
// Stages 5..10 = output alignment.
// Signed negative inputs are invalid and return zero.
// ============================================================================
module SqrtUnit #(
    parameter WIDTH = 8
)(
    input                    clk,
    input                    rst,
    input                    ce,
    input                    valid_in,
    input                    signed_mode,
    input  [WIDTH-1:0]       a,
    output [2*WIDTH-1:0]     result,
    output                   valid_out,
    output                   invalid
);

    localparam ROOT_W = WIDTH/2;

    reg [WIDTH-1:0] a0, a1, a2, a3;
    reg [ROOT_W-1:0] r1, r2, r3, r4;
    reg v0, v1, v2, v3, v4;
    reg inv0, inv1, inv2, inv3, inv4;

    // Alignment stages 5..10 (r4 is stage 4, so six more registers reach stage 10).
    reg [ROOT_W-1:0] root_d5, root_d6, root_d7, root_d8, root_d9, root_d10;
    reg valid_d5, valid_d6, valid_d7, valid_d8, valid_d9, valid_d10;
    reg inv_d5, inv_d6, inv_d7, inv_d8, inv_d9, inv_d10;

    wire [WIDTH-1:0] trial2;
    wire [WIDTH-1:0] trial3;
    wire [WIDTH-1:0] trial4;

    assign trial2 = {{ROOT_W{1'b0}}, r1} | 8'd4;
    assign trial3 = {{ROOT_W{1'b0}}, r2} | 8'd2;
    assign trial4 = {{ROOT_W{1'b0}}, r3} | 8'd1;

    integer unused_i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            a0 <= 0; a1 <= 0; a2 <= 0; a3 <= 0;
            r1 <= 0; r2 <= 0; r3 <= 0; r4 <= 0;
            v0 <= 0; v1 <= 0; v2 <= 0; v3 <= 0; v4 <= 0;
            inv0 <= 0; inv1 <= 0; inv2 <= 0; inv3 <= 0; inv4 <= 0;
            root_d5 <= 0; root_d6 <= 0; root_d7 <= 0;
            root_d8 <= 0; root_d9 <= 0; root_d10 <= 0;
            valid_d5 <= 0; valid_d6 <= 0; valid_d7 <= 0;
            valid_d8 <= 0; valid_d9 <= 0; valid_d10 <= 0;
            inv_d5 <= 0; inv_d6 <= 0; inv_d7 <= 0;
            inv_d8 <= 0; inv_d9 <= 0; inv_d10 <= 0;
        end
        else if (ce) begin
            // Stage 0
            a0 <= a;
            v0 <= valid_in;
            inv0 <= valid_in && signed_mode && a[WIDTH-1];

            // Stage 1: test root bit 3 (8)
            if ((8'd8 * 8'd8) <= a0)
                r1 <= 4'b1000;
            else
                r1 <= 4'b0000;
            a1 <= a0;
            v1 <= v0;
            inv1 <= inv0;

            // Stage 2: test root bit 2 (4)
            if ((trial2 * trial2) <= a1)
                r2 <= r1 | 4'b0100;
            else
                r2 <= r1;
            a2 <= a1;
            v2 <= v1;
            inv2 <= inv1;

            // Stage 3: test root bit 1 (2)
            if ((trial3 * trial3) <= a2)
                r3 <= r2 | 4'b0010;
            else
                r3 <= r2;
            a3 <= a2;
            v3 <= v2;
            inv3 <= inv2;

            // Stage 4: test root bit 0 (1)
            if ((trial4 * trial4) <= a3)
                r4 <= r3 | 4'b0001;
            else
                r4 <= r3;
            v4 <= v3;
            inv4 <= inv3;

            // Stages 5..10: fixed-latency alignment
            root_d5 <= r4;
            root_d6 <= root_d5;
            root_d7 <= root_d6;
            root_d8 <= root_d7;
            root_d9 <= root_d8;
            root_d10 <= root_d9;
            valid_d5 <= v4;
            valid_d6 <= valid_d5;
            valid_d7 <= valid_d6;
            valid_d8 <= valid_d7;
            valid_d9 <= valid_d8;
            valid_d10 <= valid_d9;
            inv_d5 <= inv4;
            inv_d6 <= inv_d5;
            inv_d7 <= inv_d6;
            inv_d8 <= inv_d7;
            inv_d9 <= inv_d8;
            inv_d10 <= inv_d9;
        end
    end

    assign result = inv_d10 ? {2*WIDTH{1'b0}} : {{(2*WIDTH-ROOT_W){1'b0}}, root_d10};
    assign valid_out = valid_d10;
    assign invalid = valid_d10 && inv_d10;

endmodule

// ============================================================================
// ReductionTree
// ============================================================================
// Takes already-pipelined lane contributions at transaction stage 7.
// Register level 1 (edge E8): capture four contributions (+ReLU).
// Register level 2 (edge E9): pairwise sums.
// Register level 3 (edge E10): final sum / MAC add -> output registers.
// So the results appear at N+10, aligned with valid_out.
// ============================================================================
module ReductionTree #(
    parameter WIDTH = 8,
    parameter RED_W = 2*WIDTH + 3
)(
    input                    clk,
    input                    rst,
    input                    ce,
    input                    valid_in,
    input                    signed_mode,
    input                    relu_en,
    input  [3:0]             opcode,
    input  [RED_W-1:0]       p0,
    input  [RED_W-1:0]       p1,
    input  [RED_W-1:0]       p2,
    input  [RED_W-1:0]       p3,
    input  [RED_W-1:0]       c,
    output reg [RED_W-1:0]   dot_result,
    output reg [RED_W-1:0]   mac_result,
    output reg [RED_W-1:0]   sad_result,
    output reg               valid_out,
    output reg [3:0]         opcode_out
);

    reg [RED_W-1:0] p0_s1, p1_s1, p2_s1, p3_s1, c_s1;
    reg [RED_W-1:0] sum0_s2, sum1_s2, c_s2;
    reg valid_s1, valid_s2;
    reg [3:0] op_s1, op_s2;

    integer j;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            p0_s1 <= 0; p1_s1 <= 0; p2_s1 <= 0; p3_s1 <= 0; c_s1 <= 0;
            sum0_s2 <= 0; sum1_s2 <= 0; c_s2 <= 0;
            valid_s1 <= 0; valid_s2 <= 0;
            op_s1 <= 0; op_s2 <= 0;
            dot_result <= 0; mac_result <= 0; sad_result <= 0;
            valid_out <= 0; opcode_out <= 0;
        end
        else if (ce) begin
            // Stage 1: capture contributions.
            // For signed DOT/MAC, negative product bits are already sign-extended.
            // ReLU applies to negative signed products before reduction.
            if (relu_en && signed_mode && (p0[RED_W-1] == 1'b1))
                p0_s1 <= 0;
            else
                p0_s1 <= p0;

            if (relu_en && signed_mode && (p1[RED_W-1] == 1'b1))
                p1_s1 <= 0;
            else
                p1_s1 <= p1;

            if (relu_en && signed_mode && (p2[RED_W-1] == 1'b1))
                p2_s1 <= 0;
            else
                p2_s1 <= p2;

            if (relu_en && signed_mode && (p3[RED_W-1] == 1'b1))
                p3_s1 <= 0;
            else
                p3_s1 <= p3;

            c_s1 <= c;
            valid_s1 <= valid_in;
            op_s1 <= opcode;

            // Stage 2: pairwise reduction. These are the final output registers.
            sum0_s2 <= p0_s1 + p1_s1;
            sum1_s2 <= p2_s1 + p3_s1;
            c_s2 <= c_s1;
            valid_s2 <= valid_s1;
            op_s2 <= op_s1;

            dot_result <= sum0_s2 + sum1_s2;
            mac_result <= sum0_s2 + sum1_s2 + c_s2;
            sad_result <= sum0_s2 + sum1_s2;
            valid_out <= valid_s2;
            opcode_out <= op_s2;
        end
    end

endmodule

// ============================================================================
// SimdLane
// ============================================================================
// Main lane datapath. The common lane transaction pipeline is 0..10, so all
// ordinary results are available at N+10. DIV/GCD/SQRT also expose N+10
// outputs, making selection at the same transaction slot safe.
// ============================================================================
module SimdLane #(
    parameter WIDTH = 8,
    parameter RED_W = 2*WIDTH + 3
)(
    input                    clk,
    input                    rst,
    input                    ce,
    input                    valid_in,
    input                    signed_mode,
    input                    relu_en,
    input  [3:0]             opcode,
    input  [WIDTH-1:0]       a,
    input  [WIDTH-1:0]       b,
    output [2*WIDTH-1:0]     result,
    output [2*WIDTH-1:0]     mult_result,
    output [2*WIDTH-1:0]     sad_result_lane,
    output [3:0]             opcode_out,
    output                   valid_out,
    output                   div_by_zero,
    output                   overflow,
    output                   underflow,
    output                   sqrt_invalid,
    output                   exception_out,
    output [2:0]             exception_code,
    output [2*WIDTH-1:0]     red_mult,
    output [2*WIDTH-1:0]     red_sad,
    output [3:0]             opcode_stage7,
    output                   valid_stage7,
    output                   signed_stage7,
    output                   relu_stage7
);

    // Data/control pipelines 0..10 => exactly 10 clocks from acceptance.
    reg [2*WIDTH-1:0] alu_pipe  [0:10];
    reg [2*WIDTH-1:0] mult_pipe [0:10];
    reg [2*WIDTH-1:0] sad_pipe  [0:10];
    reg [3:0]         op_pipe   [0:10];
    reg               v_pipe    [0:10];
    reg               ov_pipe   [0:10];
    reg               un_pipe   [0:10];
    reg               sm_pipe   [0:10];
    reg               relu_pipe [0:10];

    wire [2*WIDTH-1:0] div_res, gcd_res, sqrt_res;
    wire div_v, div_zero, gcd_v, sqrt_v, sqrt_inv;

    DivUnit #(.WIDTH(WIDTH)) u_div (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(valid_in && (opcode == `OP_DIV)),
        .signed_mode(signed_mode), .a(a), .b(b),
        .result(div_res), .valid_out(div_v), .div_by_zero(div_zero)
    );

    GcdUnit #(.WIDTH(WIDTH)) u_gcd (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(valid_in && (opcode == `OP_GCD)),
        .signed_mode(signed_mode), .a(a), .b(b),
        .result(gcd_res), .valid_out(gcd_v)
    );

    SqrtUnit #(.WIDTH(WIDTH)) u_sqrt (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(valid_in && (opcode == `OP_SQRT)),
        .signed_mode(signed_mode), .a(a),
        .result(sqrt_res), .valid_out(sqrt_v), .invalid(sqrt_inv)
    );

    reg [2*WIDTH-1:0] alu_comb;
    reg [2*WIDTH-1:0] mult_comb;
    reg [2*WIDTH-1:0] sad_comb;
    reg ov_comb, un_comb;

    reg signed [WIDTH:0] signed_a9;
    reg signed [WIDTH:0] signed_b9;
    reg signed [WIDTH:0] signed_tmp9;
    reg [WIDTH:0] unsigned_tmp9;
    reg signed [2*WIDTH-1:0] signed_product;
    reg [2*WIDTH-1:0] unsigned_product;
    reg signed [WIDTH:0] sad_diff_s;
    reg [WIDTH:0] sad_diff_u;
    reg [WIDTH:0] sad_mag_s;
    reg signed [WIDTH:0] lrelu_s;

    integer k;

    always @(*) begin
        alu_comb = 0;
        mult_comb = 0;
        sad_comb = 0;
        ov_comb = 0;
        un_comb = 0;

        signed_a9 = $signed({a[WIDTH-1], a});
        signed_b9 = $signed({b[WIDTH-1], b});
        signed_tmp9 = 0;
        unsigned_tmp9 = 0;
        signed_product = $signed({{WIDTH{a[WIDTH-1]}},a}) * $signed({{WIDTH{b[WIDTH-1]}},b});
        unsigned_product = {{WIDTH{1'b0}},a} * {{WIDTH{1'b0}},b};
        sad_diff_s = 0;
        sad_diff_u = 0;
        sad_mag_s = 0;
        lrelu_s = 0;

        mult_comb = signed_mode ? signed_product : unsigned_product;

        case (opcode)
            `OP_ADD: begin
                if (signed_mode) begin
                    signed_tmp9 = signed_a9 + signed_b9;
                    alu_comb = {{WIDTH{signed_tmp9[WIDTH]}}, signed_tmp9[WIDTH-1:0]};
                    if ((signed_tmp9 > 9'sd127) || (signed_tmp9 < -9'sd128))
                        ov_comb = 1'b1;
                end
                else begin
                    unsigned_tmp9 = {1'b0,a} + {1'b0,b};
                    alu_comb = {{WIDTH{1'b0}}, unsigned_tmp9[WIDTH-1:0]};
                    if (unsigned_tmp9 > 9'd255)
                        ov_comb = 1'b1;
                end
            end

            `OP_SUB: begin
                if (signed_mode) begin
                    signed_tmp9 = signed_a9 - signed_b9;
                    alu_comb = {{WIDTH{signed_tmp9[WIDTH]}}, signed_tmp9[WIDTH-1:0]};
                    if ((signed_tmp9 > 9'sd127) || (signed_tmp9 < -9'sd128))
                        ov_comb = 1'b1;
                end
                else begin
                    alu_comb = {{WIDTH{1'b0}}, (a - b)};
                    if (a < b)
                        un_comb = 1'b1;
                end
            end

            `OP_MULT: begin
                alu_comb = mult_comb;
                if (signed_mode) begin
                    if ((signed_product > 16'sd127) || (signed_product < -16'sd128))
                        ov_comb = 1'b1;
                end
                else begin
                    if (unsigned_product > 16'd255)
                        ov_comb = 1'b1;
                end
            end

            `OP_RELU: begin
                if (signed_mode)
                    alu_comb = a[WIDTH-1] ? 0 : {{WIDTH{1'b0}},a};
                else
                    alu_comb = {{WIDTH{1'b0}},a};
            end

            // Leaky ReLU: positive inputs pass through unchanged, negative
            // (signed) inputs are scaled by 1/8 (arithmetic shift right 3).
            // Unsigned inputs are never negative, so they pass through.
            `OP_LRELU: begin
                if (signed_mode && a[WIDTH-1]) begin
                    lrelu_s = signed_a9 >>> 3;
                    alu_comb = {{WIDTH{lrelu_s[WIDTH]}},lrelu_s[WIDTH-1:0]};
                end
                else begin
                    alu_comb = {{WIDTH{1'b0}},a};
                end
            end

            `OP_MIN: begin
                if (signed_mode) begin
                    if ($signed(a) < $signed(b))
                        alu_comb = {{WIDTH{a[WIDTH-1]}},a};
                    else
                        alu_comb = {{WIDTH{b[WIDTH-1]}},b};
                end
                else begin
                    if (a < b)
                        alu_comb = {{WIDTH{1'b0}},a};
                    else
                        alu_comb = {{WIDTH{1'b0}},b};
                end
            end

            `OP_MAX: begin
                if (signed_mode) begin
                    if ($signed(a) > $signed(b))
                        alu_comb = {{WIDTH{a[WIDTH-1]}},a};
                    else
                        alu_comb = {{WIDTH{b[WIDTH-1]}},b};
                end
                else begin
                    if (a > b)
                        alu_comb = {{WIDTH{1'b0}},a};
                    else
                        alu_comb = {{WIDTH{1'b0}},b};
                end
            end

            `OP_ABS: begin
                if (signed_mode) begin
                    if (a[WIDTH-1])
                        alu_comb = {{WIDTH{1'b0}}, (~a + 1'b1)};
                    else
                        alu_comb = {{WIDTH{1'b0}},a};
                end
                else begin
                    alu_comb = {{WIDTH{1'b0}},a};
                end
            end

            `OP_SATADD: begin
                if (signed_mode) begin
                    signed_tmp9 = signed_a9 + signed_b9;
                    if (signed_tmp9 > 9'sd127) begin
                        alu_comb = {{WIDTH{1'b0}},8'h7f};
                        ov_comb = 1'b1;
                    end
                    else if (signed_tmp9 < -9'sd128) begin
                        alu_comb = {{WIDTH{1'b1}},8'h80};
                        ov_comb = 1'b1;
                    end
                    else begin
                        alu_comb = {{WIDTH{signed_tmp9[WIDTH]}},signed_tmp9[WIDTH-1:0]};
                    end
                end
                else begin
                    unsigned_tmp9 = {1'b0,a} + {1'b0,b};
                    if (unsigned_tmp9 > 9'd255) begin
                        alu_comb = {{WIDTH{1'b0}},8'hff};
                        ov_comb = 1'b1;
                    end
                    else begin
                        alu_comb = {{WIDTH{1'b0}},unsigned_tmp9[WIDTH-1:0]};
                    end
                end
            end

            `OP_SQRT: begin
                alu_comb = 0;
            end

            `OP_GCD: begin
                alu_comb = 0;
            end

            `OP_POPCOUNT: begin
                alu_comb = 0;
                for (k = 0; k < WIDTH; k = k + 1)
                    alu_comb = alu_comb + a[k];
            end

            `OP_DOT,
            `OP_MAC: begin
                alu_comb = mult_comb;
            end

            `OP_SAD: begin
                if (signed_mode) begin
                    sad_diff_s = signed_a9 - signed_b9;
                    if (sad_diff_s[WIDTH])
                        sad_mag_s = (~sad_diff_s) + 1'b1;
                    else
                        sad_mag_s = sad_diff_s;
                    sad_comb = {{WIDTH{1'b0}},sad_mag_s[WIDTH-1:0]};
                end
                else begin
                    sad_diff_u = (a >= b) ? ({1'b0,a} - {1'b0,b}) : ({1'b0,b} - {1'b0,a});
                    sad_comb = {{WIDTH{1'b0}},sad_diff_u[WIDTH-1:0]};
                end
                alu_comb = sad_comb;
            end

            default: begin
                alu_comb = 0;
            end
        endcase
    end

    integer i;
    reg [2*WIDTH-1:0] selected_d10;
    reg               selected_ov_d10;
    reg               selected_un_d10;
    reg               selected_dz_d10;
    reg               selected_sqrt_inv_d10;

    always @(*) begin
        selected_d10 = alu_pipe[10];
        selected_ov_d10 = ov_pipe[10];
        selected_un_d10 = un_pipe[10];
        selected_dz_d10 = 1'b0;
        selected_sqrt_inv_d10 = 1'b0;

        case (op_pipe[10])
            `OP_MULT: begin
                selected_d10 = mult_pipe[10];
            end
            `OP_DIV: begin
                selected_d10 = div_res;
                selected_dz_d10 = div_zero;
            end
            `OP_GCD: begin
                selected_d10 = gcd_res;
            end
            `OP_SQRT: begin
                selected_d10 = sqrt_res;
                selected_sqrt_inv_d10 = sqrt_inv;
            end
            `OP_DOT,
            `OP_MAC: begin
                selected_d10 = mult_pipe[10];
            end
            `OP_SAD: begin
                selected_d10 = sad_pipe[10];
            end
            default: begin
                selected_d10 = alu_pipe[10];
            end
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i <= 10; i = i + 1) begin
                alu_pipe[i] <= 0;
                mult_pipe[i] <= 0;
                sad_pipe[i] <= 0;
                op_pipe[i] <= 0;
                v_pipe[i] <= 0;
                ov_pipe[i] <= 0;
                un_pipe[i] <= 0;
                sm_pipe[i] <= 0;
                relu_pipe[i] <= 0;
            end
        end
        else if (ce) begin
            alu_pipe[0] <= alu_comb;
            mult_pipe[0] <= mult_comb;
            sad_pipe[0] <= sad_comb;
            op_pipe[0] <= opcode;
            v_pipe[0] <= valid_in;
            ov_pipe[0] <= ov_comb;
            un_pipe[0] <= un_comb;
            sm_pipe[0] <= signed_mode;
            relu_pipe[0] <= relu_en;

            for (i = 1; i <= 10; i = i + 1) begin
                alu_pipe[i] <= alu_pipe[i-1];
                mult_pipe[i] <= mult_pipe[i-1];
                sad_pipe[i] <= sad_pipe[i-1];
                op_pipe[i] <= op_pipe[i-1];
                v_pipe[i] <= v_pipe[i-1];
                ov_pipe[i] <= ov_pipe[i-1];
                un_pipe[i] <= un_pipe[i-1];
                sm_pipe[i] <= sm_pipe[i-1];
                relu_pipe[i] <= relu_pipe[i-1];
            end
        end
    end

    // At the output edge, op_pipe[10] and all specialized-unit outputs are
    // from the same accepted transaction. The selection is combinational,
    // so no extra (11th) result register is inserted here.
    assign result = (op_pipe[10] == `OP_DIV)  ? div_res  :
                    (op_pipe[10] == `OP_GCD)  ? gcd_res  :
                    (op_pipe[10] == `OP_SQRT) ? sqrt_res :
                    alu_pipe[10];

    assign mult_result = mult_pipe[10];
    assign sad_result_lane = sad_pipe[10];
    assign opcode_out = op_pipe[10];
    assign valid_out = v_pipe[10];

    // The reduction tree consumes the stage-7 copies of the lane datapath.
    assign red_mult = mult_pipe[7];
    assign red_sad = sad_pipe[7];
    assign opcode_stage7 = op_pipe[7];
    assign valid_stage7 = v_pipe[7];
    assign signed_stage7 = sm_pipe[7];
    assign relu_stage7 = relu_pipe[7];

    assign div_by_zero = v_pipe[10] && (op_pipe[10] == `OP_DIV) && div_zero;
    assign overflow = v_pipe[10] && ov_pipe[10];
    assign underflow = v_pipe[10] && un_pipe[10];
    assign sqrt_invalid = v_pipe[10] && (op_pipe[10] == `OP_SQRT) && sqrt_inv;
    assign exception_out = v_pipe[10] &&
                           ((op_pipe[10] == `OP_DIV)  && div_zero ||
                            ov_pipe[10] ||
                            un_pipe[10] ||
                            ((op_pipe[10] == `OP_SQRT) && sqrt_inv));
    assign exception_code = !v_pipe[10] ? `EX_NONE :
                            ((op_pipe[10] == `OP_DIV) && div_zero) ? `EX_DIVZERO :
                            ov_pipe[10] ? `EX_OVERFLOW :
                            un_pipe[10] ? `EX_UNDER :
                            ((op_pipe[10] == `OP_SQRT) && sqrt_inv) ? `EX_INVALID :
                            `EX_NONE;

endmodule

// ============================================================================
// SimdTop
// ============================================================================
// Global transaction pipeline 0..10 gives the external 10-cycle contract.
// If ready_out is low while the final output is valid, the entire pipeline is
// held through 'ce'. This makes the ready/valid behavior unambiguous.
// ============================================================================
module SimdTop #(
    parameter WIDTH = 8,
    parameter RED_W = 2*WIDTH + 3
)(
    input                    clk,
    input                    rst,
    input                    valid_in,
    output                   ready_in,
    input                    ready_out,
    input                    signed_mode,
    input                    relu_en,
    input  [3:0]             opcode,
    input  [3:0]             mask,
    input  [WIDTH-1:0]       a0, a1, a2, a3,
    input  [WIDTH-1:0]       b0, b1, b2, b3,
    input  [2*WIDTH-1:0]     c,
    output [2*WIDTH-1:0]     lane0_result,
    output [2*WIDTH-1:0]     lane1_result,
    output [2*WIDTH-1:0]     lane2_result,
    output [2*WIDTH-1:0]     lane3_result,
    output [3:0]             lane_valid_out,
    output [RED_W-1:0]       dot_result,
    output [RED_W-1:0]       mac_result,
    output [RED_W-1:0]       sad_result,
    output                   reduction_valid_out,
    output                   valid_out,
    output [3:0]             opcode_out,
    output                   signed_mode_out,
    output [3:0]             div_by_zero,
    output [3:0]             overflow,
    output [3:0]             underflow,
    output [3:0]             sqrt_invalid,
    output [3:0]             exception_out,
    output [2:0]             exception_code0,
    output [2:0]             exception_code1,
    output [2:0]             exception_code2,
    output [2:0]             exception_code3
);

    reg txn_v [0:10];
    reg [3:0] txn_opcode [0:10];
    reg       txn_signed [0:10];
    reg       txn_relu   [0:10];
    reg [2*WIDTH-1:0] txn_c [0:10];

    wire mask_nonzero;
    wire pipeline_can_advance;
    wire accept;
    wire ce;

    assign mask_nonzero = |mask;
    assign pipeline_can_advance = !txn_v[10] || ready_out;
    assign ready_in = pipeline_can_advance && mask_nonzero;
    assign accept = valid_in && ready_in;
    assign ce = pipeline_can_advance;

    wire lane0_in_valid = accept && mask[0];
    wire lane1_in_valid = accept && mask[1];
    wire lane2_in_valid = accept && mask[2];
    wire lane3_in_valid = accept && mask[3];

    wire [2*WIDTH-1:0] lane0_mult, lane1_mult, lane2_mult, lane3_mult;
    wire [2*WIDTH-1:0] lane0_sad, lane1_sad, lane2_sad, lane3_sad;
    wire [2*WIDTH-1:0] lane0_red_mult, lane1_red_mult, lane2_red_mult, lane3_red_mult;
    wire [2*WIDTH-1:0] lane0_red_sad, lane1_red_sad, lane2_red_sad, lane3_red_sad;
    wire [3:0] lane0_op8, lane1_op8, lane2_op8, lane3_op8;
    wire lane0_v7, lane1_v7, lane2_v7, lane3_v7;
    wire lane0_sm7, lane1_sm7, lane2_sm7, lane3_sm7;
    wire lane0_relu7, lane1_relu7, lane2_relu7, lane3_relu7;

    wire lane0_dz, lane1_dz, lane2_dz, lane3_dz;
    wire lane0_ov, lane1_ov, lane2_ov, lane3_ov;
    wire lane0_un, lane1_un, lane2_un, lane3_un;
    wire lane0_sinv, lane1_sinv, lane2_sinv, lane3_sinv;
    wire lane0_exc, lane1_exc, lane2_exc, lane3_exc;

    SimdLane #(.WIDTH(WIDTH), .RED_W(RED_W)) lane0 (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(lane0_in_valid),
        .signed_mode(signed_mode), .relu_en(relu_en), .opcode(opcode),
        .a(a0), .b(b0),
        .result(lane0_result), .mult_result(lane0_mult),
        .sad_result_lane(lane0_sad), .opcode_out(lane0_op8),
        .valid_out(lane_valid_out[0]),
        .div_by_zero(lane0_dz), .overflow(lane0_ov), .underflow(lane0_un),
        .sqrt_invalid(lane0_sinv), .exception_out(lane0_exc), .exception_code(exception_code0),
        .red_mult(lane0_red_mult), .red_sad(lane0_red_sad),
        .opcode_stage7(), .valid_stage7(lane0_v7),
        .signed_stage7(lane0_sm7), .relu_stage7(lane0_relu7)
    );

    SimdLane #(.WIDTH(WIDTH), .RED_W(RED_W)) lane1 (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(lane1_in_valid),
        .signed_mode(signed_mode), .relu_en(relu_en), .opcode(opcode),
        .a(a1), .b(b1),
        .result(lane1_result), .mult_result(lane1_mult),
        .sad_result_lane(lane1_sad), .opcode_out(lane1_op8),
        .valid_out(lane_valid_out[1]),
        .div_by_zero(lane1_dz), .overflow(lane1_ov), .underflow(lane1_un),
        .sqrt_invalid(lane1_sinv), .exception_out(lane1_exc), .exception_code(exception_code1),
        .red_mult(lane1_red_mult), .red_sad(lane1_red_sad),
        .opcode_stage7(), .valid_stage7(lane1_v7),
        .signed_stage7(lane1_sm7), .relu_stage7(lane1_relu7)
    );

    SimdLane #(.WIDTH(WIDTH), .RED_W(RED_W)) lane2 (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(lane2_in_valid),
        .signed_mode(signed_mode), .relu_en(relu_en), .opcode(opcode),
        .a(a2), .b(b2),
        .result(lane2_result), .mult_result(lane2_mult),
        .sad_result_lane(lane2_sad), .opcode_out(lane2_op8),
        .valid_out(lane_valid_out[2]),
        .div_by_zero(lane2_dz), .overflow(lane2_ov), .underflow(lane2_un),
        .sqrt_invalid(lane2_sinv), .exception_out(lane2_exc), .exception_code(exception_code2),
        .red_mult(lane2_red_mult), .red_sad(lane2_red_sad),
        .opcode_stage7(), .valid_stage7(lane2_v7),
        .signed_stage7(lane2_sm7), .relu_stage7(lane2_relu7)
    );

    SimdLane #(.WIDTH(WIDTH), .RED_W(RED_W)) lane3 (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(lane3_in_valid),
        .signed_mode(signed_mode), .relu_en(relu_en), .opcode(opcode),
        .a(a3), .b(b3),
        .result(lane3_result), .mult_result(lane3_mult),
        .sad_result_lane(lane3_sad), .opcode_out(lane3_op8),
        .valid_out(lane_valid_out[3]),
        .div_by_zero(lane3_dz), .overflow(lane3_ov), .underflow(lane3_un),
        .sqrt_invalid(lane3_sinv), .exception_out(lane3_exc), .exception_code(exception_code3),
        .red_mult(lane3_red_mult), .red_sad(lane3_red_sad),
        .opcode_stage7(), .valid_stage7(lane3_v7),
        .signed_stage7(lane3_sm7), .relu_stage7(lane3_relu7)
    );

    integer t;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (t = 0; t <= 10; t = t + 1) begin
                txn_v[t] <= 0;
                txn_opcode[t] <= 0;
                txn_signed[t] <= 0;
                txn_relu[t] <= 0;
                txn_c[t] <= 0;
            end
        end
        else if (ce) begin
            txn_v[0] <= accept;
            txn_opcode[0] <= opcode;
            txn_signed[0] <= signed_mode;
            txn_relu[0] <= relu_en;
            txn_c[0] <= c;

            for (t = 1; t <= 10; t = t + 1) begin
                txn_v[t] <= txn_v[t-1];
                txn_opcode[t] <= txn_opcode[t-1];
                txn_signed[t] <= txn_signed[t-1];
                txn_relu[t] <= txn_relu[t-1];
                txn_c[t] <= txn_c[t-1];
            end
        end
    end

    // Reduction input at stage 7. Inactive lanes are forced to zero.
    reg [RED_W-1:0] red_p0, red_p1, red_p2, red_p3, red_c;
    reg red_signed, red_relu;
    reg red_active;

    always @(*) begin
        red_active = txn_v[7] &&
                     ((txn_opcode[7] == `OP_DOT) ||
                      (txn_opcode[7] == `OP_MAC) ||
                      (txn_opcode[7] == `OP_SAD));
        red_signed = txn_signed[7];
        red_relu = txn_relu[7];
        red_c = 0;
        red_p0 = 0;
        red_p1 = 0;
        red_p2 = 0;
        red_p3 = 0;

        if (red_active) begin
            red_c = red_signed ?
                    {{(RED_W-2*WIDTH){txn_c[7][2*WIDTH-1]}},txn_c[7]} :
                    {{(RED_W-2*WIDTH){1'b0}},txn_c[7]};

            if (txn_opcode[7] == `OP_SAD) begin
                red_p0 = lane0_v7 ? {{(RED_W-2*WIDTH){1'b0}},lane0_red_sad} : 0;
                red_p1 = lane1_v7 ? {{(RED_W-2*WIDTH){1'b0}},lane1_red_sad} : 0;
                red_p2 = lane2_v7 ? {{(RED_W-2*WIDTH){1'b0}},lane2_red_sad} : 0;
                red_p3 = lane3_v7 ? {{(RED_W-2*WIDTH){1'b0}},lane3_red_sad} : 0;
            end
            else begin
                red_p0 = lane0_v7 ?
                    (red_signed ? {{(RED_W-2*WIDTH){lane0_red_mult[2*WIDTH-1]}},lane0_red_mult} :
                                 {{(RED_W-2*WIDTH){1'b0}},lane0_red_mult}) : 0;
                red_p1 = lane1_v7 ?
                    (red_signed ? {{(RED_W-2*WIDTH){lane1_red_mult[2*WIDTH-1]}},lane1_red_mult} :
                                 {{(RED_W-2*WIDTH){1'b0}},lane1_red_mult}) : 0;
                red_p2 = lane2_v7 ?
                    (red_signed ? {{(RED_W-2*WIDTH){lane2_red_mult[2*WIDTH-1]}},lane2_red_mult} :
                                 {{(RED_W-2*WIDTH){1'b0}},lane2_red_mult}) : 0;
                red_p3 = lane3_v7 ?
                    (red_signed ? {{(RED_W-2*WIDTH){lane3_red_mult[2*WIDTH-1]}},lane3_red_mult} :
                                 {{(RED_W-2*WIDTH){1'b0}},lane3_red_mult}) : 0;
            end
        end
    end

    ReductionTree #(.WIDTH(WIDTH), .RED_W(RED_W)) reduction (
        .clk(clk), .rst(rst), .ce(ce),
        .valid_in(red_active), .signed_mode(red_signed), .relu_en(red_relu),
        .opcode(txn_opcode[7]),
        .p0(red_p0), .p1(red_p1), .p2(red_p2), .p3(red_p3), .c(red_c),
        .dot_result(dot_result), .mac_result(mac_result), .sad_result(sad_result),
        .valid_out(reduction_valid_out), .opcode_out()
    );

    assign valid_out = txn_v[10];
    assign opcode_out = txn_opcode[10];
    assign signed_mode_out = txn_signed[10];

    assign div_by_zero = {lane3_dz,lane2_dz,lane1_dz,lane0_dz};
    assign overflow = {lane3_ov,lane2_ov,lane1_ov,lane0_ov};
    assign underflow = {lane3_un,lane2_un,lane1_un,lane0_un};
    assign sqrt_invalid = {lane3_sinv,lane2_sinv,lane1_sinv,lane0_sinv};
    assign exception_out = {lane3_exc,lane2_exc,lane1_exc,lane0_exc};

endmodule

`endif
