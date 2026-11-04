`timescale 1ns/1ps

// ============================================================================
// Full self-checking testbench for SimdTop (4-lane SIMD processor)
// Verilog-2001, black-box (no hierarchical references into the DUT).
//
// Compile order: RTL first (this file uses its K_* / X_* macros), e.g.
//   iverilog -g2005 -o sim simd_processor.v tb_simd_full.v && vvp sim
//   iverilog -g2005 -DDUMP ...          -> also writes tb_simd_full.vcd
//
// What is checked on every completed transaction
//   * LATENCY  : exactly 10 clocks from acceptance to valid_out, not counting
//                clocks in which the whole pipeline is frozen by ready_out=0.
//   * DATA     : every active lane result vs. an independent reference model.
//   * MASK     : lane_valid_out == accepted mask.
//   * EXCEPT.  : exception_code / div_by_zero / overflow / underflow /
//                sqrt_invalid / exception_out per active lane; masked-off
//                lanes must stay quiet.
//   * REDUCE   : dot/mac/sad result and reduction_valid_out (must be low for
//                non-reduction opcodes).
//   * HOLD     : while valid_out && !ready_out every output bus is frozen and
//                ready_in is low (backpressure).
//   * RESET    : outputs idle during reset; in-flight work is flushed.
//
// Reference model matches the RTL's documented behaviour:
//   * unsigned ADD/SUB return the low 8 bits (+ overflow / underflow flag);
//     signed ADD/SUB return the full 9-bit result sign-extended.
//   * MULT/DOT/MAC return the full 16-bit product; MULT overflow flag is
//     raised when the product does not fit in 8 bits.
//   If you change those rules in the RTL, edit expected_lane / expected_exc.
// ============================================================================

module tb_simd_full;

    parameter WIDTH      = 8;
    parameter RED_W      = 2*WIDTH + 3;
    parameter QDEPTH     = 64;
    parameter LATENCY    = 10;
    parameter EXHAUSTIVE = 0;        // 1: every (a,b) pair for every 2-operand opcode, signed+unsigned (~490k cycles, slow)
    parameter N_RANDOM   = 20000;    // transactions per random phase
    parameter MAX_PRINT  = 40;       // stop printing after this many errors
    parameter MAX_CYCLES = 20000000; // watchdog

    // Opcode / exception-code constants (local copies of the RTL macros, so this
    // testbench compiles standalone and does not depend on compile order).
    localparam [3:0] K_ADD      = 4'b0000;
    localparam [3:0] K_SUB      = 4'b0001;
    localparam [3:0] K_MULT     = 4'b0010;
    localparam [3:0] K_DIV      = 4'b0011;
    localparam [3:0] K_MAC      = 4'b0100;
    localparam [3:0] K_DOT      = 4'b0101;
    localparam [3:0] K_RELU     = 4'b0110;
    localparam [3:0] K_LRELU    = 4'b0111;
    localparam [3:0] K_MIN      = 4'b1000;
    localparam [3:0] K_MAX      = 4'b1001;
    localparam [3:0] K_ABS      = 4'b1010;
    localparam [3:0] K_SATADD   = 4'b1011;
    localparam [3:0] K_SQRT     = 4'b1100;
    localparam [3:0] K_GCD      = 4'b1101;
    localparam [3:0] K_SAD      = 4'b1110;
    localparam [3:0] K_POPCOUNT = 4'b1111;
    localparam [2:0] X_NONE     = 3'b000;
    localparam [2:0] X_DIVZERO  = 3'b001;
    localparam [2:0] X_OVERFLOW = 3'b010;
    localparam [2:0] X_UNDER    = 3'b011;
    localparam [2:0] X_INVALID  = 3'b100;

    // ------------------------------------------------------------------
    // DUT signals
    // ------------------------------------------------------------------
    reg clk, rst;
    reg valid_in;
    wire ready_in;
    reg ready_out;
    reg signed_mode;
    reg relu_en;
    reg [3:0] opcode;
    reg [3:0] mask;
    reg [WIDTH-1:0] a0, a1, a2, a3;
    reg [WIDTH-1:0] b0, b1, b2, b3;
    reg [2*WIDTH-1:0] c;

    wire [2*WIDTH-1:0] lane0_result, lane1_result, lane2_result, lane3_result;
    wire [3:0] lane_valid_out;
    wire [RED_W-1:0] dot_result, mac_result, sad_result;
    wire reduction_valid_out, valid_out;
    wire [3:0] opcode_out;
    wire signed_mode_out;
    wire [3:0] div_by_zero, overflow, underflow, sqrt_invalid, exception_out;
    wire [2:0] exception_code0, exception_code1, exception_code2, exception_code3;

    SimdTop #(.WIDTH(WIDTH), .RED_W(RED_W)) dut (
        .clk(clk), .rst(rst),
        .valid_in(valid_in), .ready_in(ready_in), .ready_out(ready_out),
        .signed_mode(signed_mode), .relu_en(relu_en),
        .opcode(opcode), .mask(mask),
        .a0(a0), .a1(a1), .a2(a2), .a3(a3),
        .b0(b0), .b1(b1), .b2(b2), .b3(b3), .c(c),
        .lane0_result(lane0_result), .lane1_result(lane1_result),
        .lane2_result(lane2_result), .lane3_result(lane3_result),
        .lane_valid_out(lane_valid_out),
        .dot_result(dot_result), .mac_result(mac_result), .sad_result(sad_result),
        .reduction_valid_out(reduction_valid_out), .valid_out(valid_out),
        .opcode_out(opcode_out), .signed_mode_out(signed_mode_out),
        .div_by_zero(div_by_zero), .overflow(overflow), .underflow(underflow),
        .sqrt_invalid(sqrt_invalid), .exception_out(exception_out),
        .exception_code0(exception_code0), .exception_code1(exception_code1),
        .exception_code2(exception_code2), .exception_code3(exception_code3)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

`ifdef DUMP
    initial begin
        $dumpfile("tb_simd_full.vcd");
        $dumpvars(0, tb_simd_full);
    end
`endif

    // ------------------------------------------------------------------
    // Bookkeeping
    // ------------------------------------------------------------------
    integer seed;
    integer seed_ro;
    integer cycle_count;
    integer stall_total;
    integer accepted_count, completed_count, flushed_count;
    integer accepted_lanes;
    integer inflight_max;
    integer rst_edges;
    integer err_total;
    integer e_lat, e_data, e_exc, e_mask, e_red, e_hold, e_misc;
    integer cov_op [0:15];
    integer cov_sm [0:1];
    integer cov_exc [0:4];
    integer i_init;
    reg [8*24-1:0] phase;
    reg [1:0] stall_mode;
    reg last_accept;

    // scoreboard queue
    reg [3:0]  q_op   [0:QDEPTH-1];
    reg        q_sm   [0:QDEPTH-1];
    reg [3:0]  q_mask [0:QDEPTH-1];
    reg [63:0] q_in   [0:QDEPTH-1];   // {b3,b2,b1,b0,a3,a2,a1,a0}
    reg [15:0] q_e0   [0:QDEPTH-1];
    reg [15:0] q_e1   [0:QDEPTH-1];
    reg [15:0] q_e2   [0:QDEPTH-1];
    reg [15:0] q_e3   [0:QDEPTH-1];
    reg [2:0]  q_x0   [0:QDEPTH-1];
    reg [2:0]  q_x1   [0:QDEPTH-1];
    reg [2:0]  q_x2   [0:QDEPTH-1];
    reg [2:0]  q_x3   [0:QDEPTH-1];
    reg [RED_W-1:0] q_dot [0:QDEPTH-1];
    reg [RED_W-1:0] q_mac [0:QDEPTH-1];
    reg [RED_W-1:0] q_sad [0:QDEPTH-1];
    integer q_cyc   [0:QDEPTH-1];
    integer q_stall [0:QDEPTH-1];
    integer q_head, q_tail, q_count;

    // whole-output snapshot used for the backpressure hold check
    wire [199:0] out_bus = {lane3_result, lane2_result, lane1_result, lane0_result,
                            lane_valid_out, dot_result, mac_result, sad_result,
                            reduction_valid_out, valid_out, opcode_out, signed_mode_out,
                            div_by_zero, overflow, underflow, sqrt_invalid, exception_out,
                            exception_code3, exception_code2, exception_code1, exception_code0};
    reg [199:0] hold_snap;
    reg         hold_pending;

    // ------------------------------------------------------------------
    // Reference model
    // ------------------------------------------------------------------
    function integer s8;
        input [7:0] x;
        begin s8 = {{24{x[7]}}, x}; end
    endfunction

    function integer s16;
        input [15:0] x;
        begin s16 = {{16{x[15]}}, x}; end
    endfunction

    function integer op_val;      // operand as integer for the given mode
        input sm;
        input [7:0] x;
        begin
            if (sm) op_val = s8(x);
            else    op_val = x;
        end
    endfunction

    function integer gcd_int;
        input integer x_in;
        input integer y_in;
        integer x, y, t;
        begin
            x = x_in; if (x < 0) x = -x;
            y = y_in; if (y < 0) y = -y;
            while (y != 0) begin
                t = x % y;
                x = y;
                y = t;
            end
            gcd_int = x;
        end
    endfunction

    function integer isqrt;
        input integer x;
        integer r;
        begin
            r = 0;
            while (((r+1)*(r+1)) <= x) r = r + 1;
            isqrt = r;
        end
    endfunction

    function [15:0] expected_lane;
        input [3:0] op;
        input sm;
        input [7:0] aa;
        input [7:0] bb;
        integer av, bv, res, k;
        begin
            av = op_val(sm, aa);
            bv = op_val(sm, bb);
            res = 0;
            case (op)
                K_ADD: begin
                    res = av + bv;
                    if (!sm) res = res & 255;
                end
                K_SUB: begin
                    res = av - bv;
                    if (!sm) res = res & 255;
                end
                K_MULT, K_DOT, K_MAC: res = av * bv;
                K_DIV: begin
                    if (bb == 0) res = 0;
                    else         res = av / bv;
                end
                K_RELU: begin
                    if (sm && av < 0) res = 0;
                    else              res = av;
                end
                K_LRELU: begin
                    if (sm && av < 0) res = av >>> 3;
                    else              res = av;
                end
                K_MIN: begin
                    if (av < bv) res = av; else res = bv;
                end
                K_MAX: begin
                    if (av > bv) res = av; else res = bv;
                end
                K_ABS: begin
                    if (av < 0) res = -av; else res = av;
                end
                K_SATADD: begin
                    res = av + bv;
                    if (sm) begin
                        if (res > 127)  res = 127;
                        if (res < -128) res = -128;
                    end
                    else begin
                        if (res > 255) res = 255;
                    end
                end
                K_SQRT: begin
                    if (sm && av < 0) res = 0;
                    else              res = isqrt(av);
                end
                K_GCD: res = gcd_int(av, bv);
                K_POPCOUNT: begin
                    for (k = 0; k < 8; k = k + 1)
                        if (aa[k]) res = res + 1;
                end
                K_SAD: begin
                    res = av - bv;
                    if (res < 0) res = -res;
                end
                default: res = 0;
            endcase
            expected_lane = res;
        end
    endfunction

    function [2:0] expected_exc;
        input [3:0] op;
        input sm;
        input [7:0] aa;
        input [7:0] bb;
        integer av, bv, t;
        begin
            av = op_val(sm, aa);
            bv = op_val(sm, bb);
            expected_exc = X_NONE;
            case (op)
                K_DIV: begin
                    if (bb == 0) expected_exc = X_DIVZERO;
                end
                K_ADD, K_SATADD: begin
                    t = av + bv;
                    if (sm) begin
                        if (t > 127 || t < -128) expected_exc = X_OVERFLOW;
                    end
                    else begin
                        if (t > 255) expected_exc = X_OVERFLOW;
                    end
                end
                K_SUB: begin
                    t = av - bv;
                    if (sm) begin
                        if (t > 127 || t < -128) expected_exc = X_OVERFLOW;
                    end
                    else begin
                        if (aa < bb) expected_exc = X_UNDER;
                    end
                end
                K_MULT: begin
                    t = av * bv;
                    if (sm) begin
                        if (t > 127 || t < -128) expected_exc = X_OVERFLOW;
                    end
                    else begin
                        if (t > 255) expected_exc = X_OVERFLOW;
                    end
                end
                K_SQRT: begin
                    if (sm && av < 0) expected_exc = X_INVALID;
                end
                default: ;
            endcase
        end
    endfunction

    function integer lane_prod;   // DOT/MAC contribution (ReLU zeroes negative signed products)
        input sm;
        input relu;
        input [7:0] aa;
        input [7:0] bb;
        integer p;
        begin
            p = op_val(sm, aa) * op_val(sm, bb);
            if (relu && sm && p < 0) p = 0;
            lane_prod = p;
        end
    endfunction

    function integer lane_absdiff; // SAD contribution
        input sm;
        input [7:0] aa;
        input [7:0] bb;
        integer d;
        begin
            d = op_val(sm, aa) - op_val(sm, bb);
            if (d < 0) d = -d;
            lane_absdiff = d;
        end
    endfunction

    task push_expected;
        input [3:0] op;
        input sm;
        input relu;
        input [3:0] m;
        input [7:0] aa0, aa1, aa2, aa3;
        input [7:0] bb0, bb1, bb2, bb3;
        input [15:0] cc;
        integer dot_i, sad_i, cext;
        begin
            q_e0[q_tail] = expected_lane(op, sm, aa0, bb0);
            q_e1[q_tail] = expected_lane(op, sm, aa1, bb1);
            q_e2[q_tail] = expected_lane(op, sm, aa2, bb2);
            q_e3[q_tail] = expected_lane(op, sm, aa3, bb3);
            q_x0[q_tail] = expected_exc(op, sm, aa0, bb0);
            q_x1[q_tail] = expected_exc(op, sm, aa1, bb1);
            q_x2[q_tail] = expected_exc(op, sm, aa2, bb2);
            q_x3[q_tail] = expected_exc(op, sm, aa3, bb3);

            dot_i = 0;
            sad_i = 0;
            if (m[0]) begin dot_i = dot_i + lane_prod(sm, relu, aa0, bb0); sad_i = sad_i + lane_absdiff(sm, aa0, bb0); end
            if (m[1]) begin dot_i = dot_i + lane_prod(sm, relu, aa1, bb1); sad_i = sad_i + lane_absdiff(sm, aa1, bb1); end
            if (m[2]) begin dot_i = dot_i + lane_prod(sm, relu, aa2, bb2); sad_i = sad_i + lane_absdiff(sm, aa2, bb2); end
            if (m[3]) begin dot_i = dot_i + lane_prod(sm, relu, aa3, bb3); sad_i = sad_i + lane_absdiff(sm, aa3, bb3); end
            if (sm) cext = s16(cc);
            else    cext = cc;

            q_dot[q_tail] = dot_i;
            q_mac[q_tail] = dot_i + cext;
            q_sad[q_tail] = sad_i;

            q_op[q_tail]   = op;
            q_sm[q_tail]   = sm;
            q_mask[q_tail] = m;
            q_in[q_tail]   = {bb3, bb2, bb1, bb0, aa3, aa2, aa1, aa0};
            q_cyc[q_tail]   = cycle_count;
            q_stall[q_tail] = stall_total;
        end
    endtask

    // ------------------------------------------------------------------
    // Per-lane output checker
    // ------------------------------------------------------------------
    task check_lane;
        input integer li;
        input act;
        input [15:0] got;
        input [15:0] exp;
        input [2:0] gcode;
        input [2:0] xcode;
        input gdz;
        input gov;
        input gun;
        input gsi;
        input gex;
        input [3:0] op;
        input sm;
        input [7:0] la;
        input [7:0] lb;
        begin
            if (act) begin
                if (got !== exp) begin
                    if (err_total < MAX_PRINT)
                        $display("[%0s] DATA lane%0d op=%b sm=%b a=0x%h b=0x%h exp=0x%h got=0x%h",
                                 phase, li, op, sm, la, lb, exp, got);
                    err_total = err_total + 1; e_data = e_data + 1;
                end
                if (gcode !== xcode) begin
                    if (err_total < MAX_PRINT)
                        $display("[%0s] EXC   lane%0d op=%b sm=%b a=0x%h b=0x%h exp_code=%0d got_code=%0d",
                                 phase, li, op, sm, la, lb, xcode, gcode);
                    err_total = err_total + 1; e_exc = e_exc + 1;
                end
                if ((gdz !== (xcode == X_DIVZERO))  || (gov !== (xcode == X_OVERFLOW)) ||
                    (gun !== (xcode == X_UNDER))    || (gsi !== (xcode == X_INVALID))  ||
                    (gex !== (xcode != X_NONE))) begin
                    if (err_total < MAX_PRINT)
                        $display("[%0s] FLAGS lane%0d op=%b sm=%b a=0x%h b=0x%h exp_code=%0d dz/ov/un/inv/exc=%b%b%b%b%b",
                                 phase, li, op, sm, la, lb, xcode, gdz, gov, gun, gsi, gex);
                    err_total = err_total + 1; e_exc = e_exc + 1;
                end
            end
            else begin
                if ((gdz !== 1'b0) || (gov !== 1'b0) || (gun !== 1'b0) || (gsi !== 1'b0) ||
                    (gex !== 1'b0) || (gcode !== X_NONE)) begin
                    if (err_total < MAX_PRINT)
                        $display("[%0s] MASKED lane%0d raised exception op=%b sm=%b a=0x%h b=0x%h code=%0d dz/ov/un/inv/exc=%b%b%b%b%b",
                                 phase, li, op, sm, la, lb, gcode, gdz, gov, gun, gsi, gex);
                    err_total = err_total + 1; e_exc = e_exc + 1;
                end
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Monitor: samples every signal just BEFORE the clock edge (the values
    // the DUT itself sees), so accept/pop follow the ready/valid handshake.
    // ------------------------------------------------------------------
    always @(posedge clk) begin : monitor
        integer h, lat;
        reg [3:0] hm;
        reg [63:0] hin;

        cycle_count = cycle_count + 1;
        last_accept = 1'b0;

        if (cycle_count > MAX_CYCLES) begin
            $display("WATCHDOG: exceeded %0d cycles, aborting", MAX_CYCLES);
            $finish;
        end

        if (rst) begin
            rst_edges = rst_edges + 1;
            if (rst_edges > 1 &&
                (valid_out !== 1'b0 || reduction_valid_out !== 1'b0 || lane_valid_out !== 4'b0000)) begin
                if (err_total < MAX_PRINT) $display("[%0s] RESET: outputs not idle during reset", phase);
                err_total = err_total + 1; e_misc = e_misc + 1;
            end
        end
        else begin
            // ---- backpressure hold check
            if (hold_pending) begin
                if (out_bus !== hold_snap) begin
                    if (err_total < MAX_PRINT) $display("[%0s] HOLD: outputs changed while stalled (cycle %0d)", phase, cycle_count);
                    err_total = err_total + 1; e_hold = e_hold + 1;
                end
                hold_pending = 1'b0;
            end
            if (valid_out === 1'b1 && ready_out === 1'b0 && ready_in !== 1'b0) begin
                if (err_total < MAX_PRINT) $display("[%0s] HOLD: ready_in high while output stalled (cycle %0d)", phase, cycle_count);
                err_total = err_total + 1; e_hold = e_hold + 1;
            end

            // ---- pop (before push so a same-edge push is never popped)
            if (valid_out === 1'b1 && ready_out === 1'b1) begin
                if (q_count == 0) begin
                    if (err_total < MAX_PRINT) $display("[%0s] MISC: valid_out with empty scoreboard (cycle %0d)", phase, cycle_count);
                    err_total = err_total + 1; e_misc = e_misc + 1;
                end
                else begin
                    h   = q_head;
                    hm  = q_mask[h];
                    hin = q_in[h];

                    lat = (cycle_count - q_cyc[h] - 1) - (stall_total - q_stall[h]);
                    if (lat !== LATENCY) begin
                        if (err_total < MAX_PRINT)
                            $display("[%0s] LATENCY op=%b expected=%0d actual=%0d", phase, q_op[h], LATENCY, lat);
                        err_total = err_total + 1; e_lat = e_lat + 1;
                    end
                    if (opcode_out !== q_op[h] || signed_mode_out !== q_sm[h]) begin
                        if (err_total < MAX_PRINT)
                            $display("[%0s] MISC: opcode/mode exp=%b/%b got=%b/%b", phase, q_op[h], q_sm[h], opcode_out, signed_mode_out);
                        err_total = err_total + 1; e_misc = e_misc + 1;
                    end
                    if (lane_valid_out !== hm) begin
                        if (err_total < MAX_PRINT)
                            $display("[%0s] MASK op=%b exp=%b got=%b", phase, q_op[h], hm, lane_valid_out);
                        err_total = err_total + 1; e_mask = e_mask + 1;
                    end

                    check_lane(0, hm[0], lane0_result, q_e0[h], exception_code0, q_x0[h],
                               div_by_zero[0], overflow[0], underflow[0], sqrt_invalid[0], exception_out[0],
                               q_op[h], q_sm[h], hin[7:0],   hin[39:32]);
                    check_lane(1, hm[1], lane1_result, q_e1[h], exception_code1, q_x1[h],
                               div_by_zero[1], overflow[1], underflow[1], sqrt_invalid[1], exception_out[1],
                               q_op[h], q_sm[h], hin[15:8],  hin[47:40]);
                    check_lane(2, hm[2], lane2_result, q_e2[h], exception_code2, q_x2[h],
                               div_by_zero[2], overflow[2], underflow[2], sqrt_invalid[2], exception_out[2],
                               q_op[h], q_sm[h], hin[23:16], hin[55:48]);
                    check_lane(3, hm[3], lane3_result, q_e3[h], exception_code3, q_x3[h],
                               div_by_zero[3], overflow[3], underflow[3], sqrt_invalid[3], exception_out[3],
                               q_op[h], q_sm[h], hin[31:24], hin[63:56]);

                    case (q_op[h])
                        K_DOT: begin
                            if (reduction_valid_out !== 1'b1 || dot_result !== q_dot[h]) begin
                                if (err_total < MAX_PRINT)
                                    $display("[%0s] REDUCE DOT sm=%b exp=0x%h got=0x%h rvalid=%b", phase, q_sm[h], q_dot[h], dot_result, reduction_valid_out);
                                err_total = err_total + 1; e_red = e_red + 1;
                            end
                        end
                        K_MAC: begin
                            if (reduction_valid_out !== 1'b1 || mac_result !== q_mac[h]) begin
                                if (err_total < MAX_PRINT)
                                    $display("[%0s] REDUCE MAC sm=%b exp=0x%h got=0x%h rvalid=%b", phase, q_sm[h], q_mac[h], mac_result, reduction_valid_out);
                                err_total = err_total + 1; e_red = e_red + 1;
                            end
                        end
                        K_SAD: begin
                            if (reduction_valid_out !== 1'b1 || sad_result !== q_sad[h]) begin
                                if (err_total < MAX_PRINT)
                                    $display("[%0s] REDUCE SAD sm=%b exp=0x%h got=0x%h rvalid=%b", phase, q_sm[h], q_sad[h], sad_result, reduction_valid_out);
                                err_total = err_total + 1; e_red = e_red + 1;
                            end
                        end
                        default: begin
                            if (reduction_valid_out !== 1'b0) begin
                                if (err_total < MAX_PRINT)
                                    $display("[%0s] REDUCE reduction_valid_out high for op=%b", phase, q_op[h]);
                                err_total = err_total + 1; e_red = e_red + 1;
                            end
                        end
                    endcase

                    // exception coverage
                    if (hm[0]) cov_exc[q_x0[h]] = cov_exc[q_x0[h]] + 1;
                    if (hm[1]) cov_exc[q_x1[h]] = cov_exc[q_x1[h]] + 1;
                    if (hm[2]) cov_exc[q_x2[h]] = cov_exc[q_x2[h]] + 1;
                    if (hm[3]) cov_exc[q_x3[h]] = cov_exc[q_x3[h]] + 1;

                    completed_count = completed_count + 1;
                    q_head  = (q_head + 1) % QDEPTH;
                    q_count = q_count - 1;
                end
            end

            // ---- push
            if (valid_in === 1'b1 && ready_in === 1'b1) begin
                if (q_count >= QDEPTH-1) begin
                    if (err_total < MAX_PRINT) $display("[%0s] MISC: scoreboard overflow", phase);
                    err_total = err_total + 1; e_misc = e_misc + 1;
                end
                else begin
                    push_expected(opcode, signed_mode, relu_en, mask, a0, a1, a2, a3, b0, b1, b2, b3, c);
                    q_tail = (q_tail + 1) % QDEPTH;
                    q_count = q_count + 1;
                    if (q_count > inflight_max) inflight_max = q_count;
                    accepted_count = accepted_count + 1;
                    accepted_lanes = accepted_lanes + mask[0] + mask[1] + mask[2] + mask[3];
                    cov_op[opcode] = cov_op[opcode] + 1;
                    cov_sm[signed_mode] = cov_sm[signed_mode] + 1;
                    last_accept = 1'b1;
                end
            end

            // ---- freeze accounting (the whole pipeline holds on these edges)
            if (valid_out === 1'b1 && ready_out === 1'b0) begin
                stall_total = stall_total + 1;
                hold_snap = out_bus;
                hold_pending = 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // ready_out generator (backpressure)
    //   0: always ready  1: ready 75%  2: ready 25%  3: random bursts
    // ------------------------------------------------------------------
    integer burst_cnt;
    always @(negedge clk) begin
        case (stall_mode)
            2'd0: ready_out = 1'b1;
            2'd1: ready_out = ((($random(seed_ro)) & 3) != 0);
            2'd2: ready_out = ((($random(seed_ro)) & 3) == 0);
            2'd3: begin
                if (burst_cnt <= 0) begin
                    ready_out = ~ready_out;
                    burst_cnt = 1 + (($random(seed_ro)) & 15);
                end
                else burst_cnt = burst_cnt - 1;
            end
        endcase
    end

    // ------------------------------------------------------------------
    // Stimulus helpers. All tasks start and end on a falling clock edge.
    // ------------------------------------------------------------------
    task drive;   // present one transaction, hold it until accepted
        input [3:0] op;
        input [3:0] m;
        input sm;
        input relu;
        input [7:0] aa0, aa1, aa2, aa3;
        input [7:0] bb0, bb1, bb2, bb3;
        input [15:0] cc;
        reg acc;
        begin
            valid_in = 1'b1;
            opcode = op; mask = m; signed_mode = sm; relu_en = relu;
            a0 = aa0; a1 = aa1; a2 = aa2; a3 = aa3;
            b0 = bb0; b1 = bb1; b2 = bb2; b3 = bb3;
            c = cc;
            acc = 1'b0;
            while (!acc) begin
                @(posedge clk);
                #1;
                acc = last_accept;
            end
            @(negedge clk);
        end
    endtask

    task idle;
        input integer n;
        begin
            if (n > 0) begin
                valid_in = 1'b0;
                relu_en  = 1'b0;
                repeat (n) @(negedge clk);
            end
        end
    endtask

    task rnd8;    // random byte, biased toward corner values
        output [7:0] v;
        integer r;
        begin
            r = $random(seed);
            case (r[3:0])
                4'd0:    v = 8'h00;
                4'd1:    v = 8'h01;
                4'd2:    v = 8'h7F;
                4'd3:    v = 8'h80;
                4'd4:    v = 8'hFF;
                4'd5:    v = 8'hFE;
                4'd6:    v = 8'h81;
                default: v = r[15:8];
            endcase
        end
    endtask

    task drive_rand;
        input [3:0] op;
        input [3:0] m;
        input sm;
        input relu;
        reg [7:0] ra0, ra1, ra2, ra3, rb0, rb1, rb2, rb3;
        integer rc;
        begin
            rnd8(ra0); rnd8(ra1); rnd8(ra2); rnd8(ra3);
            rnd8(rb0); rnd8(rb1); rnd8(rb2); rnd8(rb3);
            rc = $random(seed);
            drive(op, m, sm, relu, ra0, ra1, ra2, ra3, rb0, rb1, rb2, rb3, rc[15:0]);
        end
    endtask

    task random_stream;
        input integer n;
        input integer gap_pct;      // chance (%) of inserting idle bubbles
        integer i, r;
        reg [3:0] rop, rm;
        reg rsm, rrl;
        begin
            for (i = 0; i < n; i = i + 1) begin
                r   = $random(seed);
                rop = r[3:0];
                rsm = r[4];
                rrl = r[5];
                rm  = r[9:6];
                if (rm == 4'b0000) rm = r[13:10] | 4'b0001;
                drive_rand(rop, rm, rsm, rrl);
                r = $random(seed);
                if (gap_pct > 0 && ((r & 255) < (gap_pct * 256 / 100)))
                    idle(1 + ((r >> 8) & 3));
            end
        end
    endtask

    task sweep_pairs;   // all 65536 (a,b) pairs, 4 pairs per transaction
        input [3:0] op;
        input sm;
        integer ia, ib, rc;
        begin
            for (ia = 0; ia < 256; ia = ia + 1)
                for (ib = 0; ib < 256; ib = ib + 4) begin
                    rc = $random(seed);
                    drive(op, 4'hF, sm, rc[16],
                          ia, ia+1, ia+2, ia+3,
                          ib, ib+1, ib+2, ib+3, rc[15:0]);
                end
        end
    endtask

    task sweep_unary;   // all 256 values of a (b irrelevant)
        input [3:0] op;
        input sm;
        integer ia;
        begin
            for (ia = 0; ia < 256; ia = ia + 4)
                drive(op, 4'hF, sm, 1'b0, ia, ia+1, ia+2, ia+3, 8'h55, 8'hAA, 8'h0F, 8'hF0, 16'h0);
        end
    endtask

    task do_reset;
        input integer ncyc;
        begin
            valid_in = 1'b0;
            rst = 1'b1;
            flushed_count = flushed_count + q_count;
            q_head = 0; q_tail = 0; q_count = 0;
            hold_pending = 1'b0;
            repeat (ncyc) @(negedge clk);
            rst = 1'b0;
            @(negedge clk);
        end
    endtask

    task drain;
        integer guard;
        begin
            stall_mode = 2'd0;
            valid_in = 1'b0;
            guard = 0;
            @(negedge clk);
            while (q_count != 0 && guard < 200) begin
                @(negedge clk);
                guard = guard + 1;
            end
            repeat (3) @(negedge clk);
        end
    endtask

    // ------------------------------------------------------------------
    // Main sequence
    // ------------------------------------------------------------------
    integer op_i, sm_i, m_i, k_i;
    integer summary_errs;

    initial begin
        seed = 32'h51D_0001;
        seed_ro = 32'h0BAD_CAFE;
        cycle_count = 0; stall_total = 0;
        accepted_count = 0; completed_count = 0; flushed_count = 0; accepted_lanes = 0;
        inflight_max = 0; rst_edges = 0; err_total = 0;
        e_lat = 0; e_data = 0; e_exc = 0; e_mask = 0; e_red = 0; e_hold = 0; e_misc = 0;
        q_head = 0; q_tail = 0; q_count = 0;
        hold_pending = 1'b0; last_accept = 1'b0;
        stall_mode = 2'd0; burst_cnt = 0;
        for (i_init = 0; i_init < 16; i_init = i_init + 1) cov_op[i_init] = 0;
        for (i_init = 0; i_init < 5;  i_init = i_init + 1) cov_exc[i_init] = 0;
        cov_sm[0] = 0; cov_sm[1] = 0;
        phase = "reset";

        valid_in = 0; ready_out = 1; signed_mode = 0; relu_en = 0;
        opcode = K_ADD; mask = 4'b1111;
        a0 = 0; a1 = 0; a2 = 0; a3 = 0; b0 = 0; b1 = 0; b2 = 0; b3 = 0; c = 0;

        rst = 1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst = 0;
        @(negedge clk);

        if (ready_in !== 1'b1 || valid_out !== 1'b0 || lane_valid_out !== 4'b0) begin
            $display("[reset] ERROR: bad state after reset ready_in=%b valid_out=%b lane_valid=%b",
                     ready_in, valid_out, lane_valid_out);
            err_total = err_total + 1; e_misc = e_misc + 1;
        end

        // ---------------------------------------------------------------
        phase = "directed-unsigned";
        //     op          mask sm rl  a0,a1,a2,a3          b0,b1,b2,b3          c
        drive(K_ADD,     4'hF,0,0,   10,20,30,40,         1,2,3,4,             0);
        drive(K_ADD,     4'hF,0,0,   255,200,128,0,       1,100,128,0,         0);
        drive(K_SUB,     4'hF,0,0,   20,30,40,50,         5,10,15,20,          0);
        drive(K_SUB,     4'hF,0,0,   0,5,0,255,           1,10,0,255,          0);
        drive(K_MULT,    4'hF,0,0,   2,3,15,255,          6,7,17,255,          0);
        drive(K_DIV,     4'hF,0,0,   20,30,255,7,         5,6,1,8,             0);
        drive(K_DIV,     4'hF,0,0,   10,20,30,40,         0,0,0,0,             0);
        drive(K_RELU,    4'hF,0,0,   8'hF8,8'h10,8'hFF,2, 0,0,0,0,             0);
        drive(K_LRELU,   4'hF,0,0,   8'hF8,8'h10,8'hFF,8, 0,0,0,0,             0);
        drive(K_MIN,     4'hF,0,0,   5,10,15,20,          8,7,30,10,           0);
        drive(K_MAX,     4'hF,0,0,   5,10,15,20,          8,7,30,10,           0);
        drive(K_ABS,     4'hF,0,0,   8'hF8,8'hFC,5,8'h80, 0,0,0,0,             0);
        drive(K_SATADD,  4'hF,0,0,   250,100,150,255,     20,100,120,10,       0);
        drive(K_SQRT,    4'hF,0,0,   0,1,15,255,          0,0,0,0,             0);
        drive(K_SQRT,    4'hF,0,0,   16,25,36,64,         0,0,0,0,             0);
        drive(K_GCD,     4'hF,0,0,   12,15,9,14,          8,5,6,7,             0);
        drive(K_GCD,     4'hF,0,0,   0,0,255,128,         0,9,0,128,           0);
        drive(K_POPCOUNT,4'hF,0,0,   8'hB5,8'hFF,0,8'hAA, 0,0,0,0,             0);
        drive(K_DOT,     4'hF,0,0,   1,2,3,4,             5,6,7,8,             0);
        drive(K_DOT,     4'hF,0,0,   255,255,255,255,     255,255,255,255,     0);
        drive(K_MAC,     4'hF,0,0,   1,2,3,4,             5,6,7,8,             10);
        drive(K_MAC,     4'hF,0,0,   255,255,255,255,     255,255,255,255,     16'hFFFF);
        drive(K_SAD,     4'hF,0,0,   10,20,30,40,         5,10,15,20,          0);
        drive(K_SAD,     4'hF,0,0,   0,255,0,255,         255,0,255,0,         0);

        phase = "directed-signed";
        drive(K_ADD,     4'hF,1,0,   8'h7F,8'h80,8'hFF,8'h40, 8'h01,8'hFF,8'h01,8'h40, 0);
        drive(K_SUB,     4'hF,1,0,   8'h80,8'h7F,8'h01,8'hF0, 8'h7F,8'h80,8'h01,8'h10, 0);
        drive(K_MULT,    4'hF,1,0,   8'h80,8'hFF,8'h7F,8'hF8, 8'hFF,8'hFF,8'h7F,8'h04, 0);
        drive(K_DIV,     4'hF,1,0,   8'hEC,8'h14,8'hD8,8'h80, 8'h14,8'hEC,8'hFB,8'hFF, 0);
        drive(K_DIV,     4'hF,1,0,   8'hF9,8'h07,8'h7F,8'h80, 8'h02,8'hFE,8'h80,8'h01, 0);
        drive(K_DIV,     4'hF,1,0,   8'h80,8'h7F,8'h01,8'hFF, 0,0,0,0,                 0);
        drive(K_RELU,    4'hF,1,0,   8'hF8,8'h10,8'hFF,8'h80, 0,0,0,0, 0);
        drive(K_LRELU,   4'hF,1,0,   8'hF8,8'h10,8'hFF,8'h80, 0,0,0,0, 0);
        drive(K_MIN,     4'hF,1,0,   8'h80,8'h7F,8'hFF,8'h01, 8'h7F,8'h80,8'h01,8'hFF, 0);
        drive(K_MAX,     4'hF,1,0,   8'h80,8'h7F,8'hFF,8'h01, 8'h7F,8'h80,8'h01,8'hFF, 0);
        drive(K_ABS,     4'hF,1,0,   8'h80,8'hFF,8'h7F,8'h00, 0,0,0,0, 0);
        drive(K_SATADD,  4'hF,1,0,   8'h7F,8'h80,8'h64,8'hC0, 8'h01,8'hFF,8'h64,8'hC0, 0);
        drive(K_SQRT,    4'hF,1,0,   8'h81,8'h7F,8'hF0,8'h10, 0,0,0,0, 0);
        drive(K_GCD,     4'hF,1,0,   8'hD0,8'hF0,8'h80,8'h7B, 8'h0C,8'h0C,8'h18,8'h0F, 0);
        drive(K_GCD,     4'hF,1,0,   8'h80,8'h00,8'hFF,8'h80, 8'h80,8'h00,8'h01,8'h7F, 0);
        drive(K_DOT,     4'hF,1,0,   8'hFE,8'h02,8'hF8,8'h05, 8'h03,8'hFC,8'h04,8'h07, 0);
        drive(K_DOT,     4'hF,1,1,   8'hFE,8'h02,8'hF8,8'h05, 8'h03,8'hFC,8'h04,8'h07, 0);
        drive(K_MAC,     4'hF,1,0,   8'hFE,8'h02,8'hF8,8'h05, 8'h03,8'hFC,8'h04,8'h07, 16'hFFF0);
        drive(K_MAC,     4'hF,1,1,   8'hFE,8'h02,8'hF8,8'h05, 8'h03,8'hFC,8'h04,8'h07, 16'h0100);
        drive(K_MAC,     4'hF,1,0,   8'h80,8'h80,8'h80,8'h80, 8'h7F,8'h7F,8'h7F,8'h7F, 16'h8000);
        drive(K_SAD,     4'hF,1,0,   8'hFE,8'h02,8'hF8,8'h05, 8'h03,8'hFC,8'h04,8'h07, 0);
        drive(K_SAD,     4'hF,1,0,   8'h80,8'h7F,8'h80,8'h7F, 8'h7F,8'h80,8'h7F,8'h80, 0);

        // ---------------------------------------------------------------
        phase = "masks";
        // masked-off lanes deliberately carry exception-provoking data
        drive(K_DIV,  4'b0101, 0,0, 10,20,30,40, 0,0,0,0, 0);
        drive(K_DIV,  4'b0101, 0,0, 10,20,30,40, 2,0,2,0, 0);
        drive(K_ADD,  4'b0010, 0,0, 255,255,255,255, 255,255,255,255, 0);
        drive(K_SQRT, 4'b0110, 1,0, 8'h81,8'h81,8'h04,8'h81, 0,0,0,0, 0);
        for (m_i = 1; m_i < 16; m_i = m_i + 1) begin
            drive_rand(K_ADD, m_i, 0, 0);
            drive_rand(K_DIV, m_i, 1, 0);
            drive_rand(K_DOT, m_i, 1, 1);
            drive_rand(K_MAC, m_i, 0, 0);
            drive_rand(K_MAC, m_i, 1, 0);
            drive_rand(K_SAD, m_i, 1, 0);
        end

        phase = "mask-zero";
        valid_in = 1; mask = 4'b0000; opcode = K_ADD;
        for (k_i = 0; k_i < 3; k_i = k_i + 1) begin
            @(posedge clk); #1;
            if (last_accept || ready_in !== 1'b0) begin
                $display("[mask-zero] ERROR: mask=0 must not be accepted (ready_in=%b)", ready_in);
                err_total = err_total + 1; e_misc = e_misc + 1;
            end
        end
        @(negedge clk);
        valid_in = 0; mask = 4'b1111;
        idle(2);

        // ---------------------------------------------------------------
        phase = "back-to-back";
        for (k_i = 0; k_i < 64; k_i = k_i + 1)
            drive_rand(k_i % 16, 4'hF, (k_i / 16) % 2, k_i[0]);
        drain;

        // ---------------------------------------------------------------
        if (EXHAUSTIVE) begin
            phase = "exhaustive-2op";
            $display("Exhaustive sweep: 11 two-operand opcodes x {unsigned,signed} x 65536 pairs ...");
            for (sm_i = 0; sm_i < 2; sm_i = sm_i + 1) begin
                sweep_pairs(K_ADD,    sm_i);
                sweep_pairs(K_SUB,    sm_i);
                sweep_pairs(K_MULT,   sm_i);
                sweep_pairs(K_DIV,    sm_i);
                sweep_pairs(K_MIN,    sm_i);
                sweep_pairs(K_MAX,    sm_i);
                sweep_pairs(K_SATADD, sm_i);
                sweep_pairs(K_GCD,    sm_i);
                sweep_pairs(K_SAD,    sm_i);
                sweep_pairs(K_DOT,    sm_i);
                sweep_pairs(K_MAC,    sm_i);
            end
            phase = "exhaustive-1op";
            for (sm_i = 0; sm_i < 2; sm_i = sm_i + 1) begin
                sweep_unary(K_RELU,     sm_i);
                sweep_unary(K_LRELU,    sm_i);
                sweep_unary(K_ABS,      sm_i);
                sweep_unary(K_SQRT,     sm_i);
                sweep_unary(K_POPCOUNT, sm_i);
            end
            drain;
        end

        // ---------------------------------------------------------------
        phase = "random-bubbles";
        stall_mode = 2'd0;
        random_stream(N_RANDOM, 30);
        drain;

        phase = "random-stall-75";
        stall_mode = 2'd1;
        random_stream(N_RANDOM, 0);
        drain;

        phase = "random-stall-25";
        stall_mode = 2'd2;
        random_stream(N_RANDOM / 2, 10);
        drain;

        phase = "random-stall-burst";
        stall_mode = 2'd3;
        random_stream(N_RANDOM / 2, 20);
        drain;

        // ---------------------------------------------------------------
        phase = "reset-in-flight";
        stall_mode = 2'd0;
        for (k_i = 0; k_i < 8; k_i = k_i + 1)
            drive_rand(k_i % 16, 4'hF, k_i[0], 0);
        idle(0);
        repeat (3) @(negedge clk);
        do_reset(3);
        idle(20);                       // any stray valid_out is flagged by the monitor
        drive_rand(K_ADD, 4'hF, 0, 0); // must work normally after reset
        drive_rand(K_DOT, 4'hF, 1, 0);
        drain;

        // ---------------------------------------------------------------
        if (completed_count + flushed_count != accepted_count) begin
            $display("ERROR: accepted=%0d completed=%0d flushed=%0d", accepted_count, completed_count, flushed_count);
            err_total = err_total + 1; e_misc = e_misc + 1;
        end

        $display("");
        $display("=====================================================");
        $display(" SIMD PROCESSOR - FULL TESTBENCH SUMMARY");
        $display("=====================================================");
        $display(" Simulated cycles       : %0d", cycle_count);
        $display(" Transactions accepted  : %0d (%0d active lanes)", accepted_count, accepted_lanes);
        $display(" Transactions completed : %0d  (flushed by reset: %0d)", completed_count, flushed_count);
        $display(" Frozen (stall) cycles  : %0d   max in-flight: %0d", stall_total, inflight_max);
        $display(" Signed / unsigned txns : %0d / %0d", cov_sm[1], cov_sm[0]);
        $display(" Opcode coverage (txns) :");
        $display("   ADD %0d SUB %0d MULT %0d DIV %0d MAC %0d DOT %0d RELU %0d LRELU %0d",
                 cov_op[0], cov_op[1], cov_op[2], cov_op[3], cov_op[4], cov_op[5], cov_op[6], cov_op[7]);
        $display("   MIN %0d MAX %0d ABS %0d SATADD %0d SQRT %0d GCD %0d SAD %0d POPCNT %0d",
                 cov_op[8], cov_op[9], cov_op[10], cov_op[11], cov_op[12], cov_op[13], cov_op[14], cov_op[15]);
        $display(" Exceptions observed (lane results): none %0d, divzero %0d, overflow %0d, underflow %0d, invalid %0d",
                 cov_exc[0], cov_exc[1], cov_exc[2], cov_exc[3], cov_exc[4]);
        $display(" Errors: total %0d  [latency %0d, data %0d, exception %0d, mask %0d, reduction %0d, hold %0d, misc %0d]",
                 err_total, e_lat, e_data, e_exc, e_mask, e_red, e_hold, e_misc);
        $display("=====================================================");
        if (err_total == 0) $display(" ALL TESTS PASSED");
        else                $display(" TESTS FAILED: %0d errors", err_total);
        $display("=====================================================");
        #20;
        $finish;
    end

endmodule
