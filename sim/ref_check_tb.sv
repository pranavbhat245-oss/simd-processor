`timescale 1ns/1ps
// Stand-alone sanity bench: proves simd_ref_pkg agrees with the DUT before it
// is trusted as the UVM scoreboard's golden model.  Runs in Verilator/Questa.
module ref_check_tb;
  import simd_ref_pkg::*;

  localparam int N_RAND = 60000;

  logic clk = 0;
  always #5 clk = ~clk;

  logic rst = 1, valid_in = 0, ready_out = 1, signed_mode = 0, relu_en = 0;
  logic [3:0]  opcode = 0, mask = 4'hF;
  logic [7:0]  a0 = 0, a1 = 0, a2 = 0, a3 = 0, b0 = 0, b1 = 0, b2 = 0, b3 = 0;
  logic [15:0] c = 0;
  wire         ready_in, valid_out, reduction_valid_out, signed_mode_out;
  wire [15:0]  l0, l1, l2, l3;
  wire [3:0]   lv, opo, dz, ov, un, si, ex;
  wire [18:0]  dot, mac, sad;
  wire [2:0]   e0, e1, e2, e3;

  SimdTop dut (
    .clk(clk), .rst(rst), .valid_in(valid_in), .ready_in(ready_in), .ready_out(ready_out),
    .signed_mode(signed_mode), .relu_en(relu_en), .opcode(opcode), .mask(mask),
    .a0(a0), .a1(a1), .a2(a2), .a3(a3), .b0(b0), .b1(b1), .b2(b2), .b3(b3), .c(c),
    .lane0_result(l0), .lane1_result(l1), .lane2_result(l2), .lane3_result(l3),
    .lane_valid_out(lv), .dot_result(dot), .mac_result(mac), .sad_result(sad),
    .reduction_valid_out(reduction_valid_out), .valid_out(valid_out),
    .opcode_out(opo), .signed_mode_out(signed_mode_out),
    .div_by_zero(dz), .overflow(ov), .underflow(un), .sqrt_invalid(si), .exception_out(ex),
    .exception_code0(e0), .exception_code1(e1), .exception_code2(e2), .exception_code3(e3));

  int errs = 0, ntx = 0;
  int per_op [16];

  function automatic bit [7:0] rnd8();
    int r;
    r = $urandom_range(15, 0);
    case (r)
      0: return 8'h00;  1: return 8'h01;  2: return 8'h7F;  3: return 8'h80;
      4: return 8'hFF;  5: return 8'hFE;  6: return 8'h81;
      default: return 8'($urandom());
    endcase
  endfunction

  task automatic run_one(bit [3:0] op, bit sm, bit rl, bit [3:0] m,
                         byte_arr_t aa, byte_arr_t bb, bit [15:0] cc);
    logic [15:0] got [4];
    logic [2:0]  gcode [4];
    logic [3:0]  gdz, gov, gun, gsi, gex;
    bit [15:0]   exp_l;
    bit [2:0]    exp_x;
    @(negedge clk);
    valid_in = 1; opcode = op; signed_mode = sm; relu_en = rl; mask = m;
    a0 = aa[0]; a1 = aa[1]; a2 = aa[2]; a3 = aa[3];
    b0 = bb[0]; b1 = bb[1]; b2 = bb[2]; b3 = bb[3]; c = cc;
    @(negedge clk);
    valid_in = 0; relu_en = 0;
    repeat (10) @(negedge clk);

    ntx++; per_op[op]++;
    if (valid_out !== 1'b1 || opo !== op || lv !== m) begin
      errs++; if (errs < 20) $display("CTRL mismatch op=%0d valid=%b opo=%0d lv=%b exp_mask=%b", op, valid_out, opo, lv, m);
    end
    got[0] = l0; got[1] = l1; got[2] = l2; got[3] = l3;
    gcode[0] = e0; gcode[1] = e1; gcode[2] = e2; gcode[3] = e3;
    gdz = dz; gov = ov; gun = un; gsi = si; gex = ex;
    for (int i = 0; i < 4; i++) begin
      if (m[i]) begin
        exp_l = ref_lane(op, sm, aa[i], bb[i]);
        exp_x = ref_exc(op, sm, aa[i], bb[i]);
        if (got[i] !== exp_l) begin
          errs++; if (errs < 20) $display("DATA lane%0d op=%s sm=%0b a=%02h b=%02h exp=%04h got=%04h", i, ref_op_name(op), sm, aa[i], bb[i], exp_l, got[i]);
        end
        if (gcode[i] !== exp_x ||
            gdz[i] !== (exp_x == EXC_DIVZERO) || gov[i] !== (exp_x == EXC_OVERFLOW) ||
            gun[i] !== (exp_x == EXC_UNDER)   || gsi[i] !== (exp_x == EXC_INVALID) ||
            gex[i] !== (exp_x != EXC_NONE)) begin
          errs++; if (errs < 20) $display("EXC  lane%0d op=%s sm=%0b a=%02h b=%02h exp=%s got=%0d flags=%b%b%b%b%b", i, ref_op_name(op), sm, aa[i], bb[i], ref_exc_name(exp_x), gcode[i], gdz[i], gov[i], gun[i], gsi[i], gex[i]);
        end
      end
      else if (gcode[i] !== 3'd0 || gdz[i] !== 1'b0 || gov[i] !== 1'b0 || gun[i] !== 1'b0 || gsi[i] !== 1'b0 || gex[i] !== 1'b0) begin
        errs++; if (errs < 20) $display("MASKED lane%0d not quiet op=%s code=%0d", i, ref_op_name(op), gcode[i]);
      end
    end
    if (is_reduction(op)) begin
      if (reduction_valid_out !== 1'b1) begin errs++; if (errs < 20) $display("RED valid low op=%s", ref_op_name(op)); end
      case (op)
        OPC_DOT: if (dot !== ref_dot(sm, rl, m, aa, bb))     begin errs++; if (errs < 20) $display("DOT exp=%0h got=%0h", ref_dot(sm, rl, m, aa, bb), dot); end
        OPC_MAC: if (mac !== ref_mac(sm, rl, m, aa, bb, cc)) begin errs++; if (errs < 20) $display("MAC exp=%0h got=%0h", ref_mac(sm, rl, m, aa, bb, cc), mac); end
        OPC_SAD: if (sad !== ref_sad(sm, m, aa, bb))         begin errs++; if (errs < 20) $display("SAD exp=%0h got=%0h", ref_sad(sm, m, aa, bb), sad); end
        default: ;
      endcase
    end
    else if (reduction_valid_out !== 1'b0) begin
      errs++; if (errs < 20) $display("RED valid high for op=%s", ref_op_name(op));
    end
  endtask

  initial begin
    byte_arr_t aa, bb;
    bit [3:0] m;
    repeat (4) @(posedge clk);
    #1 rst = 0;
    for (int n = 0; n < N_RAND; n++) begin
      for (int i = 0; i < 4; i++) begin aa[i] = rnd8(); bb[i] = rnd8(); end
      m = 4'($urandom_range(15, 1));
      run_one(4'(n % 16), 1'($urandom()), 1'($urandom()), m, aa, bb, 16'($urandom()));
    end
    begin
      int mn;
      mn = per_op[0];
      for (int k = 1; k < 16; k++) if (per_op[k] < mn) mn = per_op[k];
      $display("ref_check_tb: %0d transactions, min txns per opcode=%0d, errors=%0d", ntx, mn, errs);
    end
    if (errs == 0) $display("REF MODEL MATCHES DUT");
    else           $display("REF MODEL MISMATCH");
    $finish;
  end
endmodule
