`timescale 1ns/1ps
// ============================================================================
// simd_ref_pkg : golden reference model for the 4-lane SIMD processor.
// Pure SystemVerilog, NO UVM dependency (so it can be unit-tested stand-alone).
//
// Behaviour matches the RTL's documented rules:
//   * unsigned ADD/SUB return the low 8 bits (+ OVERFLOW / UNDER flag)
//   * signed ADD/SUB return the full 9-bit result sign-extended
//   * MULT/DOT/MAC return the full 16-bit product
//   * DIV by zero returns 0 + DIVZERO;  signed -128/-1 returns +128 (no flag)
//
// LRELU: the model implements the CONVENTIONAL leaky ReLU
//        (x>=0 ? x : x>>>3, unsigned = identity).
//        The current RTL computes x>>>3 for EVERY input. To make the model
//        mimic the RTL instead, compile with  +define+SIMD_LRELU_LEGACY
// ============================================================================
package simd_ref_pkg;

  typedef enum bit [3:0] {
    OPC_ADD = 4'd0,  OPC_SUB = 4'd1,   OPC_MULT = 4'd2,   OPC_DIV = 4'd3,
    OPC_MAC = 4'd4,  OPC_DOT = 4'd5,   OPC_RELU = 4'd6,   OPC_LRELU = 4'd7,
    OPC_MIN = 4'd8,  OPC_MAX = 4'd9,   OPC_ABS = 4'd10,   OPC_SATADD = 4'd11,
    OPC_SQRT = 4'd12, OPC_GCD = 4'd13, OPC_SAD = 4'd14,   OPC_POPCOUNT = 4'd15
  } opcode_e;

  typedef enum bit [2:0] {
    EXC_NONE = 3'd0, EXC_DIVZERO = 3'd1, EXC_OVERFLOW = 3'd2,
    EXC_UNDER = 3'd3, EXC_INVALID = 3'd4
  } exc_e;

  typedef bit [7:0] byte_arr_t [4];

  function automatic string ref_op_name(bit [3:0] op);
    opcode_e e;
    e = opcode_e'(op);
    return e.name();
  endfunction

  function automatic string ref_exc_name(bit [2:0] x);
    exc_e e;
    e = exc_e'(x);
    return e.name();
  endfunction

  function automatic int ref_val(bit sm, bit [7:0] x);
    if (sm) return int'($signed(x));
    else    return int'(x);
  endfunction

  function automatic int ref_isqrt(int x);
    int r;
    r = 0;
    while (((r + 1) * (r + 1)) <= x) r++;
    return r;
  endfunction

  function automatic int ref_gcd(int x, int y);
    int t;
    if (x < 0) x = -x;
    if (y < 0) y = -y;
    while (y != 0) begin
      t = x % y;
      x = y;
      y = t;
    end
    return x;
  endfunction

  // ---------------------------------------------------------------- per lane
  function automatic bit [15:0] ref_lane(bit [3:0] op, bit sm, bit [7:0] a, bit [7:0] b);
    int av, bv, res;
    av  = ref_val(sm, a);
    bv  = ref_val(sm, b);
    res = 0;
    case (op)
      OPC_ADD: begin
        res = av + bv;
        if (!sm) res = res & 255;
      end
      OPC_SUB: begin
        res = av - bv;
        if (!sm) res = res & 255;
      end
      OPC_MULT, OPC_DOT, OPC_MAC: res = av * bv;
      OPC_DIV: begin
        if (b == 8'h00) res = 0;
        else            res = av / bv;
      end
      OPC_RELU: res = (sm && av < 0) ? 0 : av;
      OPC_LRELU: begin
`ifdef SIMD_LRELU_LEGACY
        res = sm ? (av >>> 3) : int'(a >> 3);
`else
        res = (sm && av < 0) ? (av >>> 3) : av;
`endif
      end
      OPC_MIN: res = (av < bv) ? av : bv;
      OPC_MAX: res = (av > bv) ? av : bv;
      OPC_ABS: res = (av < 0) ? -av : av;
      OPC_SATADD: begin
        res = av + bv;
        if (sm) begin
          if (res > 127)  res = 127;
          if (res < -128) res = -128;
        end
        else begin
          if (res > 255) res = 255;
        end
      end
      OPC_SQRT: res = (sm && av < 0) ? 0 : ref_isqrt(av);
      OPC_GCD:  res = ref_gcd(av, bv);
      OPC_POPCOUNT: res = $countones(a);
      OPC_SAD: begin
        res = av - bv;
        if (res < 0) res = -res;
      end
      default: res = 0;
    endcase
    return 16'(res);
  endfunction

  function automatic bit [2:0] ref_exc(bit [3:0] op, bit sm, bit [7:0] a, bit [7:0] b);
    int av, bv, t;
    bit [2:0] r;
    av = ref_val(sm, a);
    bv = ref_val(sm, b);
    r  = EXC_NONE;
    case (op)
      OPC_DIV: begin
        if (b == 8'h00) r = EXC_DIVZERO;
      end
      OPC_ADD, OPC_SATADD: begin
        t = av + bv;
        if (sm) begin
          if (t > 127 || t < -128) r = EXC_OVERFLOW;
        end
        else begin
          if (t > 255) r = EXC_OVERFLOW;
        end
      end
      OPC_SUB: begin
        t = av - bv;
        if (sm) begin
          if (t > 127 || t < -128) r = EXC_OVERFLOW;
        end
        else begin
          if (a < b) r = EXC_UNDER;
        end
      end
      OPC_MULT: begin
        t = av * bv;
        if (sm) begin
          if (t > 127 || t < -128) r = EXC_OVERFLOW;
        end
        else begin
          if (t > 255) r = EXC_OVERFLOW;
        end
      end
      OPC_SQRT: begin
        if (sm && av < 0) r = EXC_INVALID;
      end
      default: ;
    endcase
    return r;
  endfunction

  // --------------------------------------------------------------- reductions
  function automatic bit is_reduction(bit [3:0] op);
    return (op == OPC_DOT) || (op == OPC_MAC) || (op == OPC_SAD);
  endfunction

  function automatic int ref_prod(bit sm, bit relu, bit [7:0] a, bit [7:0] b);
    int p;
    p = ref_val(sm, a) * ref_val(sm, b);
    if (relu && sm && p < 0) p = 0;
    return p;
  endfunction

  function automatic int ref_absdiff(bit sm, bit [7:0] a, bit [7:0] b);
    int d;
    d = ref_val(sm, a) - ref_val(sm, b);
    return (d < 0) ? -d : d;
  endfunction

  function automatic bit [18:0] ref_dot(bit sm, bit relu, bit [3:0] mask,
                                        byte_arr_t a, byte_arr_t b);
    int s;
    s = 0;
    for (int i = 0; i < 4; i++)
      if (mask[i]) s += ref_prod(sm, relu, a[i], b[i]);
    return 19'(s);
  endfunction

  function automatic bit [18:0] ref_mac(bit sm, bit relu, bit [3:0] mask,
                                        byte_arr_t a, byte_arr_t b, bit [15:0] c);
    int s;
    s = 0;
    for (int i = 0; i < 4; i++)
      if (mask[i]) s += ref_prod(sm, relu, a[i], b[i]);
    if (sm) s += int'($signed(c));
    else    s += int'(c);
    return 19'(s);
  endfunction

  function automatic bit [18:0] ref_sad(bit sm, bit [3:0] mask,
                                        byte_arr_t a, byte_arr_t b);
    int s;
    s = 0;
    for (int i = 0; i < 4; i++)
      if (mask[i]) s += ref_absdiff(sm, a[i], b[i]);
    return 19'(s);
  endfunction

endpackage
